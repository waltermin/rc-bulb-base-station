// base_main.c — RC light base station (ESP32 / ESP-IDF).
//
// Injects raw 802.11 beacon frames carrying the RC light control protocol
// (vendor-specific IE, OUI 52:43:68, as the ONLY IE) via esp_wifi_80211_tx().
// A serial REPL over USB lets you set per-bulb colors and trigger DFU.
//
// Commands (type `help`):
//   set <id> <r> <g> <b> <ww> <cw>   add/update a bulb entry (values 0..255)
//   clr <id>                         remove one bulb entry
//   clrall                           remove all entries
//   dfu <id>                         request DFU mode for a bulb (burst)
//   chan <1..13>                     set the broadcast channel
//   interval <ms>                    set the beacon interval
//   start | stop                     resume / pause broadcasting
//   show                             print current state
//
// The bulb firmware expects channel 1 by default (WIFI_CHANNEL in its config.h).

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_console.h"

static const char *TAG = "base";

// ---- protocol wire constants (must match the bulb firmware) -----------------
#define OUI0 0x52
#define OUI1 0x43
#define OUI2 0x68
#define PROTO_VERSION 0x01
#define CTRL_FLAG_DFU 0x01
#define MAX_ENTRIES 11
#define ENTRY_SIZE 6

// ---- shared broadcast state -------------------------------------------------
typedef struct {
    bool used;
    uint8_t id, r, g, b, ww, cw;
} entry_t;

static entry_t s_entries[MAX_ENTRIES];
static uint8_t s_control_flags = 0;
static uint8_t s_control_data = 0;
static int s_dfu_bursts_left = 0;   // while >0, keep control_flags=DFU then clear
static int s_channel = 1;
static int s_interval_ms = 200;
static bool s_enabled = true;
static uint8_t s_base_mac[6];
static SemaphoreHandle_t s_lock;

#define LOCK() xSemaphoreTake(s_lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_lock)

// ---- beacon frame builder ---------------------------------------------------
// Builds a complete 802.11 beacon (no FCS; hardware appends it) with our single
// vendor IE. Caller must hold the lock. Returns frame length.
static int build_beacon(uint8_t *p) {
    int n = 0;

    // MAC header (24 bytes)
    p[n++] = 0x80; p[n++] = 0x00;                 // frame control: beacon
    p[n++] = 0x00; p[n++] = 0x00;                 // duration
    for (int i = 0; i < 6; i++) p[n++] = 0xFF;    // DA: broadcast
    memcpy(p + n, s_base_mac, 6); n += 6;         // SA (our own MAC)
    memcpy(p + n, s_base_mac, 6); n += 6;         // BSSID (our own MAC)
    p[n++] = 0x00; p[n++] = 0x00;                 // seq/frag (system overwrites)

    // Beacon fixed params (12 bytes)
    for (int i = 0; i < 8; i++) p[n++] = 0x00;    // timestamp
    p[n++] = 0x64; p[n++] = 0x00;                 // beacon interval (100 TU)
    p[n++] = 0x00; p[n++] = 0x00;                 // capability info

    // LightUpdatePacket body
    uint8_t body[4 + MAX_ENTRIES * ENTRY_SIZE];
    int b = 0;
    body[b++] = PROTO_VERSION;
    body[b++] = s_control_flags;
    body[b++] = s_control_data;
    int count_idx = b++;                          // entry_count placeholder
    int count = 0;
    for (int i = 0; i < MAX_ENTRIES; i++) {
        if (!s_entries[i].used) continue;
        body[b++] = s_entries[i].id;
        body[b++] = s_entries[i].r;
        body[b++] = s_entries[i].g;
        body[b++] = s_entries[i].b;
        body[b++] = s_entries[i].ww;
        body[b++] = s_entries[i].cw;
        count++;
    }
    body[count_idx] = (uint8_t)count;

    // Vendor-specific IE (the ONLY IE)
    p[n++] = 0xDD;
    p[n++] = (uint8_t)(3 + b);                    // len = OUI + body
    p[n++] = OUI0; p[n++] = OUI1; p[n++] = OUI2;
    memcpy(p + n, body, b); n += b;

    return n;
}

// ---- broadcast task ---------------------------------------------------------
static void broadcast_task(void *arg) {
    (void)arg;
    static uint8_t frame[128];
    int applied_channel = -1;

    for (;;) {
        int interval;
        bool enabled;
        int len = 0;

        LOCK();
        enabled = s_enabled;
        interval = s_interval_ms;
        if (applied_channel != s_channel) {
            esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE);
            applied_channel = s_channel;
        }
        if (enabled) {
            len = build_beacon(frame);
            // DFU is a limited burst: count it down, then stop requesting it.
            if (s_control_flags == CTRL_FLAG_DFU && s_dfu_bursts_left > 0) {
                if (--s_dfu_bursts_left == 0) {
                    s_control_flags = 0;
                    s_control_data = 0;
                }
            }
        }
        UNLOCK();

        if (enabled && len > 0) {
            esp_err_t e = esp_wifi_80211_tx(WIFI_IF_STA, frame, len, true);
            if (e != ESP_OK) {
                ESP_LOGW(TAG, "80211_tx failed: %s", esp_err_to_name(e));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(interval));
    }
}

// ---- console command helpers ------------------------------------------------
static bool parse_u8(const char *s, uint8_t *out) {
    char *end;
    long v = strtol(s, &end, 0);
    if (*end != '\0' || v < 0 || v > 255) return false;
    *out = (uint8_t)v;
    return true;
}

static int cmd_set(int argc, char **argv) {
    if (argc != 7) {
        printf("usage: set <id> <r> <g> <b> <ww> <cw>\n");
        return 1;
    }
    uint8_t v[6];
    for (int i = 0; i < 6; i++) {
        if (!parse_u8(argv[i + 1], &v[i])) {
            printf("bad value '%s' (0..255)\n", argv[i + 1]);
            return 1;
        }
    }
    LOCK();
    int slot = -1, free_slot = -1;
    for (int i = 0; i < MAX_ENTRIES; i++) {
        if (s_entries[i].used && s_entries[i].id == v[0]) { slot = i; break; }
        if (!s_entries[i].used && free_slot < 0) free_slot = i;
    }
    if (slot < 0) slot = free_slot;
    if (slot < 0) {
        UNLOCK();
        printf("no free entry slots (max %d)\n", MAX_ENTRIES);
        return 1;
    }
    s_entries[slot] = (entry_t){.used = true, .id = v[0], .r = v[1], .g = v[2],
                                .b = v[3], .ww = v[4], .cw = v[5]};
    UNLOCK();
    printf("set bulb %d -> r%d g%d b%d ww%d cw%d\n", v[0], v[1], v[2], v[3], v[4], v[5]);
    return 0;
}

static int cmd_clr(int argc, char **argv) {
    if (argc != 2) { printf("usage: clr <id>\n"); return 1; }
    uint8_t id;
    if (!parse_u8(argv[1], &id)) { printf("bad id\n"); return 1; }
    LOCK();
    int found = 0;
    for (int i = 0; i < MAX_ENTRIES; i++)
        if (s_entries[i].used && s_entries[i].id == id) { s_entries[i].used = false; found = 1; }
    UNLOCK();
    printf(found ? "cleared bulb %d\n" : "no entry for bulb %d\n", id);
    return 0;
}

static int cmd_clrall(int argc, char **argv) {
    (void)argc; (void)argv;
    LOCK();
    memset(s_entries, 0, sizeof(s_entries));
    UNLOCK();
    printf("cleared all entries\n");
    return 0;
}

static int cmd_dfu(int argc, char **argv) {
    if (argc != 2) { printf("usage: dfu <id>\n"); return 1; }
    uint8_t id;
    if (!parse_u8(argv[1], &id)) { printf("bad id\n"); return 1; }
    LOCK();
    s_control_flags = CTRL_FLAG_DFU;
    s_control_data = id;
    s_dfu_bursts_left = 25;  // ~5 s at 200 ms
    UNLOCK();
    printf("requesting DFU for bulb %d (burst)\n", id);
    return 0;
}

static int cmd_chan(int argc, char **argv) {
    if (argc != 2) { printf("usage: chan <1..13>\n"); return 1; }
    int ch = atoi(argv[1]);
    if (ch < 1 || ch > 13) { printf("channel out of range\n"); return 1; }
    LOCK(); s_channel = ch; UNLOCK();
    printf("channel = %d\n", ch);
    return 0;
}

static int cmd_interval(int argc, char **argv) {
    if (argc != 2) { printf("usage: interval <ms>\n"); return 1; }
    int ms = atoi(argv[1]);
    if (ms < 20 || ms > 10000) { printf("interval out of range (20..10000)\n"); return 1; }
    LOCK(); s_interval_ms = ms; UNLOCK();
    printf("interval = %d ms\n", ms);
    return 0;
}

static int cmd_start(int argc, char **argv) {
    (void)argc; (void)argv;
    LOCK(); s_enabled = true; UNLOCK();
    printf("broadcasting\n");
    return 0;
}

static int cmd_stop(int argc, char **argv) {
    (void)argc; (void)argv;
    LOCK(); s_enabled = false; UNLOCK();
    printf("stopped\n");
    return 0;
}

static int cmd_show(int argc, char **argv) {
    (void)argc; (void)argv;
    LOCK();
    printf("state: %s  channel %d  interval %d ms  control_flags %d control_data %d\n",
           s_enabled ? "ON" : "OFF", s_channel, s_interval_ms, s_control_flags, s_control_data);
    printf("base MAC (frame src): %02x:%02x:%02x:%02x:%02x:%02x\n",
           s_base_mac[0], s_base_mac[1], s_base_mac[2], s_base_mac[3], s_base_mac[4], s_base_mac[5]);
    int count = 0;
    for (int i = 0; i < MAX_ENTRIES; i++) {
        if (!s_entries[i].used) continue;
        printf("  bulb %3d : r%3d g%3d b%3d ww%3d cw%3d\n", s_entries[i].id,
               s_entries[i].r, s_entries[i].g, s_entries[i].b, s_entries[i].ww, s_entries[i].cw);
        count++;
    }
    if (!count) printf("  (no entries)\n");
    UNLOCK();
    return 0;
}

#if 0  // benchmark shelved for now; re-enable to profile max TX rate
// Max-rate injection throughput test. Pauses normal broadcasting, then calls
// esp_wifi_80211_tx() as fast as the driver will accept for N seconds. Each OK
// return is one frame queued for transmit; when the TX buffers fill the driver
// returns an error, so we yield to let them drain (which is also when the frames
// actually go out). ok/elapsed therefore = the real sustained TX rate.
static int cmd_bench(int argc, char **argv) {
    int secs = (argc >= 2) ? atoi(argv[1]) : 3;
    if (secs < 1 || secs > 20) secs = 3;

    LOCK();
    bool was_enabled = s_enabled;
    s_enabled = false;                 // stop the periodic broadcaster competing
    static uint8_t frame[128];
    int len = build_beacon(frame);     // benchmark the current entry table
    UNLOCK();
    vTaskDelay(pdMS_TO_TICKS(50));      // let an in-flight broadcast finish

    printf("bench: hammering esp_wifi_80211_tx for %d s (frame %d bytes)...\n", secs, len);

    uint32_t ok = 0, fail = 0;
    int64_t t0 = esp_timer_get_time();
    int64_t tend = t0 + (int64_t)secs * 1000000;
    int64_t last_yield = t0, now;
    while ((now = esp_timer_get_time()) < tend) {
        // Retry immediately on a full queue: the frames drain via the WiFi
        // ISR/task (higher priority, preempts us), so tight-spinning keeps the
        // TX queue full and the radio busy. Yield ~every 10 ms (1 tick @ 1 kHz)
        // only so the idle task runs and the watchdog stays fed.
        esp_err_t e = esp_wifi_80211_tx(WIFI_IF_STA, frame, len, true);
        if (e == ESP_OK) ok++;
        else fail++;
        if (now - last_yield >= 10000) {
            last_yield = now;
            vTaskDelay(1);
        }
    }
    double sec = (esp_timer_get_time() - t0) / 1e6;

    printf("bench: %u sent, %u queue-full retries in %.3f s\n",
           (unsigned)ok, (unsigned)fail, sec);
    printf("bench: ==> %.0f packets/sec (%d-byte frames, %.1f kbit/s payload)\n",
           ok / sec, len, (ok / sec) * len * 8.0 / 1000.0);

    LOCK(); s_enabled = was_enabled; UNLOCK();
    return 0;
}
#endif  // benchmark shelved

static void register_commands(void) {
    // Designated initializers so this stays correct across ESP-IDF versions
    // that add fields (hint, argtable, func_w_context, ...) to esp_console_cmd_t.
    const esp_console_cmd_t cmds[] = {
        {.command = "set",      .help = "set <id> <r> <g> <b> <ww> <cw> : add/update a bulb entry", .func = &cmd_set},
        {.command = "clr",      .help = "clr <id> : remove one bulb entry", .func = &cmd_clr},
        {.command = "clrall",   .help = "remove all bulb entries", .func = &cmd_clrall},
        {.command = "dfu",      .help = "dfu <id> : request DFU mode for a bulb", .func = &cmd_dfu},
        {.command = "chan",     .help = "chan <1..13> : set broadcast channel", .func = &cmd_chan},
        {.command = "interval", .help = "interval <ms> : set beacon interval", .func = &cmd_interval},
        {.command = "start",    .help = "resume broadcasting", .func = &cmd_start},
        {.command = "stop",     .help = "pause broadcasting", .func = &cmd_stop},
        {.command = "show",     .help = "print current state", .func = &cmd_show},
        // {.command = "bench", .help = "bench [seconds] : max-rate TX throughput test", .func = &cmd_bench},
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
}

// ---- wifi bring-up (STA, started, not connected) ----------------------------
static void wifi_init_tx(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Promiscuous keeps the radio parked on our fixed channel (no scans) and is
    // the reliable state for raw injection; we register no RX callback.
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_base_mac));
    // Never connect: we only inject frames.
}

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock != NULL);

    wifi_init_tx();

    xTaskCreate(broadcast_task, "broadcast", 4096, NULL, 5, NULL);

    // Serial REPL over the USB UART.
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "base>";
    repl_config.max_cmdline_length = 128;
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_register_help_command());
    register_commands();

    ESP_LOGI(TAG, "RC light base station ready. Type 'help'. Broadcasting on channel %d.",
             s_channel);
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
