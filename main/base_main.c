// base_main.c — RC light base station (ESP32 / ESP-IDF).
//
// Injects raw 802.11 beacon frames carrying the RC light control protocol
// (vendor-specific IE, OUI 52:43:68, as the ONLY IE) via esp_wifi_80211_tx().
// A serial REPL over USB lets you set per-bulb colors and trigger DFU.
//
// Commands (type `help`):
//   set <id> <r> <g> <b> <ww> <cw>   add/update a bulb entry (values 0..255)
//   setrange <first> <last> <r> <g> <b> <ww> <cw>
//                                    add/update entries for every id in
//                                    first..last (inclusive) with one color
//   scaledset <id> <r> <g> <b> <ww> <cw> <scale>
//                                    PreciseLightUpdate: channels scaled by
//                                    scale% and sent as floats (one bulb at a time)
//   clr <id>                         remove one bulb entry
//   clrall                           remove all entries
//   dfu <id> [new|legacy]            request DFU (new=BulbCommand, legacy=0x01)
//   dfurange <first> <last>          DFU every id in first..last, one BulbCommand
//                                    burst per bulb in sequence, with progress
//   setconfig <id> <key> <type> <value>
//                                    set a bulb config key via BulbCommand
//   seq [value]                      show / re-seed the BulbCommand seq counter
//   sweep <id> <channels> <ms>       linear 0->255->0 loop over the channels
//   sweeprange <first> <last> <channels> <ms>
//                                    same sweep applied in lockstep to every
//                                    id in first..last (inclusive)
//   psweep <id> <channels> <ms> <min> <max>
//                                    precise float sweep bouncing between
//                                    min%..max% of full scale (one bulb at a time)
//   stopsweep                        stop the active sweep(s)
//   chan <1..13>                     set the broadcast channel
//   rate <1|2|5.5|11|6|9|12|18|24|36|48|54>
//                                    set the PHY rate frames are injected at
//   interval <ms>                    set the beacon interval
//   start | stop                     resume / pause broadcasting
//   show                             print current state
//
// Packet types on the wire (selected by the first payload byte, a packet tag):
//   0x02 PreciseLightUpdate — one bulb's float channels (scaledset / psweep)
//   0x03 LightUpdateV2      — the u8 entry table (set / sweep), continuously
//   0x04 BulbCommand        — DFU + config management, sent as a burst
//   0x01 LightUpdate        — deprecated; only emitted by `dfu <id> legacy`
// The entry table (0x03) holds up to MAX_ENTRIES bulbs, but a bulb only accepts
// ENTRIES_PER_PACKET entries per packet, so each tick the table is sharded into
// ceil(n / ENTRIES_PER_PACKET) back-to-back beacons. The entry table and any
// active precise target (0x02) are broadcast continuously; BulbCommands (0x04) and legacy DFU (0x01) are one-shot bursts
// (CMD_BURST_COUNT frames at CMD_BURST_INTERVAL_MS) so a momentary miss is
// unlikely. Each BulbCommand carries a monotonically increasing seq; the bulb
// acts on the first sighting of a new-highest seq (anti-replay).
//
// The bulb firmware expects channel 11 by default (WIFI_CHANNEL in its config.h).

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

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
#define TAG_LIGHT_UPDATE 0x01           // packet tag: (deprecated) u8 entry table + DFU
#define TAG_PRECISE_LIGHT_UPDATE 0x02   // packet tag: one bulb, f32 channels
#define TAG_LIGHT_UPDATE_V2 0x03        // packet tag: u8 entry table, no DFU
#define TAG_BULB_COMMAND 0x04           // packet tag: remote management (config / DFU)
#define CTRL_FLAG_DFU 0x01              // legacy 0x01 control_flags value for DFU
#define CMD_ENTER_DFU 0x00              // BulbCommand cmd: enter DFU (no payload)
#define CMD_SET_CONFIG 0x01             // BulbCommand cmd: set config (key u16, len u8, value)
#define MAX_ENTRIES 64                  // size of our entry table (sharded on the wire)
#define ENTRIES_PER_PACKET 11           // bulb-side limit per packet (PROTO_MAX_ENTRIES)
#define ENTRY_SIZE 6
#define MAX_SHARDS ((MAX_ENTRIES + ENTRIES_PER_PACKET - 1) / ENTRIES_PER_PACKET)
#define SHARD_FRAME_SIZE 128            // >= 36 hdr + 5 IE hdr + 2 + 11*6 = 109
#define PRECISE_BODY_SIZE (2 + 5 * 4)   // tag + bulb_id + 5 * f32
#define CMD_VALUE_MAX 64                // max SetConfig value bytes (matches bulb PROTO_CONFIG_VALUE_MAX)

// Every BulbCommand frame is sent this many times, this far apart, to ensure
// delivery (the bulb acts on the first sighting of a new-highest seq).
#define CMD_BURST_COUNT 10
#define CMD_BURST_INTERVAL_MS 100

// ---- color channel bits (for `sweep`) ---------------------------------------
#define CH_R  0x01
#define CH_G  0x02
#define CH_B  0x04
#define CH_WW 0x08
#define CH_CW 0x10

// ---- shared broadcast state -------------------------------------------------
typedef struct {
    bool used;
    uint8_t id, r, g, b, ww, cw;
} entry_t;

static entry_t s_entries[MAX_ENTRIES];

// One active linear triangle-wave sweep: 0 -> 255 -> 0 over `period_ms`, looping
// until stopped. Applied in lockstep to the `mask`ed channels of every bulb with
// id in first..last (inclusive) each broadcast tick (`sweep` sets first==last).
typedef struct {
    bool active;
    uint8_t first, last;
    uint8_t mask;        // OR of CH_* bits
    int period_ms;       // full 0->255->0 cycle
    int64_t start_us;    // esp_timer_get_time() when the sweep began
} sweep_t;

static sweep_t s_sweep;

// The single active PreciseLightUpdate target (only one bulb can be precisely
// controlled at a time — a new scaledset/psweep replaces the previous one).
// When active, it is broadcast as its own 0x02 beacon each tick.
typedef struct {
    bool active;
    uint8_t id;
    float r, g, b, ww, cw;   // normalized [0,1]
} precise_t;

static precise_t s_precise;

// Optional triangle-wave sweep driving the precise target's channels between
// [min,max] (normalized). Writes into s_precise each broadcast tick.
typedef struct {
    bool active;
    uint8_t mask;        // OR of CH_* bits
    int period_ms;       // full min->max->min cycle
    int64_t start_us;
    float min, max;      // normalized [0,1], min <= max
} psweep_t;

static psweep_t s_psweep;

static int s_channel = 11;
static int s_interval_ms = 200;

// PHY rate for injected frames. The default for esp_wifi_80211_tx is the
// 1 Mbps basic rate, where a ~110-byte shard costs ~1.3 ms of air; at 55+ bulbs
// and a 10 ms interval that saturates the channel and the driver's TX pool
// overflows (ESP_ERR_NO_MEM). 11 Mbps (802.11b, long preamble) cuts per-frame
// airtime ~4x and is still a legacy rate the bulb's sniffer handles.
typedef struct { const char *name; wifi_phy_rate_t rate; } rate_opt_t;
static const rate_opt_t s_rate_opts[] = {
    {"1",   WIFI_PHY_RATE_1M_L},  {"2",  WIFI_PHY_RATE_2M_L},  {"5.5", WIFI_PHY_RATE_5M_L},
    {"11",  WIFI_PHY_RATE_11M_L}, {"6",  WIFI_PHY_RATE_6M},    {"9",   WIFI_PHY_RATE_9M},
    {"12",  WIFI_PHY_RATE_12M},   {"18", WIFI_PHY_RATE_18M},   {"24",  WIFI_PHY_RATE_24M},
    {"36",  WIFI_PHY_RATE_36M},   {"48", WIFI_PHY_RATE_48M},   {"54",  WIFI_PHY_RATE_54M},
};
#define N_RATE_OPTS ((int)(sizeof(s_rate_opts) / sizeof(s_rate_opts[0])))
#define DEFAULT_RATE_IDX 3   // 11 Mbps
static int s_rate_idx = DEFAULT_RATE_IDX;
static bool s_enabled = true;
static uint8_t s_base_mac[6];
static SemaphoreHandle_t s_lock;

// BulbCommand sequence counter. Starts at 0; each issued command uses the
// current value then increments (the bulb only acts on a new-highest seq).
// Re-seedable via the `seq` command.
static uint32_t s_seq = 0;

// One-shot burst jobs: a fully-built frame to transmit `count` times at
// `interval_ms`. Used for BulbCommands and legacy DFU. Serviced by burst_task so
// the timing is independent of the continuous entry-table broadcast cadence.
typedef struct {
    uint8_t frame[160];
    int len;
    int count;
    int interval_ms;
    SemaphoreHandle_t done;   // optional: given by burst_task when the last frame is out
} burst_job_t;

static QueueHandle_t s_burst_queue;

// dfurange progress: a background task walks first..last issuing one DFU
// BulbCommand burst per bulb, waiting for each to finish. Only one at a time.
static bool s_dfurange_active;

#define LOCK() xSemaphoreTake(s_lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_lock)

// ---- beacon frame builder ---------------------------------------------------
// Write the 36-byte 802.11 MAC header + beacon fixed params common to every
// frame we send. Returns the offset just past them. Caller must hold the lock.
static int write_beacon_header(uint8_t *p) {
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
    return n;
}

// Append our single vendor-specific IE (0xDD, OUI 52:43:68) carrying `body` at
// offset `n`. `body` starts with the packet tag. Returns the new offset.
static int write_vendor_ie(uint8_t *p, int n, const uint8_t *body, int blen) {
    p[n++] = 0xDD;
    p[n++] = (uint8_t)(3 + blen);                 // len = OUI + body
    p[n++] = OUI0; p[n++] = OUI1; p[n++] = OUI2;
    memcpy(p + n, body, blen); n += blen;
    return n;
}

// Build one 0x03 LightUpdateV2 beacon (no FCS; hardware appends it) with our
// single vendor IE, carrying up to ENTRIES_PER_PACKET used entries starting the
// scan at table index *pos. Advances *pos past the entries consumed. Caller must
// hold the lock. Returns frame length.
static int build_beacon_shard(uint8_t *p, int *pos) {
    int n = write_beacon_header(p);

    // LightUpdateV2 body (tag 0x03): tag, entry_count, then the entry table.
    // (DFU moved out to BulbCommand, so there are no control fields here.)
    uint8_t body[2 + ENTRIES_PER_PACKET * ENTRY_SIZE];
    int b = 0;
    body[b++] = TAG_LIGHT_UPDATE_V2;
    int count_idx = b++;                          // entry_count placeholder
    int count = 0;
    int i = *pos;
    for (; i < MAX_ENTRIES && count < ENTRIES_PER_PACKET; i++) {
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
    *pos = i;

    return write_vendor_ie(p, n, body, b);
}

// Number of used entries at table index >= pos. Caller must hold the lock.
static int count_entries_from(int pos) {
    int count = 0;
    for (int i = pos; i < MAX_ENTRIES; i++) if (s_entries[i].used) count++;
    return count;
}

// Shard the whole entry table into back-to-back beacons of at most
// ENTRIES_PER_PACKET entries each. `frames` holds MAX_SHARDS frames of
// SHARD_FRAME_SIZE bytes; `lens[i]` receives each frame's length. An empty table
// still yields one (empty) beacon. Caller must hold the lock. Returns the shard
// count.
static int build_beacon_shards(uint8_t frames[][SHARD_FRAME_SIZE], int *lens) {
    int pos = 0, shards = 0;
    do {
        lens[shards] = build_beacon_shard(frames[shards], &pos);
        shards++;
    } while (shards < MAX_SHARDS && count_entries_from(pos) > 0);
    return shards;
}

// Build a legacy 0x01 LightUpdate beacon whose only purpose is to trigger DFU on
// `id` (control_flags=DFU, empty entry table). Only bulbs built with legacy
// processing enabled will act on it. Caller must hold the lock. Returns length.
static int build_legacy_dfu_beacon(uint8_t *p, uint8_t id) {
    int n = write_beacon_header(p);
    uint8_t body[4] = {TAG_LIGHT_UPDATE, CTRL_FLAG_DFU, id, 0 /* entry_count */};
    return write_vendor_ie(p, n, body, sizeof(body));
}

// Build a 0x04 BulbCommand beacon addressed to ids [start, start+bounds] with the
// given cmd and raw payload. Caller must hold the lock. Returns frame length.
static int build_command_beacon(uint8_t *p, uint32_t seq, uint8_t start, uint8_t bounds,
                                uint8_t cmd, const uint8_t *payload, int payload_len) {
    int n = write_beacon_header(p);

    // BulbCommand body: tag, seq(u32 LE), start, bounds, cmd, payload...
    uint8_t body[8 + 3 + CMD_VALUE_MAX];
    int b = 0;
    body[b++] = TAG_BULB_COMMAND;
    body[b++] = (uint8_t)(seq);
    body[b++] = (uint8_t)(seq >> 8);
    body[b++] = (uint8_t)(seq >> 16);
    body[b++] = (uint8_t)(seq >> 24);
    body[b++] = start;
    body[b++] = bounds;
    body[b++] = cmd;
    for (int i = 0; i < payload_len; i++) body[b++] = payload[i];

    return write_vendor_ie(p, n, body, b);
}

// ---- one-shot burst sender --------------------------------------------------
// Queue a fully-built frame to be transmitted `count` times at `interval_ms`.
// `done` (may be NULL) is given once the last frame has been handed to the
// driver. `wait` controls how long to block if the queue is full (the console
// passes 0 and drops; dfurange blocks). Returns true if queued.
static bool enqueue_burst_ex(const uint8_t *frame, int len, int count, int interval_ms,
                             SemaphoreHandle_t done, TickType_t wait) {
    burst_job_t job;
    if (len > (int)sizeof(job.frame)) return false;  // never happens for our frames
    memcpy(job.frame, frame, len);
    job.len = len;
    job.count = count;
    job.interval_ms = interval_ms;
    job.done = done;
    if (xQueueSend(s_burst_queue, &job, wait) != pdTRUE) {
        printf("burst queue full; frame dropped\n");
        return false;
    }
    return true;
}

static void enqueue_burst(const uint8_t *frame, int len, int count, int interval_ms) {
    enqueue_burst_ex(frame, len, count, interval_ms, NULL, 0);
}

// Build and enqueue a BulbCommand, consuming (and advancing) the seq counter.
// Not holding the lock on entry. Returns the seq that was used (or 0 and
// *queued=false if the burst could not be queued).
static uint32_t issue_command_ex(uint8_t start, uint8_t bounds, uint8_t cmd,
                                 const uint8_t *payload, int payload_len,
                                 SemaphoreHandle_t done, TickType_t wait, bool *queued) {
    uint8_t frame[160];
    LOCK();
    uint32_t used = s_seq;
    int len = build_command_beacon(frame, used, start, bounds, cmd, payload, payload_len);
    s_seq++;
    UNLOCK();
    // Queue outside the lock: with a nonzero wait this may block while the
    // burst task drains earlier jobs, and it must not hold up the broadcaster.
    bool ok = enqueue_burst_ex(frame, len, CMD_BURST_COUNT, CMD_BURST_INTERVAL_MS, done, wait);
    if (queued) *queued = ok;
    return used;
}

// Console variant: fire-and-forget, and inform the user of the seq that was
// used and the next seq to be issued.
static void issue_command(uint8_t start, uint8_t bounds, uint8_t cmd,
                          const uint8_t *payload, int payload_len) {
    uint32_t used = issue_command_ex(start, bounds, cmd, payload, payload_len, NULL, 0, NULL);
    printf("BulbCommand sent with seq=%u (%dx @ %d ms); seq now %u\n",
           (unsigned)used, CMD_BURST_COUNT, CMD_BURST_INTERVAL_MS, (unsigned)(used + 1));
}

// Transmit queued burst frames, `count` times each at `interval_ms`. Runs
// independently of the continuous entry-table broadcast.
static void burst_task(void *arg) {
    (void)arg;
    burst_job_t job;
    for (;;) {
        if (xQueueReceive(s_burst_queue, &job, portMAX_DELAY) != pdTRUE) continue;
        for (int i = 0; i < job.count; i++) {
            esp_err_t e = esp_wifi_80211_tx(WIFI_IF_STA, job.frame, job.len, true);
            if (e != ESP_OK) ESP_LOGW(TAG, "burst tx failed: %s", esp_err_to_name(e));
            if (i + 1 < job.count) vTaskDelay(pdMS_TO_TICKS(job.interval_ms));
        }
        if (job.done) xSemaphoreGive(job.done);
    }
}

// dfurange worker: one DFU BulbCommand burst per bulb, in order, each one
// waited for before the next is issued, printing progress after every burst.
// Total time is (last - first + 1) * CMD_BURST_COUNT * CMD_BURST_INTERVAL_MS.
typedef struct { uint8_t first, last; } dfurange_args_t;

static void dfurange_task(void *arg) {
    dfurange_args_t a = *(dfurange_args_t *)arg;
    free(arg);
    int total = a.last - a.first + 1;

    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    configASSERT(done != NULL);

    int sent = 0;
    for (int id = a.first; id <= a.last; id++) {
        bool queued;
        uint32_t seq = issue_command_ex((uint8_t)id, 0, CMD_ENTER_DFU, NULL, 0,
                                        done, portMAX_DELAY, &queued);
        if (!queued) {
            printf("dfurange: failed to queue burst for bulb %d; aborting\n", id);
            break;
        }
        xSemaphoreTake(done, portMAX_DELAY);
        sent++;
        printf("dfurange: bulb %d DFU sent (seq %u)  [%d/%d, %d remaining]\n",
               id, (unsigned)seq, sent, total, total - sent);
    }
    printf("dfurange: done, %d/%d bulbs (%d..%d) sent DFU\n", sent, total, a.first, a.last);

    vSemaphoreDelete(done);
    LOCK(); s_dfurange_active = false; UNLOCK();
    vTaskDelete(NULL);
}

// Build a complete 0x02 PreciseLightUpdate beacon for the active precise target.
// Caller must hold the lock (and ensure s_precise.active). Returns frame length.
static int build_precise_beacon(uint8_t *p) {
    int n = write_beacon_header(p);

    // PreciseLightUpdate body (tag 0x02): tag, bulb_id, 5 * f32 little-endian.
    // The ESP32 is little-endian, so memcpy of each float yields the wire order.
    uint8_t body[PRECISE_BODY_SIZE];
    int b = 0;
    body[b++] = TAG_PRECISE_LIGHT_UPDATE;
    body[b++] = s_precise.id;
    const float ch[5] = {s_precise.r, s_precise.g, s_precise.b,
                         s_precise.ww, s_precise.cw};
    for (int i = 0; i < 5; i++) { memcpy(body + b, &ch[i], 4); b += 4; }

    return write_vendor_ie(p, n, body, b);
}

// ---- sweep -----------------------------------------------------------------
// Return the index of the entry for `id`, or -1. Caller must hold the lock.
static int find_entry(uint8_t id) {
    for (int i = 0; i < MAX_ENTRIES; i++)
        if (s_entries[i].used && s_entries[i].id == id) return i;
    return -1;
}

// Write the current triangle-wave value into the swept channels of every entry
// in the sweep's id range. Caller must hold the lock. If no entry in the range
// remains (all cleared), stop.
static void apply_sweep(void) {
    int64_t period = (int64_t)s_sweep.period_ms * 1000;   // us
    int64_t half = period / 2;
    int64_t phase = (esp_timer_get_time() - s_sweep.start_us) % period;
    // Linear up for the first half, linear back down for the second.
    int64_t v = (phase < half) ? (255 * phase) / half
                               : (255 * (period - phase)) / half;
    uint8_t val = (uint8_t)v;

    int touched = 0;
    for (int i = 0; i < MAX_ENTRIES; i++) {
        entry_t *e = &s_entries[i];
        if (!e->used || e->id < s_sweep.first || e->id > s_sweep.last) continue;
        if (s_sweep.mask & CH_R)  e->r  = val;
        if (s_sweep.mask & CH_G)  e->g  = val;
        if (s_sweep.mask & CH_B)  e->b  = val;
        if (s_sweep.mask & CH_WW) e->ww = val;
        if (s_sweep.mask & CH_CW) e->cw = val;
        touched++;
    }
    if (!touched) s_sweep.active = false;
}

// Make sure an entry exists for every id in first..last, seeding new ones at
// all-zero (channels not being swept keep whatever they already hold). Atomic:
// returns false, changing nothing, if the new ids won't fit. Caller holds lock.
static bool ensure_range_entries(int first, int last) {
    int existing = 0;
    for (int id = first; id <= last; id++)
        if (find_entry((uint8_t)id) >= 0) existing++;
    int needed = (last - first + 1) - existing;
    if (needed > MAX_ENTRIES - count_entries_from(0)) return false;
    int next_free = 0;
    for (int id = first; id <= last; id++) {
        if (find_entry((uint8_t)id) >= 0) continue;
        while (s_entries[next_free].used) next_free++;
        s_entries[next_free] = (entry_t){.used = true, .id = (uint8_t)id};
    }
    return true;
}

// Write the current triangle-wave value into the swept channels of the precise
// target, bouncing between s_psweep.min and .max. Caller holds the lock and has
// ensured both s_precise.active and s_psweep.active.
static void apply_psweep(void) {
    int64_t period = (int64_t)s_psweep.period_ms * 1000;   // us
    int64_t half = period / 2;
    int64_t phase = (esp_timer_get_time() - s_psweep.start_us) % period;
    // Triangle in [0,1]: linear up for the first half, back down for the second.
    float tri = (phase < half) ? (float)phase / (float)half
                               : (float)(period - phase) / (float)half;
    float val = s_psweep.min + (s_psweep.max - s_psweep.min) * tri;

    if (s_psweep.mask & CH_R)  s_precise.r  = val;
    if (s_psweep.mask & CH_G)  s_precise.g  = val;
    if (s_psweep.mask & CH_B)  s_precise.b  = val;
    if (s_psweep.mask & CH_WW) s_precise.ww = val;
    if (s_psweep.mask & CH_CW) s_precise.cw = val;
}

// ---- broadcast task ---------------------------------------------------------
static void broadcast_task(void *arg) {
    (void)arg;
    static uint8_t frames[MAX_SHARDS][SHARD_FRAME_SIZE];
    static int lens[MAX_SHARDS];
    static uint8_t pframe[128];
    int applied_channel = -1;

    for (;;) {
        int interval;
        bool enabled;
        int shards = 0;
        int plen = 0;

        LOCK();
        enabled = s_enabled;
        interval = s_interval_ms;
        if (applied_channel != s_channel) {
            esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE);
            applied_channel = s_channel;
        }
        if (enabled) {
            if (s_sweep.active) apply_sweep();
            shards = build_beacon_shards(frames, lens);
            // A precise target rides along as its own separate beacon (a beacon
            // may carry only our one vendor IE, so it cannot share the frame).
            if (s_precise.active) {
                if (s_psweep.active) apply_psweep();
                plen = build_precise_beacon(pframe);
            }
        }
        UNLOCK();

        // Entry-table shards go out back-to-back; each carries a disjoint slice
        // of the table, so together they address every entry once per tick.
        for (int i = 0; enabled && i < shards; i++) {
            esp_err_t e = esp_wifi_80211_tx(WIFI_IF_STA, frames[i], lens[i], true);
            if (e != ESP_OK) {
                ESP_LOGW(TAG, "80211_tx (shard %d/%d) failed: %s", i + 1, shards,
                         esp_err_to_name(e));
            }
        }
        if (enabled && plen > 0) {
            esp_err_t e = esp_wifi_80211_tx(WIFI_IF_STA, pframe, plen, true);
            if (e != ESP_OK) {
                ESP_LOGW(TAG, "80211_tx (precise) failed: %s", esp_err_to_name(e));
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

// Parse a 0..100 percentage (fractional allowed) into a normalized [0,1] frac.
static bool parse_pct(const char *s, float *frac) {
    char *end;
    double v = strtod(s, &end);
    if (end == s || *end != '\0' || v < 0.0 || v > 100.0) return false;
    *frac = (float)(v / 100.0);
    return true;
}

// Parse a channel value on set's 0..255 scale, but as a float (fractional
// allowed) so PreciseLightUpdate keeps full precision.
static bool parse_chanf(const char *s, float *out) {
    char *end;
    double v = strtod(s, &end);
    if (end == s || *end != '\0' || v < 0.0 || v > 255.0) return false;
    *out = (float)v;
    return true;
}

// Parse a comma-separated channel list ("r,g,b" / "ww,cw" / "r,g,b,ww,cw")
// into an OR of CH_* bits. Returns false on any unknown token.
static bool parse_channel_mask(const char *s, uint8_t *out) {
    char buf[32];
    if (strlen(s) >= sizeof(buf)) return false;
    strcpy(buf, s);
    uint8_t mask = 0;
    for (char *tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
        if      (!strcmp(tok, "r"))  mask |= CH_R;
        else if (!strcmp(tok, "g"))  mask |= CH_G;
        else if (!strcmp(tok, "b"))  mask |= CH_B;
        else if (!strcmp(tok, "ww")) mask |= CH_WW;
        else if (!strcmp(tok, "cw")) mask |= CH_CW;
        else return false;
    }
    if (mask == 0) return false;
    *out = mask;
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

// setrange <first> <last> <r> <g> <b> <ww> <cw> : set every id in first..last
// (inclusive) to one color. Atomic: either every id fits in the table or
// nothing changes.
static int cmd_setrange(int argc, char **argv) {
    if (argc != 8) {
        printf("usage: setrange <first> <last> <r> <g> <b> <ww> <cw>\n");
        printf("  sets every bulb id in first..last (inclusive) to the same color (values 0..255)\n");
        return 1;
    }
    uint8_t v[7];
    for (int i = 0; i < 7; i++) {
        if (!parse_u8(argv[i + 1], &v[i])) {
            printf("bad value '%s' (0..255)\n", argv[i + 1]);
            return 1;
        }
    }
    int first = v[0], last = v[1];
    if (first > last) { printf("first must be <= last\n"); return 1; }
    int span = last - first + 1;
    if (span > MAX_ENTRIES) {
        printf("range covers %d bulbs (max %d)\n", span, MAX_ENTRIES);
        return 1;
    }

    LOCK();
    if (!ensure_range_entries(first, last)) {
        int free_slots = MAX_ENTRIES - count_entries_from(0);
        UNLOCK();
        printf("not enough free entry slots (%d free, max %d)\n", free_slots, MAX_ENTRIES);
        return 1;
    }
    for (int i = 0; i < MAX_ENTRIES; i++) {
        entry_t *e = &s_entries[i];
        if (!e->used || e->id < first || e->id > last) continue;
        e->r = v[2]; e->g = v[3]; e->b = v[4]; e->ww = v[5]; e->cw = v[6];
    }
    UNLOCK();
    printf("set bulbs %d..%d (%d) -> r%d g%d b%d ww%d cw%d\n", first, last, span,
           v[2], v[3], v[4], v[5], v[6]);
    return 0;
}

static int cmd_scaledset(int argc, char **argv) {
    if (argc != 8) {
        printf("usage: scaledset <id> <r> <g> <b> <ww> <cw> <scale>\n");
        printf("  channels 0..255 (fractional ok), scale 0..100%%; sends PreciseLightUpdate floats\n");
        return 1;
    }
    uint8_t id;
    float v[5];
    if (!parse_u8(argv[1], &id)) { printf("bad id\n"); return 1; }
    for (int i = 0; i < 5; i++) {
        if (!parse_chanf(argv[i + 2], &v[i])) {
            printf("bad value '%s' (0..255)\n", argv[i + 2]);
            return 1;
        }
    }
    float scale;
    if (!parse_pct(argv[7], &scale)) { printf("bad scale '%s' (0..100)\n", argv[7]); return 1; }

    // Everything stays in float: normalize each channel to [0,1], then apply the
    // scale factor, so `scale` acts as a proportional dimmer on a 0..255 color.
    float ch[5];
    for (int i = 0; i < 5; i++) ch[i] = (v[i] / 255.0f) * scale;

    LOCK();
    s_psweep.active = false;   // a static precise set replaces any precise sweep
    s_precise = (precise_t){.active = true, .id = id, .r = ch[0], .g = ch[1],
                            .b = ch[2], .ww = ch[3], .cw = ch[4]};
    UNLOCK();
    printf("precise set bulb %d -> r%.3f g%.3f b%.3f ww%.3f cw%.3f (scale %s%%)\n",
           id, ch[0], ch[1], ch[2], ch[3], ch[4], argv[7]);
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
    if (argc < 2 || argc > 3) {
        printf("usage: dfu <id> [new|legacy]\n");
        printf("  new (default): BulbCommand EnterDfuMode (0x04)\n");
        printf("  legacy       : deprecated LightUpdate DFU (0x01), for old bulbs\n");
        return 1;
    }
    uint8_t id;
    if (!parse_u8(argv[1], &id)) { printf("bad id\n"); return 1; }

    bool legacy = false;
    if (argc == 3) {
        if (!strcmp(argv[2], "legacy"))   legacy = true;
        else if (!strcmp(argv[2], "new")) legacy = false;
        else { printf("bad mode '%s' (use 'new' or 'legacy')\n", argv[2]); return 1; }
    }

    if (legacy) {
        uint8_t frame[160];
        LOCK();
        int len = build_legacy_dfu_beacon(frame, id);
        UNLOCK();
        enqueue_burst(frame, len, CMD_BURST_COUNT, CMD_BURST_INTERVAL_MS);
        printf("requesting DFU for bulb %d via legacy LightUpdate (%dx @ %d ms)\n",
               id, CMD_BURST_COUNT, CMD_BURST_INTERVAL_MS);
    } else {
        printf("requesting DFU for bulb %d via BulbCommand\n", id);
        issue_command(id, 0, CMD_ENTER_DFU, NULL, 0);
    }
    return 0;
}

// dfurange <first> <last> : DFU every bulb in first..last, one burst per bulb.
static int cmd_dfurange(int argc, char **argv) {
    if (argc != 3) {
        printf("usage: dfurange <first> <last>\n");
        printf("  one BulbCommand DFU burst (%dx @ %d ms) per bulb, in order, with progress\n",
               CMD_BURST_COUNT, CMD_BURST_INTERVAL_MS);
        return 1;
    }
    uint8_t first, last;
    if (!parse_u8(argv[1], &first)) { printf("bad first id\n"); return 1; }
    if (!parse_u8(argv[2], &last))  { printf("bad last id\n"); return 1; }
    if (first > last) { printf("first must be <= last\n"); return 1; }

    LOCK();
    if (s_dfurange_active) {
        UNLOCK();
        printf("a dfurange is already running\n");
        return 1;
    }
    s_dfurange_active = true;
    UNLOCK();

    dfurange_args_t *a = malloc(sizeof(*a));
    if (!a) { LOCK(); s_dfurange_active = false; UNLOCK(); printf("out of memory\n"); return 1; }
    a->first = first; a->last = last;
    int total = last - first + 1;
    if (xTaskCreate(dfurange_task, "dfurange", 4096, a, 5, NULL) != pdPASS) {
        free(a);
        LOCK(); s_dfurange_active = false; UNLOCK();
        printf("failed to start dfurange task\n");
        return 1;
    }
    printf("dfurange: sending DFU to bulbs %d..%d (%d bulbs, ~%d s)\n", first, last, total,
           (total * CMD_BURST_COUNT * CMD_BURST_INTERVAL_MS + 999) / 1000);
    return 0;
}

// setconfig <id> <key> <u8|u32|f32|str> <value> : issue a SetConfig BulbCommand.
static int cmd_setconfig(int argc, char **argv) {
    if (argc != 5) {
        printf("usage: setconfig <id> <key> <u8|u32|f32|str> <value>\n");
        printf("  key: 16-bit config tag (e.g. 0x0031); value encoded per type (LE)\n");
        return 1;
    }
    uint8_t id;
    if (!parse_u8(argv[1], &id)) { printf("bad id\n"); return 1; }

    char *end;
    long k = strtol(argv[2], &end, 0);
    if (*end != '\0' || k < 0 || k > 0xFFFF) {
        printf("bad key '%s' (0..65535)\n", argv[2]);
        return 1;
    }
    uint16_t key = (uint16_t)k;

    const char *type = argv[3];
    const char *vs = argv[4];
    uint8_t value[CMD_VALUE_MAX];
    int vlen = 0;

    if (!strcmp(type, "u8")) {
        long v = strtol(vs, &end, 0);
        if (*end != '\0' || v < 0 || v > 255) { printf("bad u8 value '%s'\n", vs); return 1; }
        value[0] = (uint8_t)v; vlen = 1;
    } else if (!strcmp(type, "u32")) {
        unsigned long v = strtoul(vs, &end, 0);
        if (*end != '\0') { printf("bad u32 value '%s'\n", vs); return 1; }
        value[0] = (uint8_t)v; value[1] = (uint8_t)(v >> 8);
        value[2] = (uint8_t)(v >> 16); value[3] = (uint8_t)(v >> 24); vlen = 4;
    } else if (!strcmp(type, "f32")) {
        double v = strtod(vs, &end);
        if (end == vs || *end != '\0') { printf("bad f32 value '%s'\n", vs); return 1; }
        float f = (float)v; memcpy(value, &f, 4); vlen = 4;
    } else if (!strcmp(type, "str")) {
        size_t sl = strlen(vs);
        if (sl > CMD_VALUE_MAX - 1) { printf("string too long (max %d)\n", CMD_VALUE_MAX - 1); return 1; }
        memcpy(value, vs, sl); vlen = (int)sl;
    } else {
        printf("bad type '%s' (u8|u32|f32|str)\n", type);
        return 1;
    }

    uint8_t payload[3 + CMD_VALUE_MAX];
    payload[0] = (uint8_t)key;
    payload[1] = (uint8_t)(key >> 8);
    payload[2] = (uint8_t)vlen;
    memcpy(&payload[3], value, vlen);

    printf("setconfig bulb %d key 0x%04x = %s (%s, %d bytes)\n", id, key, vs, type, vlen);
    issue_command(id, 0, CMD_SET_CONFIG, payload, 3 + vlen);
    return 0;
}

// seq [value] : show, or re-seed, the BulbCommand sequence counter.
static int cmd_seq(int argc, char **argv) {
    if (argc == 1) {
        LOCK(); uint32_t s = s_seq; UNLOCK();
        printf("seq = %u\n", (unsigned)s);
        return 0;
    }
    if (argc != 2) { printf("usage: seq [value]\n"); return 1; }
    char *end;
    unsigned long v = strtoul(argv[1], &end, 0);
    if (*end != '\0') { printf("bad seq '%s'\n", argv[1]); return 1; }
    LOCK(); s_seq = (uint32_t)v; UNLOCK();
    printf("seq re-seeded to %u\n", (unsigned)v);
    return 0;
}

// Shared body of `sweep` / `sweeprange`: start a linear sweep of `chan_str`
// channels over ids first..last with a full cycle every `ms_str` ms.
static int start_sweep(int first, int last, const char *chan_str, const char *ms_str) {
    uint8_t mask;
    if (!parse_channel_mask(chan_str, &mask)) {
        printf("bad channels '%s' (use r,g,b,ww,cw)\n", chan_str);
        return 1;
    }
    int ms = atoi(ms_str);
    if (ms < 100 || ms > 600000) {
        printf("duration out of range (100..600000 ms)\n");
        return 1;
    }

    LOCK();
    // Ensure an entry exists for every swept bulb so the sweep has somewhere
    // to write; channels not being swept keep whatever value they already hold.
    if (!ensure_range_entries(first, last)) {
        int free_slots = MAX_ENTRIES - count_entries_from(0);
        UNLOCK();
        printf("not enough free entry slots (%d free, max %d)\n", free_slots, MAX_ENTRIES);
        return 1;
    }
    s_sweep = (sweep_t){.active = true, .first = (uint8_t)first, .last = (uint8_t)last,
                        .mask = mask, .period_ms = ms, .start_us = esp_timer_get_time()};
    UNLOCK();
    if (first == last)
        printf("sweeping bulb %d channels %s over %d ms (0->255->0, looping)\n",
               first, chan_str, ms);
    else
        printf("sweeping bulbs %d..%d (%d) channels %s over %d ms (0->255->0, looping, in lockstep)\n",
               first, last, last - first + 1, chan_str, ms);
    return 0;
}

static int cmd_sweep(int argc, char **argv) {
    if (argc != 4) {
        printf("usage: sweep <id> <channels> <duration_ms>\n");
        printf("  channels: comma list of r,g,b,ww,cw  (e.g. r,g,b)\n");
        return 1;
    }
    uint8_t id;
    if (!parse_u8(argv[1], &id)) { printf("bad id\n"); return 1; }
    return start_sweep(id, id, argv[2], argv[3]);
}

// sweeprange <first> <last> <channels> <duration_ms> : one sweep driving every
// bulb in first..last in lockstep (replaces any active linear sweep).
static int cmd_sweeprange(int argc, char **argv) {
    if (argc != 5) {
        printf("usage: sweeprange <first> <last> <channels> <duration_ms>\n");
        printf("  channels: comma list of r,g,b,ww,cw  (e.g. r,g,b)\n");
        return 1;
    }
    uint8_t first, last;
    if (!parse_u8(argv[1], &first)) { printf("bad first id\n"); return 1; }
    if (!parse_u8(argv[2], &last))  { printf("bad last id\n"); return 1; }
    if (first > last) { printf("first must be <= last\n"); return 1; }
    if (last - first + 1 > MAX_ENTRIES) {
        printf("range covers %d bulbs (max %d)\n", last - first + 1, MAX_ENTRIES);
        return 1;
    }
    return start_sweep(first, last, argv[3], argv[4]);
}

static int cmd_psweep(int argc, char **argv) {
    if (argc != 6) {
        printf("usage: psweep <id> <channels> <duration_ms> <min> <max>\n");
        printf("  channels: comma list of r,g,b,ww,cw ; min/max are 0..100%% of full scale\n");
        return 1;
    }
    uint8_t id, mask;
    if (!parse_u8(argv[1], &id)) { printf("bad id\n"); return 1; }
    if (!parse_channel_mask(argv[2], &mask)) {
        printf("bad channels '%s' (use r,g,b,ww,cw)\n", argv[2]);
        return 1;
    }
    int ms = atoi(argv[3]);
    if (ms < 100 || ms > 600000) {
        printf("duration out of range (100..600000 ms)\n");
        return 1;
    }
    float mn, mx;
    if (!parse_pct(argv[4], &mn)) { printf("bad min '%s' (0..100)\n", argv[4]); return 1; }
    if (!parse_pct(argv[5], &mx)) { printf("bad max '%s' (0..100)\n", argv[5]); return 1; }
    if (mn > mx) { printf("min must be <= max\n"); return 1; }

    LOCK();
    // Retarget the single precise slot if needed; a fresh target starts at 0 on
    // the channels the sweep won't touch (mirrors how `sweep` seeds an entry).
    if (!s_precise.active || s_precise.id != id) {
        s_precise = (precise_t){.active = true, .id = id};
    }
    s_psweep = (psweep_t){.active = true, .mask = mask, .period_ms = ms,
                          .start_us = esp_timer_get_time(), .min = mn, .max = mx};
    UNLOCK();
    printf("precise sweeping bulb %d channels %s over %d ms between %s%%..%s%% (looping)\n",
           id, argv[2], ms, argv[4], argv[5]);
    return 0;
}

static int cmd_stopsweep(int argc, char **argv) {
    (void)argc; (void)argv;
    LOCK();
    bool was = s_sweep.active || s_psweep.active;
    s_sweep.active = false;
    s_psweep.active = false;   // precise target holds its last value, keeps broadcasting
    UNLOCK();
    printf(was ? "sweep(s) stopped\n" : "no active sweep\n");
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

// rate <mbps> : set the PHY rate used for injected frames (applies immediately).
static int cmd_rate(int argc, char **argv) {
    if (argc != 2) {
        printf("usage: rate <");
        for (int i = 0; i < N_RATE_OPTS; i++) printf("%s%s", i ? "|" : "", s_rate_opts[i].name);
        printf(">  (Mbps)\n");
        return 1;
    }
    int idx = -1;
    for (int i = 0; i < N_RATE_OPTS; i++)
        if (!strcmp(argv[1], s_rate_opts[i].name)) { idx = i; break; }
    if (idx < 0) { printf("unknown rate '%s'\n", argv[1]); return 1; }
    esp_err_t e = esp_wifi_config_80211_tx_rate(WIFI_IF_STA, s_rate_opts[idx].rate);
    if (e != ESP_OK) { printf("failed to set rate: %s\n", esp_err_to_name(e)); return 1; }
    LOCK(); s_rate_idx = idx; UNLOCK();
    printf("tx rate = %s Mbps\n", s_rate_opts[idx].name);
    return 0;
}

static int cmd_interval(int argc, char **argv) {
    if (argc != 2) { printf("usage: interval <ms>\n"); return 1; }
    int ms = atoi(argv[1]);
    if (ms < 5 || ms > 10000) { printf("interval out of range (5..10000)\n"); return 1; }
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
    printf("state: %s  channel %d  rate %s Mbps  interval %d ms  next seq %u\n",
           s_enabled ? "ON" : "OFF", s_channel, s_rate_opts[s_rate_idx].name,
           s_interval_ms, (unsigned)s_seq);
    printf("base MAC (frame src): %02x:%02x:%02x:%02x:%02x:%02x\n",
           s_base_mac[0], s_base_mac[1], s_base_mac[2], s_base_mac[3], s_base_mac[4], s_base_mac[5]);
    if (s_dfurange_active) printf("dfurange: in progress\n");
    if (s_sweep.active) {
        if (s_sweep.first == s_sweep.last) printf("sweep: bulb %d ", s_sweep.first);
        else printf("sweep: bulbs %d..%d ", s_sweep.first, s_sweep.last);
        printf(" channels%s%s%s%s%s  period %d ms\n",
               (s_sweep.mask & CH_R)  ? " r"  : "", (s_sweep.mask & CH_G)  ? " g"  : "",
               (s_sweep.mask & CH_B)  ? " b"  : "", (s_sweep.mask & CH_WW) ? " ww" : "",
               (s_sweep.mask & CH_CW) ? " cw" : "", s_sweep.period_ms);
    }
    if (s_precise.active) {
        printf("precise: bulb %d  r%.3f g%.3f b%.3f ww%.3f cw%.3f\n", s_precise.id,
               s_precise.r, s_precise.g, s_precise.b, s_precise.ww, s_precise.cw);
        if (s_psweep.active) {
            printf("psweep: channels%s%s%s%s%s  period %d ms  range %.0f%%..%.0f%%\n",
                   (s_psweep.mask & CH_R)  ? " r"  : "", (s_psweep.mask & CH_G)  ? " g"  : "",
                   (s_psweep.mask & CH_B)  ? " b"  : "", (s_psweep.mask & CH_WW) ? " ww" : "",
                   (s_psweep.mask & CH_CW) ? " cw" : "", s_psweep.period_ms,
                   s_psweep.min * 100.0f, s_psweep.max * 100.0f);
        }
    }
    int count = 0;
    for (int i = 0; i < MAX_ENTRIES; i++) {
        if (!s_entries[i].used) continue;
        printf("  bulb %3d : r%3d g%3d b%3d ww%3d cw%3d\n", s_entries[i].id,
               s_entries[i].r, s_entries[i].g, s_entries[i].b, s_entries[i].ww, s_entries[i].cw);
        count++;
    }
    if (!count) printf("  (no entries)\n");
    else printf("  %d entries in %d packet(s)/tick (%d per packet)\n", count,
                (count + ENTRIES_PER_PACKET - 1) / ENTRIES_PER_PACKET, ENTRIES_PER_PACKET);
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
    static uint8_t frame[SHARD_FRAME_SIZE];
    int pos = 0;
    int len = build_beacon_shard(frame, &pos);  // benchmark the first shard of the table
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
        {.command = "setrange", .help = "setrange <first> <last> <r> <g> <b> <ww> <cw> : set every id in first..last to one color", .func = &cmd_setrange},
        {.command = "scaledset",.help = "scaledset <id> <r> <g> <b> <ww> <cw> <scale> : PreciseLightUpdate, channels scaled by scale%", .func = &cmd_scaledset},
        {.command = "clr",      .help = "clr <id> : remove one bulb entry", .func = &cmd_clr},
        {.command = "clrall",   .help = "remove all bulb entries", .func = &cmd_clrall},
        {.command = "dfu",      .help = "dfu <id> [new|legacy] : request DFU (new=BulbCommand, legacy=0x01)", .func = &cmd_dfu},
        {.command = "dfurange", .help = "dfurange <first> <last> : DFU every id in first..last, one burst per bulb, with progress", .func = &cmd_dfurange},
        {.command = "setconfig",.help = "setconfig <id> <key> <u8|u32|f32|str> <value> : SetConfig BulbCommand", .func = &cmd_setconfig},
        {.command = "seq",      .help = "seq [value] : show or re-seed the BulbCommand sequence counter", .func = &cmd_seq},
        {.command = "sweep",    .help = "sweep <id> <channels> <duration_ms> : linear 0->255->0 loop (channels = r,g,b,ww,cw)", .func = &cmd_sweep},
        {.command = "sweeprange",.help = "sweeprange <first> <last> <channels> <duration_ms> : linear sweep of every id in first..last in lockstep", .func = &cmd_sweeprange},
        {.command = "psweep",   .help = "psweep <id> <channels> <duration_ms> <min> <max> : precise float sweep between min%..max%", .func = &cmd_psweep},
        {.command = "stopsweep",.help = "stop the active sweep(s)", .func = &cmd_stopsweep},
        {.command = "chan",     .help = "chan <1..13> : set broadcast channel", .func = &cmd_chan},
        {.command = "rate",     .help = "rate <mbps> : set PHY rate for injected frames (1|2|5.5|11|6|9|12|18|24|36|48|54)", .func = &cmd_rate},
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
    // Inject at 11 Mbps rather than the 1 Mbps default (see s_rate_opts).
    ESP_ERROR_CHECK(esp_wifi_config_80211_tx_rate(WIFI_IF_STA, s_rate_opts[s_rate_idx].rate));
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

    s_burst_queue = xQueueCreate(8, sizeof(burst_job_t));
    configASSERT(s_burst_queue != NULL);

    wifi_init_tx();

    xTaskCreate(broadcast_task, "broadcast", 4096, NULL, 5, NULL);
    xTaskCreate(burst_task, "burst", 4096, NULL, 5, NULL);

    // Serial REPL over the USB UART.
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "base>";
    repl_config.max_cmdline_length = 128;
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    ESP_ERROR_CHECK(esp_console_register_help_command());
    register_commands();

    ESP_LOGI(TAG, "RC light base station ready. Type 'help'. Broadcasting on channel %d at %s Mbps.",
             s_channel, s_rate_opts[s_rate_idx].name);
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
