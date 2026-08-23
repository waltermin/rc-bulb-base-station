// test_frame_compat.c — cross-firmware wire-compatibility check.
//
// Builds beacon frames byte-for-byte the way the ESP32 base station builds them,
// then decodes them with the ACTUAL bulb parser (bulb-firmware/main/protocol.c).
// This proves the two firmwares agree on the wire format without needing either
// microcontroller.
//
// The 0x01 legacy DFU test requires the bulb parser to be built with legacy
// processing enabled, so this is compiled with -DPROTO_ENABLE_LEGACY_LIGHT_UPDATE=1
// (see the Makefile) — mirroring a bulb that still accepts the deprecated packet.
//
// Build & run (from this dir):
//   make

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "protocol.h"  // from bulb-firmware/main

// ---- mirror of the base station's frame layout ------------------------------
#define OUI0 0x52
#define OUI1 0x43
#define OUI2 0x68
#define TAG_LIGHT_UPDATE 0x01
#define TAG_PRECISE_LIGHT_UPDATE 0x02
#define TAG_LIGHT_UPDATE_V2 0x03
#define TAG_BULB_COMMAND 0x04
#define CTRL_FLAG_DFU 0x01
#define CMD_ENTER_DFU 0x00
#define CMD_SET_CONFIG 0x01
#define MAX_ENTRIES 64          // base-station table size
#define ENTRIES_PER_PACKET 11   // bulb-side per-packet limit (PROTO_MAX_ENTRIES)
#define ENTRY_SIZE 6

typedef struct { uint8_t id, r, g, b, ww, cw; } entry_t;

// Float compare with tolerance covering 1/255 quantization.
#define FEQ(a, b) (((a) - (b) < 0.001f) && ((b) - (a) < 0.001f))

static int write_header(uint8_t *p, const uint8_t base_mac[6]) {
    int n = 0;
    p[n++] = 0x80; p[n++] = 0x00;                 // FC: beacon
    p[n++] = 0x00; p[n++] = 0x00;                 // duration
    for (int i = 0; i < 6; i++) p[n++] = 0xFF;    // DA broadcast
    memcpy(p + n, base_mac, 6); n += 6;           // SA
    memcpy(p + n, base_mac, 6); n += 6;           // BSSID
    p[n++] = 0x00; p[n++] = 0x00;                 // seq/frag
    for (int i = 0; i < 8; i++) p[n++] = 0x00;    // timestamp
    p[n++] = 0x64; p[n++] = 0x00;                 // beacon interval
    p[n++] = 0x00; p[n++] = 0x00;                 // capability
    return n;
}

static int write_ie(uint8_t *p, int n, const uint8_t *body, int blen) {
    p[n++] = 0xDD;
    p[n++] = (uint8_t)(3 + blen);
    p[n++] = OUI0; p[n++] = OUI1; p[n++] = OUI2;
    memcpy(p + n, body, blen); n += blen;
    return n;
}

// 0x03 LightUpdateV2 entry table — one shard of <= ENTRIES_PER_PACKET entries
// (the base station's build_beacon_shard). n_entries may exceed that only in
// the negative test below.
static int build_v2(uint8_t *p, const uint8_t base_mac[6],
                    const entry_t *entries, int n_entries) {
    int n = write_header(p, base_mac);
    uint8_t body[2 + MAX_ENTRIES * ENTRY_SIZE];
    int b = 0;
    body[b++] = TAG_LIGHT_UPDATE_V2;
    body[b++] = (uint8_t)n_entries;
    for (int i = 0; i < n_entries; i++) {
        body[b++] = entries[i].id;
        body[b++] = entries[i].r;  body[b++] = entries[i].g;  body[b++] = entries[i].b;
        body[b++] = entries[i].ww; body[b++] = entries[i].cw;
    }
    return write_ie(p, n, body, b);
}

// 0x01 legacy DFU beacon (the base station's `dfu <id> legacy`).
static int build_legacy_dfu(uint8_t *p, const uint8_t base_mac[6], uint8_t id) {
    int n = write_header(p, base_mac);
    uint8_t body[4] = {TAG_LIGHT_UPDATE, CTRL_FLAG_DFU, id, 0};
    return write_ie(p, n, body, sizeof(body));
}

// 0x04 BulbCommand (the base station's build_command_beacon).
static int build_command(uint8_t *p, const uint8_t base_mac[6], uint32_t seq,
                         uint8_t start, uint8_t bounds, uint8_t cmd,
                         const uint8_t *payload, int payload_len) {
    int n = write_header(p, base_mac);
    uint8_t body[8 + 3 + 64];
    int b = 0;
    body[b++] = TAG_BULB_COMMAND;
    body[b++] = (uint8_t)seq;        body[b++] = (uint8_t)(seq >> 8);
    body[b++] = (uint8_t)(seq >> 16); body[b++] = (uint8_t)(seq >> 24);
    body[b++] = start;
    body[b++] = bounds;
    body[b++] = cmd;
    for (int i = 0; i < payload_len; i++) body[b++] = payload[i];
    return write_ie(p, n, body, b);
}

static int build_precise_beacon(uint8_t *p, const uint8_t base_mac[6],
                                uint8_t id, const float ch[5]) {
    int n = write_header(p, base_mac);
    uint8_t body[2 + 5 * 4];
    int b = 0;
    body[b++] = TAG_PRECISE_LIGHT_UPDATE;         // packet tag 0x02
    body[b++] = id;
    for (int i = 0; i < 5; i++) { memcpy(body + b, &ch[i], 4); b += 4; }
    return write_ie(p, n, body, b);
}

static int checks = 0, fails = 0;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("  FAIL: %s (line %d)\n", #c, __LINE__); } } while (0)

int main(void) {
    uint8_t mac[6] = {0x24, 0x6f, 0x28, 0x11, 0x22, 0x33};
    uint8_t buf[256];

    // Base station sets bulbs 2 and 7 (LightUpdateV2); bulb 7 decodes its entry.
    {
        entry_t e[] = {{2, 10, 20, 30, 40, 50}, {7, 200, 150, 100, 60, 25}};
        int len = build_v2(buf, mac, e, 2);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 7);
        printf("v2: base sets bulbs 2 & 7; bulb 7 decodes:\n");
        CHECK(r.valid);
        CHECK(r.has_entry);
        CHECK(!r.dfu_requested && !r.is_command);
        CHECK(FEQ(r.r, 200 / 255.0f) && FEQ(r.g, 150 / 255.0f) && FEQ(r.b, 100 / 255.0f) &&
              FEQ(r.ww, 60 / 255.0f) && FEQ(r.cw, 25 / 255.0f));
    }

    // Same frame, a bulb not addressed (id 9) -> valid but no entry.
    {
        entry_t e[] = {{2, 10, 20, 30, 40, 50}, {7, 200, 150, 100, 60, 25}};
        int len = build_v2(buf, mac, e, 2);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 9);
        printf("v2: bulb 9 not addressed:\n");
        CHECK(r.valid);
        CHECK(!r.has_entry);
    }

    // A full 11-entry packet still fits and decodes.
    {
        entry_t e[ENTRIES_PER_PACKET];
        for (int i = 0; i < ENTRIES_PER_PACKET; i++)
            e[i] = (entry_t){(uint8_t)(i + 1), (uint8_t)i, 0, 0, 0, 0};
        int len = build_v2(buf, mac, e, ENTRIES_PER_PACKET);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 11);
        printf("v2: full 11-entry packet, bulb 11:\n");
        CHECK(r.valid);
        CHECK(r.has_entry);
        CHECK(FEQ(r.r, 10 / 255.0f));
    }

    // A 12-entry packet is rejected by the bulb — which is why the base station
    // shards its table across packets instead of growing the packet.
    {
        entry_t e[12];
        for (int i = 0; i < 12; i++) e[i] = (entry_t){(uint8_t)(i + 1), 1, 2, 3, 4, 5};
        int len = build_v2(buf, mac, e, 12);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 12);
        printf("v2: 12-entry packet rejected by bulb:\n");
        CHECK(!r.valid || !r.has_entry);
    }

    // Full 64-entry table (e.g. `setrange 1 64 ...`) sharded into ceil(64/11)=6
    // packets of <=11 entries: every bulb 1..64 finds its entry in exactly one
    // shard, including the last (partial, 9-entry) shard.
    {
        entry_t table[MAX_ENTRIES];
        for (int i = 0; i < MAX_ENTRIES; i++)
            table[i] = (entry_t){(uint8_t)(i + 1), (uint8_t)(i + 1), 0, 0, 0, 0};
        printf("v2: 64-entry table sharded across packets:\n");
        int shards = 0;
        int hits[MAX_ENTRIES + 1] = {0};
        for (int pos = 0; pos < MAX_ENTRIES; pos += ENTRIES_PER_PACKET) {
            int n = MAX_ENTRIES - pos;
            if (n > ENTRIES_PER_PACKET) n = ENTRIES_PER_PACKET;
            int len = build_v2(buf, mac, table + pos, n);
            shards++;
            for (int id = 1; id <= MAX_ENTRIES; id++) {
                bulb_parse_result_t r = protocol_parse_beacon(buf, len, (uint8_t)id);
                CHECK(r.valid);
                if (r.has_entry) {
                    hits[id]++;
                    CHECK(FEQ(r.r, id / 255.0f));
                }
            }
        }
        CHECK(shards == 6);
        int all_once = 1;
        for (int id = 1; id <= MAX_ENTRIES; id++) if (hits[id] != 1) all_once = 0;
        CHECK(all_once);
    }

    // Empty table (base broadcasting nothing) is still a valid frame.
    {
        int len = build_v2(buf, mac, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 1);
        printf("v2: empty entry table:\n");
        CHECK(r.valid);
        CHECK(!r.has_entry);
    }

    // `dfu 1 legacy` -> a 0x01 LightUpdate DFU that a legacy-enabled bulb accepts.
    {
        int len = build_legacy_dfu(buf, mac, 1);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 1);
        printf("legacy dfu -> bulb 1 sees DFU:\n");
        CHECK(r.valid);
        CHECK(r.dfu_requested);
    }

    // `dfu 1` (new) -> a BulbCommand EnterDfuMode; bulb 1 sees DFU, seq exposed.
    {
        uint8_t none = 0;
        int len = build_command(buf, mac, 5, 1, 0, CMD_ENTER_DFU, &none, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 1);
        printf("command dfu -> bulb 1 sees DFU:\n");
        CHECK(r.valid && r.is_command);
        CHECK(r.seq == 5);
        CHECK(r.dfu_requested);
    }

    // Same command decoded by bulb 2 (out of range) -> command seen, no effect.
    {
        int len = build_command(buf, mac, 5, 1, 0, CMD_ENTER_DFU, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 2);
        printf("command dfu -> bulb 2 ignores (out of range):\n");
        CHECK(r.valid && r.is_command && r.seq == 5);
        CHECK(!r.dfu_requested);
    }

    // Range addressing: start=5 bounds=3 covers ids 5..8; bulb 7 is in range.
    {
        int len = build_command(buf, mac, 9, 5, 3, CMD_ENTER_DFU, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 7);
        printf("command dfu range 5..8 -> bulb 7 in range:\n");
        CHECK(r.valid && r.is_command && r.dfu_requested);
    }

    // `setconfig 7 0x0031 f32 2.2` -> bulb 7 decodes key/len/value bytes.
    {
        float f = 2.2f;
        uint8_t payload[3 + 4];
        uint16_t key = 0x0031;
        payload[0] = (uint8_t)key; payload[1] = (uint8_t)(key >> 8);
        payload[2] = 4;
        memcpy(&payload[3], &f, 4);
        int len = build_command(buf, mac, 42, 7, 0, CMD_SET_CONFIG, payload, sizeof(payload));
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 7);
        printf("command setconfig -> bulb 7 decodes:\n");
        CHECK(r.valid && r.is_command && r.seq == 42);
        CHECK(r.has_config);
        CHECK(r.config_key == 0x0031);
        CHECK(r.config_len == 4);
        float got;
        memcpy(&got, r.config_value, 4);
        CHECK(FEQ(got, 2.2f));
    }

    // Base station 'scaledset'/'psweep' precise beacon for bulb 7 decodes as
    // the same float channels on the bulb.
    {
        float ch[5] = {1.0f, 0.5f, 0.0f, 0.25f, 0.75f};
        int len = build_precise_beacon(buf, mac, 7, ch);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 7);
        printf("base precise beacon -> bulb 7 decodes floats:\n");
        CHECK(r.valid);
        CHECK(r.has_entry);
        CHECK(!r.dfu_requested);
        CHECK(FEQ(r.r, 1.0f) && FEQ(r.g, 0.5f) && FEQ(r.b, 0.0f) &&
              FEQ(r.ww, 0.25f) && FEQ(r.cw, 0.75f));
    }

    // Same precise beacon, a bulb not addressed -> valid, no entry.
    {
        float ch[5] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        int len = build_precise_beacon(buf, mac, 7, ch);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 8);
        printf("base precise beacon -> bulb 8 ignores:\n");
        CHECK(r.valid);
        CHECK(!r.has_entry);
    }

    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
