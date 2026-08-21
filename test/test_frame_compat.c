// test_frame_compat.c — cross-firmware wire-compatibility check.
//
// Builds beacon frames byte-for-byte the way the ESP32 base station's
// build_beacon() / build_precise_beacon() do, then decodes them with the ACTUAL
// bulb parser (bulb-firmware/main/protocol.c). This proves the two firmwares
// agree on the wire format without needing either microcontroller.
//
// Build & run (from this dir):
//   cc -I../../bulb-firmware/main test_frame_compat.c ../../bulb-firmware/main/protocol.c -o t && ./t

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
#define MAX_ENTRIES 11
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

static int build_beacon(uint8_t *p, const uint8_t base_mac[6],
                        uint8_t control_flags, uint8_t control_data,
                        const entry_t *entries, int n_entries) {
    int n = write_header(p, base_mac);
    uint8_t body[4 + MAX_ENTRIES * ENTRY_SIZE];
    int b = 0;
    body[b++] = TAG_LIGHT_UPDATE;                 // packet tag 0x01
    body[b++] = control_flags;
    body[b++] = control_data;
    int count_idx = b++;
    for (int i = 0; i < n_entries; i++) {
        body[b++] = entries[i].id;
        body[b++] = entries[i].r;
        body[b++] = entries[i].g;
        body[b++] = entries[i].b;
        body[b++] = entries[i].ww;
        body[b++] = entries[i].cw;
    }
    body[count_idx] = (uint8_t)n_entries;
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

    // Base station sets bulbs 2 and 7; bulb 7 decodes its own entry.
    {
        entry_t e[] = {{2, 10, 20, 30, 40, 50}, {7, 200, 150, 100, 60, 25}};
        int len = build_beacon(buf, mac, 0, 0, e, 2);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 7);
        printf("base sets bulbs 2 & 7; bulb 7 decodes:\n");
        CHECK(r.valid);
        CHECK(r.has_entry);
        CHECK(!r.dfu_requested);
        CHECK(FEQ(r.r, 200 / 255.0f) && FEQ(r.g, 150 / 255.0f) && FEQ(r.b, 100 / 255.0f) &&
              FEQ(r.ww, 60 / 255.0f) && FEQ(r.cw, 25 / 255.0f));
    }

    // Same frame, a bulb not addressed (id 9) -> valid but no entry.
    {
        entry_t e[] = {{2, 10, 20, 30, 40, 50}, {7, 200, 150, 100, 60, 25}};
        int len = build_beacon(buf, mac, 0, 0, e, 2);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 9);
        printf("bulb 9 not addressed:\n");
        CHECK(r.valid);
        CHECK(!r.has_entry);
    }

    // DFU burst for bulb 1.
    {
        int len = build_beacon(buf, mac, 0x01, 1, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 1);
        printf("base 'dfu 1' -> bulb 1 sees DFU:\n");
        CHECK(r.valid);
        CHECK(r.dfu_requested);
    }

    // DFU targeted at bulb 1, decoded by bulb 2 -> not its DFU.
    {
        int len = build_beacon(buf, mac, 0x01, 1, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 2);
        printf("base 'dfu 1' -> bulb 2 ignores:\n");
        CHECK(r.valid);
        CHECK(!r.dfu_requested);
    }

    // Full 11-entry table still fits and decodes.
    {
        entry_t e[MAX_ENTRIES];
        for (int i = 0; i < MAX_ENTRIES; i++)
            e[i] = (entry_t){(uint8_t)(i + 1), (uint8_t)i, 0, 0, 0, 0};
        int len = build_beacon(buf, mac, 0, 0, e, MAX_ENTRIES);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 11);
        printf("full 11-entry table, bulb 11:\n");
        CHECK(r.valid);
        CHECK(r.has_entry);
        CHECK(FEQ(r.r, 10 / 255.0f));
    }

    // Empty table (base broadcasting nothing) is still a valid frame.
    {
        int len = build_beacon(buf, mac, 0, 0, NULL, 0);
        bulb_parse_result_t r = protocol_parse_beacon(buf, len, 1);
        printf("empty entry table:\n");
        CHECK(r.valid);
        CHECK(!r.has_entry);
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
