# rc-base-station (ESP32)

A serial-driven **base station** for testing `bulb-firmware`. It injects raw
802.11 beacon frames carrying the RC light control protocol (vendor-specific IE,
OUI `52:43:68`, as the **only** IE — which is exactly what the bulb's strict
parser requires and what a normal AP/hostapd cannot produce).

Built on **ESP-IDF** (ESP32) — a different SDK from the ESP8266 bulb firmware.
Uses `esp_wifi_80211_tx()` to transmit, with WiFi started in STA mode but never
connected.

## Why an ESP32 and not the laptop

Raw 802.11 beacon injection isn't possible with native Windows WiFi drivers, and
a normal access point adds mandatory IEs the bulb rejects. An ESP crafting the
frame directly is the simplest, most reliable injector — and mirrors a real
production base station.

## Build & flash

With ESP-IDF installed and exported (`. $HOME/esp/esp-idf/export.sh`, or the
Windows ESP-IDF prompt):

```sh
cd base-station
idf.py set-target esp32
idf.py -p COM9 flash monitor
```

`idf.py monitor` gives you the interactive console directly. Alternatively use
the bundled bridge from any shell:

```sh
python3 tools/base_console.py COM9
```

## Console commands

Console runs at **115200 baud**. Type `help` for the list.

| Command | Effect |
|---|---|
| `set <id> <r> <g> <b> <ww> <cw>` | add/update a bulb entry, `LightUpdateV2` (values 0..255) |
| `setrange <first> <last> <r> <g> <b> <ww> <cw>` | add/update an entry for **every** id in `first..last` (inclusive) with the same color. Atomic: fails without changing anything if the range won't fit in the table |
| `scaledset <id> <r> <g> <b> <ww> <cw> <scale>` | `PreciseLightUpdate`: each channel (`0..255`, fractional ok) is normalized (`/255`) then multiplied by `scale`% and sent as a float — all math in float space; one bulb at a time |
| `clr <id>` | remove one entry |
| `clrall` | remove all entries |
| `dfu <id> [new\|legacy]` | request DFU mode for a bulb. `new` (default) sends a `BulbCommand` (`0x04`); `legacy` sends the deprecated `LightUpdate` DFU (`0x01`) for bulbs built with legacy processing enabled |
| `dfurange <first> <last>` | request DFU for every id in `first..last`, one `BulbCommand` burst (10× @ 100 ms) per bulb in sequence; prints progress after each burst completes. Runs in the background (~1 s per bulb); `show` reports if one is in progress |
| `setconfig <id> <key> <u8\|u32\|f32\|str> <value>` | set a bulb config key via `BulbCommand`. `key` is the 16-bit config tag (e.g. `0x0031` = gamma); `value` is encoded little-endian per the given type |
| `seq [value]` | show, or re-seed, the `BulbCommand` sequence counter |
| `sweep <id> <channels> <ms>` | smooth linear `0→255→0` loop over `channels` (comma list of `r,g,b,ww,cw`) with a full cycle every `<ms>` |
| `sweeprange <first> <last> <channels> <ms>` | the same linear sweep driving **every** id in `first..last` in lockstep (one shared phase). Replaces any active `sweep`; entries are created as needed |
| `psweep <id> <channels> <ms> <min> <max>` | precise float sweep: triangle-wave over `channels` bouncing between `min`%..`max`% of full scale every `<ms>`; one bulb at a time |
| `stopsweep` | stop the active sweep(s), linear and precise (channels freeze at their last value) |
| `chan <1..13>` | broadcast channel (bulb defaults to 11) |
| `rate <mbps>` | PHY rate for injected frames: `1`, `2`, `5.5`, `11` (802.11b) or `6`..`54` (802.11g). Default **11** |
| `interval <ms>` | beacon interval (default 200 ms) |
| `start` / `stop` | resume / pause broadcasting |
| `show` | print current state (incl. the next `seq`) |

A background task broadcasts the current entry table continuously, so once you
`set` a bulb it keeps receiving updates (and its 15 s fallback never fires). Stop
broadcasting (or `clr` the entry) to watch a bulb fall back to its default color.

The entry table holds up to **64** bulbs. The bulb firmware only accepts **11**
entries per packet (`PROTO_MAX_ENTRIES`), so each tick the table is sharded into
`ceil(n / 11)` back-to-back `LightUpdateV2` beacons (at most 6 for a full
table), each carrying a disjoint slice; every entry is addressed exactly once
per tick. `show` reports how many packets the current table takes.

Frames are injected at **11 Mbps** by default (`rate` changes this). At the
1 Mbps basic rate a shard costs ~1.3 ms of air, so a large table at a short
`interval` (e.g. 55 bulbs / 10 ms = 5 shards per tick) saturates the channel and
the driver's TX pool overflows with `ESP_ERR_NO_MEM`; 11 Mbps cuts that ~4x.

The `LightUpdateV2` (0x03) entry table and the single `PreciseLightUpdate` (0x02)
target are independent: each beacon carries only our one vendor IE, so a precise
target is broadcast as its own extra beacon each tick, right after the entry
table. Only one bulb can be precisely controlled at a time — a new `scaledset`
or `psweep` replaces the previous precise target.

### BulbCommand (`dfu` / `setconfig`)

`dfu` and `setconfig` issue a `BulbCommand` (`0x04`): a one-shot management
packet, distinct from the continuously-broadcast entry table. Each is
transmitted **10 times at 100 ms intervals** to ensure delivery, then stops.

Every `BulbCommand` carries a **`seq`** (starts at `0`, auto-increments per
command). Each bulb tracks the highest `seq` it has seen since power-on and acts
on a command only the first time it sees a *new* highest `seq` — so the 10-frame
burst is applied exactly once, and a stale re-send is ignored. The console prints
the `seq` each time it advances. If you **reboot the base station**, `seq` resets
to `0` while a still-powered bulb remembers a higher value and will ignore new
commands; use `seq <value>` to jump the counter back above what the bulb has
seen.

`setconfig` targets the config keys in the bulb's NVS store. Example — switch a
bulb to the gamma duty curve, then set gamma to 2.6:

```
setconfig 1 0x0030 u8 0        # duty_curve = PWM_CURVE_GAMMA
setconfig 1 0x0031 f32 2.6     # gamma = 2.6
```

## Testing flow with a bulb (id 1)

```
set 1 255 0 0 0 0        # bulb 1 -> red   (drives GPIO4 on the devboard)
setrange 1 64 0 0 255 0 0  # bulbs 1..64 -> blue, sharded over 6 packets per tick
set 1 0 0 0 255 0        # bulb 1 -> warm white (GPIO13)
scaledset 1 255 0 0 0 0 40  # bulb 1 -> 40% red via PreciseLightUpdate (float 0.4)
sweep 1 r,g,b 3000       # bulb 1 -> smooth 0->255->0 breathing on RGB, 3 s/cycle
sweeprange 1 64 ww 2000  # bulbs 1..64 -> warm-white breathing together, 2 s/cycle
psweep 1 ww 4000 10 90   # bulb 1 -> precise warm-white breathe between 10% and 90%
stopsweep                # freeze the sweep(s) where they are
clr 1                    # stop addressing it -> reverts to default after ~15 s
setconfig 1 0x0031 f32 2.6  # bulb 1 -> gamma = 2.6 (persisted to NVS)
dfu 1                    # bulb 1 leaves promiscuous, joins the DFU AP (BulbCommand)
```

Keep the base station and bulb on the **same channel** (11 by default). Note the
bulb's console is 74880 baud, the base station's is 115200.
