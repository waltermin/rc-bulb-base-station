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
| `set <id> <r> <g> <b> <ww> <cw>` | add/update a bulb entry, `LightUpdate` (values 0..255) |
| `scaledset <id> <r> <g> <b> <ww> <cw> <scale>` | `PreciseLightUpdate`: each channel (`0..255`, fractional ok) is normalized (`/255`) then multiplied by `scale`% and sent as a float — all math in float space; one bulb at a time |
| `clr <id>` | remove one entry |
| `clrall` | remove all entries |
| `dfu <id>` | request DFU mode for a bulb (~5 s burst, then auto-clears) |
| `sweep <id> <channels> <ms>` | smooth linear `0→255→0` loop over `channels` (comma list of `r,g,b,ww,cw`) with a full cycle every `<ms>` |
| `psweep <id> <channels> <ms> <min> <max>` | precise float sweep: triangle-wave over `channels` bouncing between `min`%..`max`% of full scale every `<ms>`; one bulb at a time |
| `stopsweep` | stop the active sweep(s), linear and precise (channels freeze at their last value) |
| `chan <1..13>` | broadcast channel (bulb defaults to 1) |
| `interval <ms>` | beacon interval (default 200 ms) |
| `start` / `stop` | resume / pause broadcasting |
| `show` | print current state |

A background task broadcasts the current entry table continuously, so once you
`set` a bulb it keeps receiving updates (and its 15 s fallback never fires). Stop
broadcasting (or `clr` the entry) to watch a bulb fall back to its default color.

The `LightUpdate` (0x01) entry table and the single `PreciseLightUpdate` (0x02)
target are independent: each beacon carries only our one vendor IE, so a precise
target is broadcast as its own extra beacon each tick, right after the entry
table. Only one bulb can be precisely controlled at a time — a new `scaledset`
or `psweep` replaces the previous precise target.

## Testing flow with a bulb (id 1)

```
set 1 255 0 0 0 0        # bulb 1 -> red   (drives GPIO4 on the devboard)
set 1 0 0 0 255 0        # bulb 1 -> warm white (GPIO13)
scaledset 1 255 0 0 0 0 40  # bulb 1 -> 40% red via PreciseLightUpdate (float 0.4)
sweep 1 r,g,b 3000       # bulb 1 -> smooth 0->255->0 breathing on RGB, 3 s/cycle
psweep 1 ww 4000 10 90   # bulb 1 -> precise warm-white breathe between 10% and 90%
stopsweep                # freeze the sweep(s) where they are
clr 1                    # stop addressing it -> reverts to default after ~15 s
dfu 1                    # bulb 1 leaves promiscuous, joins the DFU AP
```

Keep the base station and bulb on the **same channel** (1 by default). Note the
bulb's console is 74880 baud, the base station's is 115200.
