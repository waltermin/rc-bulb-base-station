#!/usr/bin/env python3
"""Interactive serial console for the RC light base station (ESP32).

Bidirectional bridge: prints everything the board sends, and forwards each line
you type to the board. The base-station console runs at 115200 baud.

    python3 tools/base_console.py [PORT] [BAUD]
    python3 tools/base_console.py COM9

Then type commands, e.g.:
    set 1 255 0 0 0 0     # bulb 1 -> red
    dfu 1                 # request DFU for bulb 1
    show
Ctrl-C to quit.
"""
import sys
import threading

try:
    import serial
except ImportError:
    sys.exit("pyserial required: pip install pyserial")

port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

s = serial.Serial(port, baud, timeout=0.1)
stop = False


def reader():
    while not stop:
        data = s.read(4096)
        if data:
            sys.stdout.write(data.decode("utf-8", "replace"))
            sys.stdout.flush()


t = threading.Thread(target=reader, daemon=True)
t.start()
print(f"connected to {port} @ {baud} — type commands, Ctrl-C to quit")
try:
    for line in sys.stdin:
        s.write(line.rstrip("\n").encode() + b"\r\n")
except KeyboardInterrupt:
    pass
finally:
    stop = True
    s.close()
