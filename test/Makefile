# Host build of the base<->bulb wire-compatibility test. Uses the bulb's real
# protocol parser.
#   make        # build and run
CC ?= cc
BULB ?= ../../bulb-firmware/main
# Enable the bulb's legacy 0x01 parsing so the `dfu <id> legacy` compat test
# exercises a bulb that still accepts the deprecated packet.
CFLAGS ?= -I$(BULB) -Wall -Wextra -O2 -DPROTO_ENABLE_LEGACY_LIGHT_UPDATE=1

test_frame_compat: test_frame_compat.c $(BULB)/protocol.c $(BULB)/protocol.h
	$(CC) $(CFLAGS) test_frame_compat.c $(BULB)/protocol.c -o test_frame_compat
	./test_frame_compat

.PHONY: clean
clean:
	rm -f test_frame_compat test_frame_compat.exe *.obj
