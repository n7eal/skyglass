#pragma once
// Shared log for the [adsb] / [gps] / [ais] lines. Each line goes where it went before
// (Serial, and on the P4 also the ROM console that reaches its USB port) and is kept in a
// RAM ring that the config web server shows at /log -- the log without a USB cable.
#include <stddef.h>

#define SG_LOG_RING_BYTES 16384     // ~250 lines

void   sg_log(const char *fmt, ...);
// Copy the ring, oldest line first, NUL-terminated. Returns the length copied.
size_t sg_log_copy(char *dst, size_t cap);
