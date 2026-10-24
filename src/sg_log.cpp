#include "sg_log.h"
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#if defined(BOARD_WAVESHARE_P4_LCD_4C)
#  include <esp_rom_sys.h>
#endif

static char        *s_ring = nullptr;     // PSRAM when available
static size_t       s_cap  = 0;
static size_t       s_head = 0;           // next write position
static bool         s_wrapped = false;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static void ring_alloc() {
    if (s_ring) return;
    size_t cap = SG_LOG_RING_BYTES;
    char *p = (char *)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    if (!p) { cap = 2048; p = (char *)malloc(cap); }   // no PSRAM: a small ring is still useful
    if (!p) return;
    portENTER_CRITICAL(&s_mux);
    if (!s_ring) { s_ring = p; s_cap = cap; p = nullptr; }
    portEXIT_CRITICAL(&s_mux);
    if (p) free(p);                                     // another task got there first
}

void sg_log(const char *fmt, ...) {
    char line[288];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    Serial.print(line);
#if defined(BOARD_WAVESHARE_P4_LCD_4C)
    esp_rom_printf("%s", line);                         // P4: Serial is the other UART
#endif

    // Ring copy, prefixed with uptime (the web page has no terminal timestamps).
    ring_alloc();
    if (!s_ring) return;
    char stamp[16];
    const uint32_t ms = millis();
    const int sl = snprintf(stamp, sizeof(stamp), "[%6lu.%lus] ",
                            (unsigned long)(ms / 1000), (unsigned long)((ms / 100) % 10));
    const size_t ll = strlen(line);
    const bool addNl = (ll == 0 || line[ll - 1] != '\n');
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < sl; ++i)      { s_ring[s_head] = stamp[i]; if (++s_head == s_cap) { s_head = 0; s_wrapped = true; } }
    for (size_t i = 0; i < ll; ++i)   { s_ring[s_head] = line[i];  if (++s_head == s_cap) { s_head = 0; s_wrapped = true; } }
    if (addNl)                        { s_ring[s_head] = '\n';     if (++s_head == s_cap) { s_head = 0; s_wrapped = true; } }
    portEXIT_CRITICAL(&s_mux);
}

size_t sg_log_copy(char *dst, size_t cap) {
    if (!dst || cap == 0) return 0;
    dst[0] = 0;
    if (!s_ring) return 0;
    size_t out = 0;
    portENTER_CRITICAL(&s_mux);
    const size_t total = s_wrapped ? s_cap : s_head;
    const size_t start = s_wrapped ? s_head : 0;
    for (size_t i = 0; i < total && out < cap - 1; ++i)
        dst[out++] = s_ring[(start + i) % s_cap];
    portEXIT_CRITICAL(&s_mux);
    dst[out] = 0;
    if (s_wrapped) {                                    // first line is cut off after a wrap
        char *nl = strchr(dst, '\n');
        if (nl) {
            const size_t skip = (size_t)(nl - dst) + 1;
            memmove(dst, nl + 1, out - skip + 1);
            out -= skip;
        }
    }
    return out;
}
