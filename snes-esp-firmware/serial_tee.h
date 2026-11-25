#pragma once
/*
 * serial_tee.h - copies everything the sketch prints on Serial into PSRAM, so
 * the hidden debug view of the web portal (GET /api/log) can show the log
 * without the USB cable. Ported from the Wii RA adapter (ra_web_log_tee).
 *
 * Only on boards with PSRAM (ESP32-S3, rev 2): the ESP32-C3 has no RAM to spare
 * during the patch download, so there the tee is a pass-through and the debug
 * view shows no log.
 *
 * Two buffers, positions are absolute byte counts since boot (the browser polls
 * incrementally with ?from=N):
 *   - boot: the first LOG_BOOT_CAP bytes since boot, frozen;
 *   - tail: rolling ring with the newest lines.
 *
 * Only active with ENABLE_INTERNAL_WEB_APP_SUPPORT. Must be included after the
 * core headers and before any use of Serial: from here on, "Serial" in the
 * sketch is the tee; libraries keep printing straight to the real port.
 */
#ifdef ENABLE_INTERNAL_WEB_APP_SUPPORT

#include <Arduino.h>

#define LOG_BOOT_CAP (8 * 1024)
#define LOG_TAIL_CAP (32 * 1024)

static char             *g_log_boot = nullptr;
static char             *g_log_tail = nullptr;
static uint32_t          g_log_tail_cap = 0;
static volatile uint32_t g_log_total = 0;        // bytes printed since boot
static volatile bool     g_log_mute = false;     // skip huge dumps (patch JSON)
static portMUX_TYPE      g_log_mux = portMUX_INITIALIZER_UNLOCKED;

static void log_tee(const uint8_t *buf, size_t len)
{
  if (g_log_tail == nullptr || g_log_mute || buf == nullptr || len == 0) return;
  portENTER_CRITICAL(&g_log_mux);
  uint32_t total = g_log_total;
  if (total < LOG_BOOT_CAP) {
    uint32_t n = LOG_BOOT_CAP - total;
    if (n > len) n = len;
    memcpy(g_log_boot + total, buf, n);
  }
  const uint8_t *src = buf;
  size_t n = len;
  uint32_t at = total;
  if (n > g_log_tail_cap) {               // keep only what fits
    src += n - g_log_tail_cap;
    at  += n - g_log_tail_cap;
    n    = g_log_tail_cap;
  }
  uint32_t pos = at % g_log_tail_cap;
  uint32_t first = g_log_tail_cap - pos;
  if (first > n) first = n;
  memcpy(g_log_tail + pos, src, first);
  if (n > first) memcpy(g_log_tail, src + first, n - first);
  g_log_total = total + len;
  portEXIT_CRITICAL(&g_log_mux);
}

// Call first thing in setup(): PSRAM is ready before setup() runs.
static void log_tee_alloc()
{
  if (g_log_tail != nullptr || !psramFound()) return;
  char *boot = (char *)ps_malloc(LOG_BOOT_CAP);
  char *tail = (char *)ps_malloc(LOG_TAIL_CAP);
  if (boot == nullptr || tail == nullptr) {
    free(boot);
    free(tail);
    return;
  }
  g_log_boot = boot;
  g_log_tail_cap = LOG_TAIL_CAP;
  g_log_tail = tail;
}

// Whatever "Serial" is for the selected board options (HWCDCSerial on the
// ESP32-C3 / S3 with "USB CDC On Boot: Enabled").
static auto &g_realSerial = Serial;

class TeeSerial : public Stream {
public:
  template <typename... Args> void begin(Args... args) { g_realSerial.begin(args...); }
  int available() override { return g_realSerial.available(); }
  int read() override { return g_realSerial.read(); }
  int peek() override { return g_realSerial.peek(); }
  void flush() override { g_realSerial.flush(); }
  size_t write(uint8_t c) override {
    log_tee(&c, 1);
    return g_realSerial.write(c);
  }
  size_t write(const uint8_t *buf, size_t size) override {
    log_tee(buf, size);
    return g_realSerial.write(buf, size);
  }
  using Print::write;
};

static TeeSerial g_teeSerial;

#undef Serial
#define Serial g_teeSerial

#endif // ENABLE_INTERNAL_WEB_APP_SUPPORT
