#pragma once
/*
 * web_portal.h - LAN dashboard of the SNES RA Adapter: http://snes-ra-adapter.local/
 *
 * Ported from the Wii RA adapter dashboard (wii-ra-adapter/main/ra_web.cpp +
 * web/index.html): game header, full achievement list with filters, live
 * WebSocket pushes (unlock toast + sound, challenge, progress, rich presence),
 * theme switch and a hidden debug view (tap the game image 10x).
 *
 * On the SNES adapter rcheevos runs on the Pico, so the Pico is the single
 * source of truth and the ESP32 keeps no achievement data: when a browser asks
 * for the list (WebSocket message "list"), the ESP32 sends ACH_LIST=<from> to
 * the Pico and relays every ALH= / AL= / ALE= line to the browsers as is; the
 * next chunk is only requested when the Pico says ALN=<next> (flow control).
 * Live events (A= / C= / P=) are pushed as small JSON messages. The ESP32 only
 * keeps the game header (GAME_INFO), the rich presence of the last r=ping and
 * a few diagnostic counters.
 *
 * Threading: the AsyncWebServer handlers run on the async_tcp task, the
 * Pico/serial handlers on the Arduino loop task. The small shared state is
 * guarded by w_mux; LCD drawing, EEPROM and the Pico UART stay on the loop
 * task (handlers only set flags that web_loop() acts on).
 *
 * Included textually by snes-esp-firmware.ino after its globals.
 */

#ifdef ENABLE_INTERNAL_WEB_APP_SUPPORT

#include <esp_system.h>
#include <freertos/semphr.h>

// Sketch functions used here (defined further down in the .ino)
void config_mDNS();
#ifdef ENABLE_LCD
void lcd_redraw_for_theme();
void save_theme_to_eeprom(uint8_t idx);
#endif
void apply_lcd_brightness();
void save_brightness_to_eeprom(uint8_t pct);

#define WEB_MAX_WS_CLIENTS 3
#define WEB_LIST_TIMEOUT_MS 3000   // no answer from the Pico (old firmware?)

// ---------------------------------------------------------------------------
// Shared state (guarded by w_mux)
// ---------------------------------------------------------------------------

static SemaphoreHandle_t w_mux = nullptr;
#define WEB_LOCK()   xSemaphoreTake(w_mux, portMAX_DELAY)
#define WEB_UNLOCK() xSemaphoreGive(w_mux)

static uint32_t  w_gid = 0;                 // game loaded on the Pico (GAME_INFO)
static char      w_title[128] = "";
static char      w_img[160] = "";
static char      w_rp[128] = "";            // rich presence (m= of the last r=ping)
static char      w_err[48] = "";            // last error shown on the LCD
static char      w_pico_ver[24] = "";
static uint16_t  w_total = 0, w_unl_cnt = 0;  // ACH_SUMMARY (+ live unlocks)

// Achievement list relay (loop task only, except the request flag)
static volatile bool w_list_req = false;    // a browser asked for the list
static bool      w_list_active = false;     // waiting for Pico lines
static uint32_t  w_list_deadline = 0;
static uint32_t  w_list_relayed = 0;        // lists relayed since boot (diag)

// Portal plumbing
static AsyncWebServer *server = nullptr;
static AsyncWebSocket *ws = nullptr;
static bool      w_started = false;
static uint32_t  w_last_cleanup = 0;
static uint32_t  w_start_retry_ms = 0;
static volatile int8_t   w_pending_theme = -1;
static volatile int8_t   w_pending_brightness = -1;   // % (debug view)
static volatile uint32_t w_reboot_at = 0;

// Diagnostics
static volatile uint32_t w_wifi_drops = 0, w_wifi_last_drop_ms = 0;
static volatile uint8_t  w_wifi_last_reason = 0;
static uint32_t  w_http_errs = 0, w_http_last_ms = 0;
static int       w_http_last_code = 0;
static char      w_http_last_req[16] = "";

#define WEB_DIAG_MAGIC 0x534E4553u   // "SNES"
static RTC_NOINIT_ATTR uint32_t w_rtc_magic;
static RTC_NOINIT_ATTR uint32_t w_rtc_boots;
static RTC_NOINIT_ATTR uint32_t w_rtc_brownouts;
static esp_reset_reason_t w_reset_reason = ESP_RST_UNKNOWN;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

static void json_esc_print(Print &o, const char *s)
{
  for (; s && *s; s++) {
    uint8_t c = (uint8_t)*s;
    if (c == '"' || c == '\\') { o.write('\\'); o.write(c); }
    else if (c < 0x20)         { o.printf("\\u%04x", c); }
    else                       { o.write(c); }
  }
}

static size_t json_esc_buf(char *dst, size_t cap, size_t o, const char *s)
{
  for (size_t i = 0; s && s[i] && o + 7 < cap; i++) {
    uint8_t c = (uint8_t)s[i];
    if (c == '"' || c == '\\') { dst[o++] = '\\'; dst[o++] = (char)c; }
    else if (c < 0x20)         { o += snprintf(dst + o, cap - o, "\\u%04x", c); }
    else                       { dst[o++] = (char)c; }
  }
  if (o < cap) dst[o] = '\0';
  return o;
}

static void copy_str(char *dst, size_t cap, const char *src, size_t len)
{
  if (len >= cap) len = cap - 1;
  memcpy(dst, src, len);
  dst[len] = '\0';
}

// Value of key in an x-www-form-urlencoded body, URL-decoded ('+' = space).
static bool form_get(const char *data, const char *key, char *out, size_t cap)
{
  size_t klen = strlen(key);
  const char *p = data;
  while (p && *p) {
    if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
      p += klen + 1;
      size_t o = 0;
      while (*p && *p != '&' && o + 1 < cap) {
        char c = *p++;
        if (c == '+') c = ' ';
        else if (c == '%' && isxdigit((uint8_t)p[0]) && isxdigit((uint8_t)p[1])) {
          char hx[3] = { p[0], p[1], 0 };
          c = (char)strtoul(hx, nullptr, 16);
          p += 2;
        }
        out[o++] = c;
      }
      out[o] = '\0';
      return true;
    }
    p = strchr(p, '&');
    if (p) p++;
  }
  return false;
}

static inline bool ws_has_clients() { return ws != nullptr && ws->count() > 0; }

static void ws_push(const char *json)
{
  if (ws_has_clients()) ws->textAll(json);
}

// ---------------------------------------------------------------------------
// Hooks called by the sketch (loop task)
// ---------------------------------------------------------------------------

static void web_diag_early_init()
{
  w_reset_reason = esp_reset_reason();
  if (w_reset_reason == ESP_RST_POWERON || w_rtc_magic != WEB_DIAG_MAGIC) {
    w_rtc_magic = WEB_DIAG_MAGIC;
    w_rtc_boots = 0;
    w_rtc_brownouts = 0;
  }
  w_rtc_boots++;
  if (w_reset_reason == ESP_RST_BROWNOUT) w_rtc_brownouts++;
  if (w_mux == nullptr) w_mux = xSemaphoreCreateMutex();
}

// GAME_INFO=<id>;<title>;<image_url>
static void web_on_game_info(const char *id, const char *title, const char *img_url)
{
  WEB_LOCK();
  w_gid = strtoul(id, nullptr, 10);
  copy_str(w_title, sizeof(w_title), title, strlen(title));
  copy_str(w_img, sizeof(w_img), img_url, strlen(img_url));
  if (strncmp(w_img, "http", 4) != 0) w_img[0] = '\0';   // "No URL"
  w_err[0] = '\0';
  WEB_UNLOCK();
  ws_push("{\"e\":\"reload\"}");
}

static void web_on_summary(uint16_t unlocked, uint16_t total)
{
  WEB_LOCK();
  w_unl_cnt = unlocked;
  w_total = total;
  WEB_UNLOCK();
  ws_push("{\"e\":\"reload\"}");
}

// A=<id>;<title>;<badge_url>
static void web_on_unlock(uint32_t id, const char *title, const char *badge_url)
{
  WEB_LOCK();
  w_unl_cnt++;
  WEB_UNLOCK();
  char buf[400];
  size_t o = snprintf(buf, sizeof(buf), "{\"e\":\"unlock\",\"id\":%lu,\"t\":\"", (unsigned long)id);
  o = json_esc_buf(buf, sizeof(buf), o, title);
  o += snprintf(buf + o, sizeof(buf) - o, "\",\"b\":\"");
  o = json_esc_buf(buf, sizeof(buf), o, badge_url);
  snprintf(buf + o, sizeof(buf) - o, "\"}");
  ws_push(buf);
}

// C=S;<id>;<title>;<url>;<progress>  C=H;<id>  P=S;...  P=H;<id>
static void web_on_indicator(const char *line, size_t len)
{
  if (len < 4 || line[1] != '=' || line[3] != ';' || !ws_has_clients()) return;
  char tmp[256];
  copy_str(tmp, sizeof(tmp), line, len);
  uint32_t id = strtoul(tmp + 4, nullptr, 10);
  bool show = (tmp[2] == 'S');
  char m[24] = "";
  if (show) {
    const char *last = strrchr(tmp, ';');
    if (last) copy_str(m, sizeof(m), last + 1, strlen(last + 1));
  }
  char buf[96];
  if (tmp[0] == 'C') {
    snprintf(buf, sizeof(buf), "{\"e\":\"challenge\",\"show\":%d,\"id\":%lu}", show ? 1 : 0, (unsigned long)id);
  } else {
    size_t o = snprintf(buf, sizeof(buf), "{\"e\":\"progress\",\"show\":%d,\"id\":%lu,\"m\":\"",
                        show ? 1 : 0, (unsigned long)id);
    o = json_esc_buf(buf, sizeof(buf), o, m);
    snprintf(buf + o, sizeof(buf) - o, "\"}");
  }
  ws_push(buf);
}

static void web_on_console_reset()
{
  ws_push("{\"e\":\"reload\"}");
}

static void web_on_error(const char *msg)
{
  WEB_LOCK();
  copy_str(w_err, sizeof(w_err), msg, strlen(msg));
  WEB_UNLOCK();
  ws_push("{\"e\":\"reload\"}");
}

static void web_on_pico_version(const char *v, size_t len)
{
  while (len > 0 && (v[len - 1] == '\r' || v[len - 1] == '\n' || v[len - 1] == ' ')) len--;
  WEB_LOCK();
  copy_str(w_pico_ver, sizeof(w_pico_ver), v, len);
  WEB_UNLOCK();
}

static void web_on_wifi_drop(uint8_t reason)
{
  w_wifi_drops++;
  w_wifi_last_reason = reason;
  w_wifi_last_drop_ms = millis();
}

// A REQ= from the Pico, before it goes out: the rich presence rides on r=ping
static void web_on_request(const char *data)
{
  if (strncmp(data, "r=ping", 6) != 0) return;
  char v[128];
  if (!form_get(data, "m", v, sizeof(v))) return;
  WEB_LOCK();
  bool changed = strcmp(v, w_rp) != 0;
  if (changed) copy_str(w_rp, sizeof(w_rp), v, strlen(v));
  WEB_UNLOCK();
  if (changed) {
    char buf[300];
    size_t o = snprintf(buf, sizeof(buf), "{\"e\":\"rp\",\"m\":\"");
    o = json_esc_buf(buf, sizeof(buf), o, v);
    snprintf(buf + o, sizeof(buf) - o, "\"}");
    ws_push(buf);
  }
}

static void web_on_request_failed(const char *data, int code)
{
  w_http_errs++;
  w_http_last_code = code;
  w_http_last_ms = millis();
  const char *r = strncmp(data, "r=", 2) == 0 ? data + 2 : data;
  size_t n = 0;
  while (r[n] && r[n] != '&' && n < sizeof(w_http_last_req) - 1) n++;
  copy_str(w_http_last_req, sizeof(w_http_last_req), r, n);
}

// A WebSocket text frame must be valid UTF-8 (browsers drop the connection
// otherwise), so a line garbled on the UART is not relayed.
static bool utf8_valid(const uint8_t *s, size_t n)
{
  for (size_t i = 0; i < n;) {
    uint8_t c = s[i];
    size_t k = c < 0x80 ? 0 : (c & 0xE0) == 0xC0 ? 1 : (c & 0xF0) == 0xE0 ? 2 : (c & 0xF8) == 0xF0 ? 3 : 9;
    if (k == 9 || i + k >= n + (k == 0)) return false;   // bad lead byte / cut sequence
    for (size_t j = 1; j <= k; j++) if ((s[i + j] & 0xC0) != 0x80) return false;
    i += k + 1;
  }
  return true;
}

// ALH= / AL= / ALN= / ALE= from the Pico (answers to ACH_LIST=<from>)
static void web_on_list_line(const char *line, size_t len)
{
  if (len >= 4 && strncmp(line, "ALN=", 4) == 0) {
    // chunk done: ask for the next one only while somebody is still watching
    if (w_list_active && ws_has_clients()) {
      Serial0.print(F("ACH_LIST="));
      Serial0.write((const uint8_t *)line + 4, len - 4);
      Serial0.print(F("\r\n"));
      w_list_deadline = millis() + WEB_LIST_TIMEOUT_MS;
    } else {
      w_list_active = false;
    }
    return;
  }
  if (ws_has_clients() && utf8_valid((const uint8_t *)line, len))
    ws->textAll(line, len);   // relayed as is
  if (len >= 4 && strncmp(line, "ALE=", 4) == 0) {
    w_list_active = false;
    w_list_relayed++;
    Serial.print(F("[web] achievement list relayed: "));
    Serial.write(line + 4, len - 4);
    Serial.println();
  }
}

// ---------------------------------------------------------------------------
// HTTP handlers (async_tcp task)
// ---------------------------------------------------------------------------

static const char *web_state_text()
{
  if (w_err[0]) return w_err;
  switch (state) {
    case STATE_UNINITIALIZED:       return "starting";
    case STATE_IDENTIFY_CARTRIDGE:
    case STATE_WAITING_CART_INFO:   return "identifying cartridge";
    case STATE_CART_INFO_RECEIVED:
    case STATE_WATCHING:            return w_gid ? "game loaded - waiting for the console" : "loading game";
    case STATE_IDLE:                return w_gid ? "in game" : "idle";
    default:                        return isErrorState(state) ? getStateErrorMessage(state) : "?";
  }
}

static void handle_api_state(AsyncWebServerRequest *req)
{
  AsyncResponseStream *o = req->beginResponseStream("application/json");
  o->addHeader("Cache-Control", "no-store");
#ifdef ENABLE_LCD
  int theme = currentThemeIndex;
#else
  int theme = 0;
#endif
  WEB_LOCK();
  o->printf("{\"state\":%u,\"st\":\"", (unsigned)state);
  json_esc_print(*o, web_state_text());
  o->printf("\",\"gid\":%lu,\"title\":\"", (unsigned long)w_gid);
  json_esc_print(*o, w_title);
  o->print(F("\",\"img\":\""));
  json_esc_print(*o, w_img);
  o->print(F("\",\"rp\":\""));
  json_esc_print(*o, w_rp);
  o->printf("\",\"total\":%u,\"unlocked\":%u,\"theme\":%d}",
            (unsigned)w_total, (unsigned)w_unl_cnt, theme);
  WEB_UNLOCK();
  req->send(o);
}

static void handle_api_themes(AsyncWebServerRequest *req)
{
  AsyncResponseStream *o = req->beginResponseStream("application/json");
#ifdef ENABLE_LCD
  o->printf("{\"cur\":%u,\"themes\":[", (unsigned)currentThemeIndex);
  for (uint8_t i = 0; i < LCD_THEME_COUNT; i++) {
    const LcdTheme &t = *LCD_THEMES[i];
    // order: bg, header, footer, border, fill, text, ok, warn, error
    const uint16_t c[9] = { t.background, t.headerText, t.footerText, t.boxBorder, t.boxFill,
                            t.contentText, t.statusOk, t.statusWarn, t.statusError };
    o->printf("%s{\"n\":\"", i ? "," : "");
    json_esc_print(*o, t.name);
    o->print(F("\",\"c\":["));
    for (int k = 0; k < 9; k++) {
      uint8_t r5 = (c[k] >> 11) & 0x1F, g6 = (c[k] >> 5) & 0x3F, b5 = c[k] & 0x1F;
      o->printf("%s\"#%02x%02x%02x\"", k ? "," : "",
                (r5 * 527 + 23) >> 6, (g6 * 259 + 33) >> 6, (b5 * 527 + 23) >> 6);
    }
    o->print(F("]}"));
  }
  o->print(F("]}"));
#else
  o->print(F("{\"cur\":0,\"themes\":[]}"));
#endif
  req->send(o);
}

static void handle_api_theme_post(AsyncWebServerRequest *req)
{
#ifdef ENABLE_LCD
  const AsyncWebParameter *p = req->hasParam("i", true) ? req->getParam("i", true)
                             : req->hasParam("i") ? req->getParam("i") : nullptr;
  int i = p ? p->value().toInt() : -1;
  if (i < 0 || i >= LCD_THEME_COUNT) {
    req->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  w_pending_theme = (int8_t)i;   // applied (LCD + EEPROM) on the loop task
  char buf[32];
  snprintf(buf, sizeof(buf), "{\"ok\":1,\"i\":%d}", i);
  req->send(200, "application/json", buf);
#else
  req->send(404, "application/json", "{\"ok\":0}");
#endif
}

// POST /api/brightness (p=<10..100>): LCD backlight in %, applied and saved
// in the EEPROM on the loop task
static void handle_api_brightness_post(AsyncWebServerRequest *req)
{
  const AsyncWebParameter *p = req->hasParam("p", true) ? req->getParam("p", true)
                             : req->hasParam("p") ? req->getParam("p") : nullptr;
  int pct = p ? p->value().toInt() : -1;
  if (pct < LCD_MIN_BRIGHTNESS || pct > 100) {
    req->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  w_pending_brightness = (int8_t)pct;
  char buf[32];
  snprintf(buf, sizeof(buf), "{\"ok\":1,\"p\":%d}", pct);
  req->send(200, "application/json", buf);
}

static const char *reset_reason_str(esp_reset_reason_t r)
{
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT_PIN";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    case ESP_RST_USB:       return "USB";
    case ESP_RST_JTAG:      return "JTAG";
    default:                return "UNKNOWN";
  }
}

static void handle_api_diag(AsyncWebServerRequest *req)
{
  AsyncResponseStream *o = req->beginResponseStream("application/json");
  o->addHeader("Cache-Control", "no-store");
  uint32_t now = millis();
  o->printf("{\"fw\":\"%s\",\"board\":\"rev %d (%s)\",\"cpu_mhz\":%lu,\"up_s\":%lu,"
            "\"reset\":\"%s\",\"boots\":%lu,\"brownouts\":%lu,",
            VERSION, SNES_BOARD_REV, ESP.getChipModel(), (unsigned long)getCpuFrequencyMhz(),
            (unsigned long)(now / 1000), reset_reason_str(w_reset_reason),
            (unsigned long)w_rtc_boots, (unsigned long)w_rtc_brownouts);
  o->printf("\"heap\":%lu,\"heap_min\":%lu,\"heap_blk\":%lu,\"psram\":%lu,",
            (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(),
            (unsigned long)ESP.getMaxAllocHeap(), (unsigned long)ESP.getFreePsram());
  o->print(F("\"ssid\":\""));
  json_esc_print(*o, WiFi.SSID().c_str());
  o->printf("\",\"rssi\":%d,\"ch\":%d,\"ip\":\"%s\",\"wifi_drops\":%lu,\"wifi_last_reason\":%u,\"wifi_last_ago_s\":%ld,",
            (int)WiFi.RSSI(), (int)WiFi.channel(), WiFi.localIP().toString().c_str(),
            (unsigned long)w_wifi_drops, (unsigned)w_wifi_last_reason,
            w_wifi_drops ? (long)((now - w_wifi_last_drop_ms) / 1000) : -1L);
  o->printf("\"http_errs\":%lu,\"http_last_code\":%d,\"http_last_ago_s\":%ld,\"http_last\":\"",
            (unsigned long)w_http_errs, w_http_last_code,
            w_http_errs ? (long)((now - w_http_last_ms) / 1000) : -1L);
  json_esc_print(*o, w_http_last_req);
  WEB_LOCK();
  o->printf("\",\"state\":%u,\"st\":\"", (unsigned)state);
  json_esc_print(*o, web_state_text());
  o->print(F("\",\"pico\":\""));
  json_esc_print(*o, w_pico_ver);
  o->printf("\",\"md5\":\"%s\",\"gid\":%lu,", md5_global, (unsigned long)w_gid);
  WEB_UNLOCK();
  o->printf("\"bright\":%u,\"bright_def\":%u,\"lists\":%lu,\"ws\":%u,\"log_pos\":%lu,\"log_cap\":%lu}",
            (unsigned)lcd_brightness, (unsigned)LCD_DEFAULT_BRIGHTNESS,
            (unsigned long)w_list_relayed, ws ? (unsigned)ws->count() : 0u,
            (unsigned long)g_log_total, (unsigned long)g_log_tail_cap);
  req->send(o);
}

// GET /api/log?from=N -> log bytes from absolute position N (clamped to what
// the ring still holds, at most 8 KB per call); X-Log-Start / X-Log-Pos give
// the range actually sent. ?boot=1 -> the frozen boot capture. The log only
// exists on boards with PSRAM (see serial_tee.h).
static void handle_api_log(AsyncWebServerRequest *req)
{
  if (g_log_tail == nullptr || g_log_boot == nullptr) {
    req->send(404, "text/plain", "log only on boards with PSRAM");
    return;
  }
  uint32_t from = req->hasParam("from") ? strtoul(req->getParam("from")->value().c_str(), nullptr, 10) : 0;
  bool boot = req->hasParam("boot") && req->getParam("boot")->value() == "1";
  AsyncResponseStream *o = req->beginResponseStream("text/plain; charset=utf-8");
  o->addHeader("Cache-Control", "no-store");
  char chunk[512];
  uint32_t start, sent = 0;
  if (boot) {
    portENTER_CRITICAL(&g_log_mux);
    uint32_t end = g_log_total < LOG_BOOT_CAP ? g_log_total : LOG_BOOT_CAP;
    portEXIT_CRITICAL(&g_log_mux);
    start = 0;
    o->write((const uint8_t *)g_log_boot, end);   // bytes below end never change
    sent = end;
  } else {
    portENTER_CRITICAL(&g_log_mux);
    uint32_t end = g_log_total;
    portEXIT_CRITICAL(&g_log_mux);
    uint32_t oldest = end > g_log_tail_cap ? end - g_log_tail_cap : 0;
    if (from > end) from = 0;                     // adapter rebooted
    start = from < oldest ? oldest : from;
    uint32_t p = start;
    if (end - p > 8192) end = p + 8192;
    while (p < end) {
      uint32_t n = end - p;
      if (n > sizeof(chunk)) n = sizeof(chunk);
      bool lost;
      portENTER_CRITICAL(&g_log_mux);
      uint32_t now_oldest = g_log_total > g_log_tail_cap ? g_log_total - g_log_tail_cap : 0;
      lost = p < now_oldest;
      if (!lost) {
        uint32_t pos = p % g_log_tail_cap;
        uint32_t first = g_log_tail_cap - pos;
        if (first > n) first = n;
        memcpy(chunk, g_log_tail + pos, first);
        if (n > first) memcpy(chunk + first, g_log_tail, n - first);
      }
      portEXIT_CRITICAL(&g_log_mux);
      if (lost) break;                            // overrun mid-copy: client catches up
      o->write((const uint8_t *)chunk, n);
      sent += n;
      p += n;
    }
  }
  char hs[12], he[12];
  snprintf(hs, sizeof(hs), "%lu", (unsigned long)start);
  snprintf(he, sizeof(he), "%lu", (unsigned long)(start + sent));
  o->addHeader("X-Log-Start", hs);
  o->addHeader("X-Log-Pos", he);
  req->send(o);
}

static void handle_api_reboot(AsyncWebServerRequest *req)
{
  w_reboot_at = millis() + 700;   // the reply reaches the browser first
  req->send(200, "application/json", "{\"ok\":1}");
}

static const char NO_PAGE_HTML[] PROGMEM =
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>"
    "<h2>SNES RA Adapter</h2><p>The dashboard page is missing from LittleFS.</p>"
    "<p>Upload the <code>data</code> folder of <code>snes-esp-firmware</code> with the "
    "Arduino LittleFS Upload plug-in.</p><p><a style='color:#4cc2ff' href='/game'>Sniff game selector</a></p>";

static void on_websocket_event(AsyncWebSocket *srv, AsyncWebSocketClient *client,
                               AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  (void)srv; (void)arg;
  if (type == WS_EVT_CONNECT) {
    client->setCloseClientOnQueueFull(false);
  } else if (type == WS_EVT_DATA) {
    if (len == 4 && memcmp(data, "ping", 4) == 0) client->text("pong");
    else if (len == 4 && memcmp(data, "list", 4) == 0) w_list_req = true;   // sent from the loop task
  }
}

static void web_maybe_start()
{
  if (w_started || WiFi.status() != WL_CONNECTED) return;
  // Not while the big response buffer is allocated (until the patch has been
  // forwarded to the Pico): on the ESP32-C3 that leaves ~14 KB of heap, not
  // enough to serve the page, and a browser hitting the portal during the
  // patch download could starve its TLS session. The buffer shrinks right
  // after the patch, or in the error handling of loop().
  if (response.capacity() > SMALL_BUFFER_SIZE) return;
  if (w_start_retry_ms != 0 && (int32_t)(millis() - w_start_retry_ms) < 0) return;
  ws = new (std::nothrow) AsyncWebSocket("/ws");
  server = new (std::nothrow) AsyncWebServer(80);
  if (ws == nullptr || server == nullptr) {
    Serial.println(F("[web] portal alloc failed, retrying in 5 s"));
    delete ws;
    delete server;
    ws = nullptr;
    server = nullptr;
    w_start_retry_ms = millis() + 5000;
    return;
  }
  ws->onEvent(on_websocket_event);
  server->addHandler(ws);

  server->on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    // data/ ships the minified page as index.html.gz (built from web/index.html
    // by web/build.mjs); the file response serves it with Content-Encoding: gzip
    if (!LittleFS.exists("/index.html") && !LittleFS.exists("/index.html.gz")) {
      req->send_P(200, "text/html", NO_PAGE_HTML);
      return;
    }
    AsyncWebServerResponse *r = req->beginResponse(LittleFS, "/index.html", "text/html");
    r->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    req->send(r);
  });
  server->on("/api/state", HTTP_GET, handle_api_state);
  server->on("/api/themes", HTTP_GET, handle_api_themes);
  server->on("/api/theme", HTTP_POST, handle_api_theme_post);
  server->on("/api/diag", HTTP_GET, handle_api_diag);
  server->on("/api/log", HTTP_GET, handle_api_log);
  server->on("/api/reboot", HTTP_POST, handle_api_reboot);
  server->on("/api/brightness", HTTP_POST, handle_api_brightness_post);
  game_selector_route(server);
  server->serveStatic("/snd.mp3", LittleFS, "/snd.mp3").setCacheControl("max-age=86400");
  server->serveStatic("/sw.js", LittleFS, "/sw.js").setCacheControl("no-cache, no-store, must-revalidate");
  server->onNotFound([](AsyncWebServerRequest *req) { req->send(404, "text/plain", "not found"); });
  server->begin();
  config_mDNS();
  w_started = true;
  Serial.print(F("[web] portal up: http://snes-ra-adapter.local/ (http://"));
  Serial.print(WiFi.localIP());
  Serial.println(F("/)"));
}

// Called every loop() iteration.
static void web_loop()
{
  web_maybe_start();
  if (!w_started) return;

  if (millis() - w_last_cleanup > 1000) {
    w_last_cleanup = millis();
    ws->cleanupClients(WEB_MAX_WS_CLIENTS);
  }

  // achievement list: one request at a time; a request that arrives while a
  // list is streaming starts a fresh one right after it ends
  if (w_list_active && (int32_t)(millis() - w_list_deadline) >= 0) {
    w_list_active = false;
    ws_push("{\"e\":\"list_timeout\"}");
    Serial.println(F("[web] achievement list: no answer from the Pico"));
  }
  if (w_list_req && !w_list_active) {
    w_list_req = false;
    if (ws_has_clients()) {
      w_list_active = true;
      w_list_deadline = millis() + WEB_LIST_TIMEOUT_MS;
      Serial0.print(F("ACH_LIST=0\r\n"));
    }
  }

#ifdef ENABLE_LCD
  int8_t t = w_pending_theme;
  if (t >= 0) {
    w_pending_theme = -1;
    if (t != currentThemeIndex && t < LCD_THEME_COUNT) {
      currentThemeIndex = t;
      save_theme_to_eeprom(t);
      lcd_redraw_for_theme();
      Serial.print(F("[web] theme -> "));
      Serial.println(LCD_THEMES[t]->name);
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "{\"e\":\"theme\",\"i\":%d}", (int)t);
    ws_push(buf);
  }
#endif

  int8_t b = w_pending_brightness;
  if (b >= 0) {
    w_pending_brightness = -1;
    if (b != lcd_brightness) {
      lcd_brightness = b;
      apply_lcd_brightness();
      save_brightness_to_eeprom(b);
      Serial.print(F("[web] LCD brightness -> "));
      Serial.print(b);
      Serial.println(F("%"));
    }
  }

  if (w_reboot_at != 0 && (int32_t)(millis() - w_reboot_at) >= 0) {
    Serial.println(F("[web] reboot requested from the portal"));
    Serial0.print(F("RESET\r\n"));   // keep the Pico in sync, as in the setup flow
    Serial0.flush();
    delay(300);
    ESP.restart();
  }
}

#else  // !ENABLE_INTERNAL_WEB_APP_SUPPORT: hooks compile to nothing

static inline void web_diag_early_init() {}
static inline void web_on_game_info(const char *, const char *, const char *) {}
static inline void web_on_summary(uint16_t, uint16_t) {}
static inline void web_on_unlock(uint32_t, const char *, const char *) {}
static inline void web_on_indicator(const char *, size_t) {}
static inline void web_on_console_reset() {}
static inline void web_on_error(const char *) {}
static inline void web_on_pico_version(const char *, size_t) {}
static inline void web_on_wifi_drop(uint8_t) {}
static inline void web_on_request(const char *) {}
static inline void web_on_request_failed(const char *, int) {}
static inline void web_on_list_line(const char *, size_t) {}
static inline void web_loop() {}
static inline void log_tee_alloc() {}

#endif // ENABLE_INTERNAL_WEB_APP_SUPPORT
