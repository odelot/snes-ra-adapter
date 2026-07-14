/**********************************************************************************
                    SNES RA Adapter - ESP32 Firmware

   This project is the ESP32 firmware for the SNES RA Adapter, a device that connects
   to a Super Nintendo console and enables users to connect to the RetroAchievements
   server to unlock achievements in games played on original hardware.

   The ESP32 provides Internet connectivity to the SNES and also manages a TFT screen
   to display unlocked achievements, along with a buzzer for sound effects. It stores
   Wi-Fi and RA credentials in EEPROM and allows users to configure settings via a
   smartphone connected to the ESP32's access point. Users can reset the configuration
   by holding the reset button for 5 seconds during startup. LittleFS is used to cache
   game/achievement images downloaded from RetroAchievements. The firmware communicates
   with a Raspberry Pi Pico 2 (RP2350B) over Serial0 to retrieve the cartridge MD5,
   send HTTP requests to RetroAchievements, and forward HTTP responses.

   Unlike the NES variant, the SNES adapter does NOT need a CRC32->MD5 lookup table:
   the Pico parses the SNES header itself and computes the ROM MD5 inline during the
   cart-ID phase, then ships everything to the ESP32 in a single CART_INFO message.
   Bus isolation between SNES and cartridge is also handled entirely by the Pico
   (via 74HCT245 transceivers and a Q1 NPN that drives /RESET), so the ESP32 owns
   no analog-switch / OE control pin.

   Date:             2026-05-15
   Version:          0.1-snes
   By odelot

   Arduino IDE ESP32 Boards: v3.0.7

   Libs used:
   WifiManager: https://github.com/tzapu/WiFiManager v2.0.17
   PNGdec: https://github.com/bitbank2/PNGdec v1.1.2
   TFT_eSPI lib: https://github.com/Bodmer/TFT_eSPI v2.5.43
   ESP Async WebServer: https://github.com/ESP32Async/ESPAsyncWebServer v.3.7.6
   Async TCP: https://github.com/ESP32Async/AsyncTCP v3.4.0

   Compiled with Arduino IDE 2.3.4

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.

**********************************************************************************/

/*********************************************************************************
* FEATURE FLAGS
**********************************************************************************/

#define ENABLE_RESET 1

/**
 * defines to use a aws lambda function to shrink the JSON response with the list of achievements
 * remember: you need to deploy the lambda and inform its URL
 */

#define ENABLE_SHRINK_LAMBDA 1 // 0 - disable / 1 - enable
#define SHRINK_LAMBDA_URL "https://xxxxxxxxx.execute-api.us-east-1.amazonaws.com/default/NES_RA_ADAPTER?"

/**
 enable internal web app (comment to disable)
*/

//#define ENABLE_INTERNAL_WEB_APP_SUPPORT

/**
* enable LCD (comment to disable)
*/
#define ENABLE_LCD

/**
* flip screen upside down
*/
#define FLIP_SCREEN

#include <WiFiManager.h>
#include <new> // std::nothrow
#include <EEPROM.h>

#include <StreamString.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include "HardwareSerial.h"
#include "LittleFS.h"
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <driver/uart.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Ticker.h>
#include "CharBufferStream.h"

#ifdef ENABLE_LCD
  #include <PNGdec.h>
  #include "SPI.h"
  #include <TFT_eSPI.h>
#endif

#define VERSION "1.1"

/**
 * defines for the LittleFS
 */
#define FileSys LittleFS
#define FORMAT_LITTLEFS_IF_FAILED true

/**
 * defines for the EEPROM
 * Real usage: ~100 bytes (2 ID + 1 flag + 1 user_len + ~32 user + 1 pass_len + ~64 pass)
 */
#define EEPROM_SIZE 256
#define EEPROM_ID_1 142
#define EEPROM_ID_2 208

/**
 * Bus isolation on SNES is handled entirely by the Pico (via BUFFER_OE_N on
 * its GP45 driving the /OE of the 74HCT245 bank). The ESP32 doesn't drive
 * any analog switch, so there's no equivalent of the NES ANALOG_SWITCH_PIN
 * macros here. Boot-sequencing of the SNES is the Pico's responsibility
 * (assert /RESET via Q1 -> drive bus for cart-ID -> release).
 */

/**
 * defines for sending big serial data into chunks with a delay
 */
#define SERIAL_COMM_CHUNK_SIZE 64
#define SERIAL_COMM_TX_DELAY_MS 2

#define SERIAL_MAX_PICO_BUFFER 92160 // Fase A: Pico 2 com buffer de 96KB; margem p/ prefixo RESP=
/**
 * defines for the fifo used to store achievements to be showed on screen
 */
#define ACHIEVEMENTS_FIFO_SIZE 5

/**
 * defines for the png decoder
 */
#define MAX_IMAGE_WIDTH 240

/**
 * defines for the LCD
 */
#define LCD_BRIGHTNESS_PIN 7

/**
 * defines for status led
 */
#define LED_STATUS_RED_PIN 6
#define LED_STATUS_GREEN_PIN 5

/**
 * defines for the buzzer
 */

#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_D5 587
#define NOTE_C4 262
#define NOTE_G4 392
#define NOTE_G5 784
#define NOTE_A5 880
#define NOTE_B5 988
#define NOTE_E6 1319
#define NOTE_G6 1568
#define NOTE_C7 2093
#define NOTE_D7 2349
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_G7 3136
#define NOTE_B4   494
#define NOTE_FS5  740
#define NOTE_C5  523
#define NOTE_GS4 415
#define NOTE_AS4 466


#define SOUND_PIN 10

/**
 * defines for the reset button
 */

// time that reset button should be pressed to kick off reset operation
#define RESET_PRESSED_TIME 5000L

// pin to the reset button
#define RESET_PIN 8

// types for http request
typedef enum HttpRequestMethod
{
  GET = 0,
  POST = 1
};

typedef enum HttpRequestResult
{
  HTTP_SUCCESS = 0,
  HTTP_ERR_NO_WIFI = -1,
  HTTP_ERR_REQUEST_FAILED = -2,
  HTTP_ERR_HTTP_4XX = -3,
  HTTP_ERR_TIMEOUT = -4,
  HTTP_ERR_REPONSE_TOO_BIG = -5,
};

// Device state machine states
//
// SNES adapter flow vs NES:
//   STATE_WAITING_CART_INFO replaces the old STATE_WAITING_CART_INFO -- on SNES the
//   Pico computes the MD5 of the ROM during cart-ID and ships everything in a
//   single CART_INFO message, so there's no CRC->MD5 lookup step.
typedef enum DeviceState {
  STATE_IDENTIFY_CARTRIDGE      = 0,
  STATE_WAITING_CART_INFO       = 1,
  STATE_CART_INFO_RECEIVED      = 2,
  STATE_WATCHING                = 3,
  STATE_IDLE                    = 128,
  STATE_ERROR_CONNECTIVITY      = 198,
  STATE_ERROR_RESPONSE_TOO_BIG  = 199,
  STATE_ERROR_LOGIN_FAILED      = 253,
  STATE_ERROR_CARTRIDGE_NOT_FOUND = 254,
  STATE_UNINITIALIZED           = 255
} DeviceState;

// Check if state is an error state
inline bool isErrorState(DeviceState s) {
  return s >= STATE_ERROR_CONNECTIVITY;
}

// Get error message for a given state
const char* getStateErrorMessage(DeviceState s) {
  switch(s) {
    case STATE_ERROR_CONNECTIVITY:
      return "Connectivity Error";
    case STATE_ERROR_RESPONSE_TOO_BIG:
      return "RA response is too big";
    case STATE_ERROR_LOGIN_FAILED:
      return "Could not log in RA!";
    case STATE_ERROR_CARTRIDGE_NOT_FOUND:
      return "Cartridge not identified";
    default:
      return "Unknown error";
  }
}

// type to store the achievements to be showed on screen
typedef struct
{
  uint32_t id;
  String title;
  String url;
} achievements_t;

// type to store the achievement fifo
typedef struct
{
  achievements_t buffer[ACHIEVEMENTS_FIFO_SIZE];
  int head;
  int tail;
  int count;
} achievements_FIFO_t;

// types for LED control

enum LedMode {
  LED_OFF,
  LED_ON,
  LED_BLINK_SLOW,
  LED_BLINK_MEDIUM,
  LED_BLINK_FAST
};

enum LedColor {
  LED_RED,
  LED_GREEN,
  LED_YELLOW  
};

struct Led {
  uint8_t pin;
  LedMode mode;
  bool state; // HIGH ou LOW atual
  Ticker ticker;
};

// Forward declarations for functions using custom types
// (needed because Arduino IDE auto-generates prototypes before type definitions)
void updateLed(Led* led);
void configureTicker(Led* led);
void setSemaphore(LedMode mode, LedColor color);
void fifo_init(achievements_FIFO_t *fifo);
bool fifo_is_empty(achievements_FIFO_t *fifo);
bool fifo_is_full(achievements_FIFO_t *fifo);
bool fifo_enqueue(achievements_FIFO_t *fifo, achievements_t value);
bool fifo_dequeue(achievements_FIFO_t *fifo, achievements_t *value);
void show_achievement(achievements_t achievement);


// global variables for LED control
Led ledRed   = {LED_STATUS_RED_PIN, LED_OFF, false, Ticker()};
Led ledGreen = {LED_STATUS_GREEN_PIN, LED_OFF, false, Ticker()};


#ifdef ENABLE_LCD
// global variables for the PNG decoder - pointer to save memory during patch download
PNG* png = nullptr;
int16_t x_pos = 0;
int16_t y_pos = 0;
File png_file;

// global variables for the TFT screen
TFT_eSPI tft = TFT_eSPI();
#endif

// global variables for the achievements fifo
achievements_FIFO_t achievements_fifo;

// custom html and parameters for the configuration portal
const char head[] PROGMEM = "<style>#l,#i,#z{text-align:center}#i,#z{margin:15px auto}button{background-color:#0000FF;}#l{margin:0 auto;width:100%; font-size: 28px;}p{margin-bottom:-5px}[type='checkbox']{height: 20px;width: 20px;}</style><script>var w=window,d=document,e=\"password\";function cc(){z=gE(this.w);z.type!=e?z.type=e:z.type='text';z.focus();}function iB(a,b,c){a.insertBefore(b,c)}function gE(a){return d.getElementById(a)}function cE(a){return d.createElement(a)};\"http://192.168.1.1/\"==w.location.href&&(w.location.href+=\"wifi\");</script>\0";
const char html_p1[] PROGMEM = "<p id='z' style='width: 80%;'>Enter the RetroAchievements credentials below:</p>\0";
const char html_p2[] PROGMEM = "<p>&#8226; RetroAchievements user name: </p>\0";
const char html_p3[] PROGMEM = "<p>&#8226; RetroAchievements user password: </p>\0";
const char html_s[] PROGMEM = "<script>gE(\"s\").required=!0;l=cE(\"div\");l.innerHTML=\"SNES RetroAchievements Adapter\",l.id=\"l\";m=d.body.childNodes[0];iB(m,l,m.childNodes[0]);p=cE(\"p\");p.id=\"i\",p.innerHTML=\"Choose the network you want to connect with:\",iB(m,p,m.childNodes[1]),a=d.createTextNode(\" show \"+e),sp=(s=d.getElementById(\"up\").nextSibling).parentNode,(c1=d.createElement(\"input\")).type=\"checkbox\",c1.onclick=cc,c1.w=\"up\",iB(sp,c1,s),iB(sp,a,s);</script>\0";

// ws variables
#ifdef ENABLE_INTERNAL_WEB_APP_SUPPORT
AsyncWebServer* server = nullptr;
AsyncWebSocket* ws = nullptr;
uint32_t last_ws_cleanup = millis();
AsyncWebSocketClient *ws_client = NULL;
bool websocket_initialized = false;
#endif

// -----------------------------------------------------------------------------
// Seletor de jogo do modo sniff do Pico (pagina /game, para trocar do celular)
// -----------------------------------------------------------------------------
// Independente do webapp experimental: com ENABLE_INTERNAL_WEB_APP_SUPPORT a
// rota entra no server principal (em init_websocket); sem ele, sobe um
// AsyncWebServer minimo so com /game assim que o WiFi conectar.
// Lista de jogos DUPLICADA da tabela sniff_games[] do
// snes-pico-firmware/src/main.c -- manter as duas em sincronia.

static uint32_t game_switch_restart_at = 0;  // 0 = nada pendente
static bool game_selector_started = false;

static const char GAME_MENU_HTML[] PROGMEM =
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>body{font-family:sans-serif;background:#222;color:#eee;text-align:center}"
    "a{display:block;margin:12px auto;padding:16px;max-width:320px;background:#444;"
    "color:#fff;text-decoration:none;border-radius:8px;font-size:1.2em}</style>"
    "<h2>SNES RA &mdash; trocar jogo</h2>"
    "<a href='/game?n=0'>Super Mario World</a>"
    "<a href='/game?n=1'>Street Fighter II</a>"
    "<a href='/game?n=2'>Final Fantasy III (J)</a>"
    "<a href='/game?n=3'>Yoshi's Island</a>"
    "<a href='/game?n=4'>Star Fox</a>"
    "<a href='/game?n=5'>Mega Man X</a>"
    "<a href='/game?n=6'>Super Castlevania IV</a>"
    "<a href='/game?n=7'>EarthBound</a>"
    "<a href='/game?n=8'>Zelda: A Link to the Past</a>"
    "<a href='/game?n=9'>Super Punch-Out!!</a>"
    "<a href='/game?n=10'>Super Mario Kart</a>";

static void game_selector_route(AsyncWebServer *srv)
{
  srv->on("/game", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("n")) {
      String n = request->getParam("n")->value();
      Serial.print(F("GAME SWITCH -> "));
      Serial.println(n);
      Serial0.print(F("GAME="));
      Serial0.print(n);
      Serial0.print(F("\r\n"));
      Serial0.flush();
      request->send(200, "text/html",
        F("<meta name='viewport' content='width=device-width'>"
          "<h2>Trocando de jogo...</h2>"
          "<p>O adaptador vai reiniciar. Recarregue em ~20s.</p>"));
      game_switch_restart_at = millis() + 500;  // restart no loop(), pos-flush
    } else {
      request->send_P(200, "text/html", GAME_MENU_HTML);
    }
  });
}

// Chamado do loop(): no build sem webapp, cria o server minimo quando o WiFi
// estiver de pe. No build com webapp e' no-op (rota ja vai no server principal).
static void maybe_start_game_selector()
{
#ifndef ENABLE_INTERNAL_WEB_APP_SUPPORT
  if (game_selector_started || WiFi.status() != WL_CONNECTED)
    return;
  AsyncWebServer *srv = new (std::nothrow) AsyncWebServer(80);
  if (srv == nullptr)
    return;                     // sem memoria: tenta de novo no proximo loop
  game_selector_route(srv);
  srv->begin();
  game_selector_started = true; // nunca destruimos: vive ate o proximo reboot
  Serial.print(F("seletor de jogos: http://"));
  Serial.print(WiFi.localIP());
  Serial.println(F("/game"));
#endif
}

// global variables for the state machine
DeviceState state = STATE_UNINITIALIZED;

// URL base for RetroAchievements API
const char* base_url = "https://retroachievements.org/dorequest.php?";

// Achievement tracking for display
uint16_t total_achievements = 0;
uint16_t unlocked_achievements = 0;

// global variables for storing states, timestamps and useful information
String ra_user_token;

// Buffer fixo para comunicação serial com o Pico (evita fragmentação de memória)
// Reduzido para economizar RAM - comandos típicos são < 512 bytes
#define SERIAL_BUFFER_SIZE 768
char serial_buffer[SERIAL_BUFFER_SIZE];
size_t serial_buffer_len = 0;

// Flexible buffer for large HTTP responses 
#define LARGE_BUFFER_SIZE 64000 // 64 KB 
#define SMALL_BUFFER_SIZE 10240 // 10 KB
CharBufferStream response;
// HTTP client global to reuse SSL buffers (avoids fragmentation)
NetworkClientSecure globalSecureClient;
HTTPClient globalHttpClient;
bool httpClientInitialized = false;

// Cartridge MD5 - use fixed buffer instead of String to avoid fragmentation
char md5_global[34] = {0};

String game_name;
String game_image;
String game_id;
String game_session;
bool go_back_to_title_screen = false;
bool already_showed_title_screen = false;
long go_back_to_title_screen_timestamp;
unsigned long last_wifi_status_update = 0;

/**
 * functions related to LED status and control
 */

// function called by the ticker to update the LED state
void updateLed(Led* led) {
  switch (led->mode) {
    case LED_OFF:
      led->state = false;
      digitalWrite(led->pin, LOW);
      break;

    case LED_ON:
      led->state = true;
      digitalWrite(led->pin, HIGH);
      break;

    case LED_BLINK_SLOW:
    case LED_BLINK_MEDIUM:
    case LED_BLINK_FAST:
      led->state = !led->state;
      digitalWrite(led->pin, led->state);
      break;
  }
}

// configure the LED ticker based on the mode
void configureTicker(Led* led) {
  led->ticker.detach();  // Para evitar duplicatas

  float interval = 0;

  switch (led->mode) {
    case LED_BLINK_SLOW:   interval = 0.9; break;
    case LED_BLINK_MEDIUM: interval = 0.6; break;
    case LED_BLINK_FAST:   interval = 0.3; break;
    case LED_ON:
    case LED_OFF:
      updateLed(led);  // atualiza o estado imediatamente
      return;
  }

  led->ticker.attach(interval, updateLed, led);
}

// set the LED mode and color
void setSemaphore (LedMode mode, LedColor color) {
  ledRed.mode = LED_OFF;
  ledGreen.mode = LED_OFF;
  switch (color) {
    case LED_RED:
      ledRed.mode = mode;
      break;
    case LED_GREEN:
      ledGreen.mode = mode;      
      break;
    case LED_YELLOW:
      ledRed.mode = mode;
      ledGreen.mode = mode;
      break;
  }
  configureTicker(&ledRed);
  configureTicker(&ledGreen);
}


/**
 * functions to identify the cartridge
 *
 * SNES adapter note: the NES firmware had a `get_MD5(crc, first_bank, ...)`
 * helper that looked up the cartridge MD5 in `games.txt` (a 1971-entry
 * CRC32 -> MD5 lookup table). On SNES the Pico parses the cartridge header
 * itself and computes the ROM MD5 during the cart-ID phase, then sends the
 * result inline via the CART_INFO=... serial message. So no lookup table,
 * no `games.txt`, no `get_MD5()` function needed here.
 *
 * LittleFS is still used for caching game/achievement images downloaded
 * from RetroAchievements -- that part is unchanged.
 */
// ============================================================================
// JSON cleaning functions that operate directly on CharBufferStream (in-place)
// They do not copy memory - they modify the original buffer
// ============================================================================

// Remove spaces, newlines, and tabs outside of strings (in-place on CharBufferStream)
void remove_space_new_lines_buffer(CharBufferStream &buf)
{
  char* data = buf.data();
  size_t len = buf.length();
  bool inside_quotes = false;
  size_t write_idx = 0;

  for (size_t read_idx = 0; read_idx < len; read_idx++)
  {
    char c = data[read_idx];

    if (c == '"' && (read_idx == 0 || data[read_idx - 1] != '\\')) {
      inside_quotes = !inside_quotes;
    }

    if (inside_quotes || (c != ' ' && c != '\n' && c != '\r' && c != '\t')) {
      data[write_idx++] = c;
    }
  }
  buf.setLength(write_idx);
}

// Remove a complete JSON field (in-place on CharBufferStream)
void remove_json_field_buffer(CharBufferStream &buf, const char* field_to_remove)
{
  remove_space_new_lines_buffer(buf);
  
  char* data = buf.data();
  size_t len = buf.length();
  size_t field_len = strlen(field_to_remove);
  
  bool inside_quotes = false;
  bool inside_array = false;
  bool skip_field = false;
  size_t read_idx = 0, write_idx = 0, skip_init = 0;

  while (read_idx < len)
  {
    char c = data[read_idx];

    if (c == '"' && (read_idx == 0 || data[read_idx - 1] != '\\')) {
      inside_quotes = !inside_quotes;
    }

    if (c == '[' && skip_field) inside_array = true;
    if (c == ']' && skip_field) inside_array = false;

    // Detect start of field to remove
    if (inside_quotes && 
        read_idx + 1 + field_len + 1 < len &&
        strncmp(data + read_idx + 1, field_to_remove, field_len) == 0 &&
        data[read_idx + field_len + 1] == '"')
    {
      skip_field = true;
      skip_init = read_idx;
    }

    if (!skip_field) {
      data[write_idx++] = c;
    }

    // End of field
    if (skip_field && read_idx + 1 < len && data[read_idx + 1] == '}') {
      skip_field = false;
      if (skip_init > 0 && data[skip_init - 1] == ',') {
        write_idx--;
      }
    }
    else if (skip_field && data[read_idx] == ',' && !inside_array && !inside_quotes) {
      skip_field = false;
    }

    read_idx++;
  }
  buf.setLength(write_idx);
}

// Clean the string value of a field - replace with "" (in-place on CharBufferStream)
void clean_json_field_str_value_buffer(CharBufferStream &buf, const char* field_to_remove)
{
  remove_space_new_lines_buffer(buf);
  
  char* data = buf.data();
  size_t len = buf.length();
  size_t field_len = strlen(field_to_remove);
  
  bool inside_quotes = false;
  bool skip_field = false;
  bool remove_next_str = false;
  size_t read_idx = 0, write_idx = 0, skip_init = 0;

  while (read_idx < len)
  {
    char c = data[read_idx];

    if (c == '"' && (read_idx == 0 || data[read_idx - 1] != '\\'))
    {
      inside_quotes = !inside_quotes;
      if (inside_quotes && remove_next_str) {
        skip_field = true;
        data[write_idx++] = '"';
      }
      if (!inside_quotes && remove_next_str && read_idx > skip_init) {
        remove_next_str = false;
        skip_field = false;
      }
    }

    if (inside_quotes &&
        read_idx + 1 + field_len + 1 < len &&
        strncmp(data + read_idx + 1, field_to_remove, field_len) == 0 &&
        data[read_idx + field_len + 1] == '"')
    {
      remove_next_str = true;
      skip_init = read_idx + field_len + 2;
    }

    if (!skip_field) {
      data[write_idx++] = c;
    }
    read_idx++;
  }
  buf.setLength(write_idx);
}

// Clean the array value of a field - replace with [] (in-place on CharBufferStream)
void clean_json_field_array_value_buffer(CharBufferStream &buf, const char* field_to_remove)
{
  remove_space_new_lines_buffer(buf);
  
  char* data = buf.data();
  size_t len = buf.length();
  size_t field_len = strlen(field_to_remove);
  
  bool inside_quotes = false;
  bool skip_field = false;
  bool remove_next_array = false;
  int array_depth = 0;
  size_t read_idx = 0, write_idx = 0;

  while (read_idx < len)
  {
    char c = data[read_idx];

    // Update quote state first (handling escaped quotes)
    if (c == '"' && (read_idx == 0 || data[read_idx - 1] != '\\')) {
      inside_quotes = !inside_quotes;
    }

    // Only process [ and ] when NOT inside quotes
    if (!inside_quotes) {
      if (c == '[') {
        if (remove_next_array) {
          if (array_depth == 0) {
            skip_field = true;
            data[write_idx++] = '[';
          }
          array_depth++;
        }
      }
      if (c == ']') {
        if (remove_next_array) {
          array_depth--;
          if (array_depth == 0) {
            remove_next_array = false;
            skip_field = false;
          }
        }
      }
    }

    // Detect the field name to remove
    if (inside_quotes &&
        read_idx + 1 + field_len + 1 < len &&
        strncmp(data + read_idx + 1, field_to_remove, field_len) == 0 &&
        data[read_idx + field_len + 1] == '"')
    {
      remove_next_array = true;
    }

    if (!skip_field) {
      data[write_idx++] = c;
    }
    read_idx++;
  }
  buf.setLength(write_idx);
}

// Remove achievements with flags 5 (unofficial) - in-place on CharBufferStream
void remove_achievements_with_flags_5_buffer(CharBufferStream &buf)
{
  remove_space_new_lines_buffer(buf);
  
  char* data = buf.data();
  int achvStart = buf.indexOf("\"Achievements\":[");
  if (achvStart == -1) return;

  int arrayStart = buf.indexOf("[", achvStart);
  int arrayEnd = buf.indexOf("]", arrayStart);
  if (arrayStart == -1 || arrayEnd == -1) return;

  int objCount = 0;
  int pos = arrayStart + 1;
  
  while (pos < arrayEnd)
  {
    int objStart = buf.indexOf("{", pos);
    if (objStart == -1 || objStart > arrayEnd) break;

    int objEnd = objStart;
    int braces = 1;
    bool inString = false;
    while (braces > 0 && objEnd < arrayEnd) {
      objEnd++;
      char ch = data[objEnd];
      if (ch == '"' && (objEnd == 0 || data[objEnd - 1] != '\\')) {
        inString = !inString;
      }
      if (!inString) {
        if (ch == '{') braces++;
        else if (ch == '}') braces--;
      }
    }

    if (objEnd >= arrayEnd) break;
    objCount++;

    // Search for "Flags":5 inside the object
    bool hasFlags5 = false;
    for (int i = objStart; i < objEnd - 8; i++) {
      if (strncmp(data + i, "\"Flags\":5", 9) == 0) {
        hasFlags5 = true;
        break;
      }
    }

    if (hasFlags5)
    {
      int removeStart = objStart;
      while (removeStart > arrayStart && (data[removeStart - 1] == ' ' || data[removeStart - 1] == '\n'))
        removeStart--;
      if (data[removeStart - 1] == ',')
        removeStart--;
      if (objCount == 1 && objEnd + 1 < (int)buf.length() && data[objEnd + 1] == ',')
        objEnd++;

      buf.removeRange(removeStart, objEnd - removeStart + 1);
      arrayEnd = buf.indexOf("]", arrayStart);
      pos = removeStart;
    }
    else
    {
      pos = objEnd + 1;
    }
  }
}

// Helper: find closing quote handling escaped quotes
int findClosingQuote(char* data, int start, int limit) {
  for (int i = start; i < limit; i++) {
    if (data[i] == '"' && (i == 0 || data[i - 1] != '\\')) {
      return i;
    }
  }
  return -1;
}

// Remove achievements with very large MemAddr - in-place on CharBufferStream
void remove_achievements_with_long_MemAddr_buffer(CharBufferStream &buf, uint32_t maxSize)
{
  char* data = buf.data();
  int achievementsPos = buf.indexOf("\"Achievements\"");
  if (achievementsPos == -1) return;

  int arrayStart = buf.indexOf("[", achievementsPos);
  int arrayEnd = buf.indexOf("]", arrayStart);
  if (arrayStart == -1 || arrayEnd == -1) return;

  int pos = arrayStart + 1;
  
  while (pos < arrayEnd)
  {
    int objStart = buf.indexOf("{", pos);
    if (objStart == -1 || objStart >= arrayEnd) break;

    int objEnd = objStart;
    int braceCount = 1;
    bool inString = false;
    while (braceCount > 0 && objEnd + 1 < (int)buf.length()) {
      objEnd++;
      char ch = data[objEnd];
      if (ch == '"' && (objEnd == 0 || data[objEnd - 1] != '\\')) {
        inString = !inString;
      }
      if (!inString) {
        if (ch == '{') braceCount++;
        else if (ch == '}') braceCount--;
      }
    }

    if (objEnd >= arrayEnd) break;

    // Search for "MemAddr"
    int memAddrPos = -1;
    for (int i = objStart; i < objEnd - 8; i++) {
      if (strncmp(data + i, "\"MemAddr\"", 9) == 0) {
        memAddrPos = i;
        break;
      }
    }

    if (memAddrPos == -1) {
      pos = objEnd + 1;
      continue;
    }

    int valueStart = buf.indexOf("\"", memAddrPos + 9);
    if (valueStart == -1 || valueStart > objEnd) {
      pos = objEnd + 1;
      continue;
    }

    int valueEnd = findClosingQuote(data, valueStart + 1, objEnd);
    if (valueEnd == -1 || valueEnd > objEnd) {
      pos = objEnd + 1;
      continue;
    }

    int memAddrLength = valueEnd - valueStart - 1;
    
    if (memAddrLength > (int)maxSize)
    {
      int removeStart = objStart;
      int removeEnd = objEnd + 1;

      if (removeEnd < (int)buf.length() && data[removeEnd] == ',')
        removeEnd++;
      else if (removeStart > arrayStart + 1 && data[removeStart - 1] == ',')
        removeStart--;

      buf.removeRange(removeStart, removeEnd - removeStart);
      arrayEnd -= (removeEnd - removeStart);
      pos = removeStart;
    }
    else
    {
      pos = objEnd + 1;
    }
  }
}


void print_memory_stats(const char* label = "") {
  Serial.println(F("=== MEMORY STATS ==="));
  if (label[0] != '\0') { Serial.print(F("Label: ")); Serial.println(label); }
  Serial.print(F("Free heap: ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
  Serial.print(F("Largest block: ")); Serial.print(ESP.getMaxAllocHeap()); Serial.println(F(" bytes"));
  Serial.print(F("Min free heap: ")); Serial.print(ESP.getMinFreeHeap()); Serial.println(F(" bytes"));
  Serial.print(F("Buffer capacity: ")); Serial.print(response.capacity()); Serial.println(F(" bytes"));
  Serial.print(F("Buffer length: ")); Serial.print(response.length()); Serial.println(F(" bytes"));
  Serial.println(F("===================="));
}


#ifdef ENABLE_LCD
/*
 * functions related to the png decoder and file management
 */

// open a png file from the LittleFS
void *png_open(const char *file_name, int32_t *size)
{
  Serial.print(F("Attempting to open ")); Serial.println(file_name);
  png_file = FileSys.open(file_name, "r");
  *size = png_file.size();
  return &png_file;
}

// close a png file from the LittleFS
void png_close(void *handle)
{
  File png_file = *((File *)handle);
  if (png_file)
    png_file.close();
}

// read a png file from the LittleFS
int32_t png_read(PNGFILE *page, uint8_t *buffer, int32_t length)
{
  if (!png_file)
    return 0;
  page = page; // Avoid warning
  return png_file.read(buffer, length);
}

// seek a position in a png file from the LittleFS
int32_t png_seek(PNGFILE *page, int32_t position)
{
  if (!png_file)
    return 0;
  page = page; // Avoid warning
  return png_file.seek(position);
}

// callback function to draw pixels to the display
void png_draw(PNGDRAW *pDraw)
{
  uint16_t line_buffer[MAX_IMAGE_WIDTH];
  png->getLineAsRGB565(pDraw, line_buffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(x_pos, y_pos + pDraw->y, pDraw->iWidth, 1, line_buffer);
}

void initLCD () {
  tft.begin(); // initialize the TFT screen
  #ifdef FLIP_SCREEN
    tft.setRotation(2);
  #endif

  //  draw the title screen
  tft.fillScreen(TFT_YELLOW);
  tft.setTextColor(TFT_BLACK, TFT_YELLOW, true);

  tft.setCursor(10, 10, 4);
  tft.setTextSize(1);
  tft.println("RetroAchievements");
  tft.setCursor(140, 40, 4);
  tft.println("Adapter");

  tft.fillRoundRect(16, 76, 208, 128, 15, TFT_BLUE);
  tft.fillRoundRect(20, 80, 200, 120, 12, TFT_BLACK);

  tft.setCursor(60, 220, 2);
  tft.println("by Odelot & mi213");
}
#endif

/**
 * functions related to manage the fifo of achievements
 */

// initialize the fifo
void fifo_init(achievements_FIFO_t *fifo)
{
  fifo->head = 0;
  fifo->tail = 0;
  fifo->count = 0;
}

// check if the fifo is empty
bool fifo_is_empty(achievements_FIFO_t *fifo)
{
  return fifo->count == 0;
}

// check if the fifo is full
bool fifo_is_full(achievements_FIFO_t *fifo)
{
  return fifo->count == ACHIEVEMENTS_FIFO_SIZE;
}

// enqueue a achievement in the fifo
bool fifo_enqueue(achievements_FIFO_t *fifo, achievements_t value)
{
  if (fifo_is_full(fifo))
  {
    return false; // FIFO is full
  }
  fifo->buffer[fifo->tail] = value;
  fifo->tail = (fifo->tail + 1) % ACHIEVEMENTS_FIFO_SIZE;
  fifo->count++;
  return true;
}

// dequeue a achievement from the fifo
bool fifo_dequeue(achievements_FIFO_t *fifo, achievements_t *value)
{
  if (fifo_is_empty(fifo))
  {
    return false; // FIFO is empty
  }
  *value = fifo->buffer[fifo->head];
  fifo->head = (fifo->head + 1) % ACHIEVEMENTS_FIFO_SIZE;
  fifo->count--;
  return true;
}

/**
 * functions related to the sound
 */

// play a sound to get the user attention
void play_attention_sound()
{
  delay(35);
  tone(SOUND_PIN, NOTE_G4, 35);
  delay(35);
  tone(SOUND_PIN, NOTE_G5, 35);
  delay(35);
  tone(SOUND_PIN, NOTE_G6, 35);
  delay(35);
  noTone(SOUND_PIN);
  tone(SOUND_PIN, NOTE_G4, 35);
  delay(35);
  tone(SOUND_PIN, NOTE_G5, 35);
  delay(35);
  tone(SOUND_PIN, NOTE_G6, 35);
  delay(35);
  noTone(SOUND_PIN);
}

// play a sound to indicate a success
void play_success_sound()
{

  delay(130);
  tone(SOUND_PIN, NOTE_E6, 125);
  delay(130);
  tone(SOUND_PIN, NOTE_G6, 125);
  delay(130);
  tone(SOUND_PIN, NOTE_E7, 125);
  delay(130);
  tone(SOUND_PIN, NOTE_C7, 125);
  delay(130);
  tone(SOUND_PIN, NOTE_D7, 125);
  delay(130);
  tone(SOUND_PIN, NOTE_G7, 125);
  delay(125);
  noTone(SOUND_PIN);
}


// play a sound to indicate a success (FF Victory Fanfare)
void play_victory_sound()
{
  // Timing constants for the melody
  const int BIG_GAP   = 45;   // big gap 
  const int NOTE_GAP  = 30;   // normal gap between notes
  const int SIXTEENTH = 100;
  const int EIGHTH    = 200;
  const int QUARTER   = 400;
  const int DOTTED_Q  = 600;
  const int HALF      = 800;
  
  // Intro: C C C C 
  tone(SOUND_PIN, NOTE_C5);
  delay(SIXTEENTH);
  noTone(SOUND_PIN);
  delay(BIG_GAP);
  
  tone(SOUND_PIN, NOTE_C5);
  delay(SIXTEENTH);
  noTone(SOUND_PIN);
  delay(BIG_GAP);
  
  tone(SOUND_PIN, NOTE_C5);
  delay(SIXTEENTH);
  noTone(SOUND_PIN);
  delay(BIG_GAP);
  
   
  // C (long note)
  tone(SOUND_PIN, NOTE_C5);
  delay(DOTTED_Q);
  noTone(SOUND_PIN);
  delay(NOTE_GAP);
  
  // Ab (quarter)
  tone(SOUND_PIN, NOTE_GS4);
  delay(QUARTER);
  noTone(SOUND_PIN);
  delay(NOTE_GAP);
  
  // Bb (quarter)
  tone(SOUND_PIN, NOTE_AS4);
  delay(QUARTER);
  noTone(SOUND_PIN);
  delay(NOTE_GAP);
  
  // C (8th)
  tone(SOUND_PIN, NOTE_C5);
  delay(EIGHTH);
  noTone(SOUND_PIN);
  delay(SIXTEENTH + NOTE_GAP); // rest
  
  // Bb (16th)
  tone(SOUND_PIN, NOTE_AS4);
  delay(SIXTEENTH);
  noTone(SOUND_PIN);
  delay(NOTE_GAP);
  
  // C (hold - final note)
  tone(SOUND_PIN, NOTE_C5);
  delay(HALF + QUARTER);
  noTone(SOUND_PIN);

}

// play a sound to indicate an error
void play_error_sound()
{
  delay(100);
  tone(SOUND_PIN, NOTE_G4);
  delay(250);
  tone(SOUND_PIN, NOTE_C4);
  delay(500);
  noTone(SOUND_PIN);
}

// play a sound to indicate an achievement unlocked
void play_sound_achievement_unlocked()
{
  int snd_notes_achievement_unlocked[] = {
      NOTE_D5, NOTE_D5, NOTE_CS6, NOTE_D6};

  int snd_velocity_achievement_unlocked[] = {
      52,
      37,
      83,
      74,
  };

  int snd_notes_duration_achievement_unlocked[] = {
      2,
      4,
      3,
      12,
  };
  for (int this_note = 0; this_note < 4; this_note++)
  {
    int note_duration = snd_notes_duration_achievement_unlocked[this_note] * 16;
    tone(SOUND_PIN, snd_notes_achievement_unlocked[this_note], snd_velocity_achievement_unlocked[this_note]);
    delay(note_duration);
    noTone(SOUND_PIN);
  }
}


/**
 * functions related to the TFT screen and PNG decoder
 */

// print a line of text in the TFT screen
void print_line(const char* text, int line, int line_status)
{
  print_line(text, line, line_status, 0, TFT_BLACK);
}

// print a line of text in the TFT screen
void print_line_bgcolor(const char* text, int line, int line_status, int color)
{
  print_line(text, line, line_status, 0, color);
}

void print_line(const char* text, int line, int line_status, int delta) {
  print_line(text, line, line_status, delta, TFT_BLACK);
}

// print a line of text in the TFT screen with a delta (left to right)
void print_line(const char* text, int line, int line_status, int delta, int color)
{
  #ifdef ENABLE_LCD
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, color, true);
  tft.setCursor(20, 90 + line * 22, 2);
  tft.println("                                 ");
  tft.setCursor(46 + delta, 90 + line * 22, 2);
  if (text == nullptr || text[0] == '\0')
  {
    return;
  }
  tft.println(text);
  if (line_status == 2)
  {
    tft.fillCircle(32, 98 + line * 22, 7, TFT_RED);
  }
  else if (line_status == 1)
  {
    tft.fillCircle(32, 98 + line * 22, 7, TFT_YELLOW);
  }
  else if (line_status == 0)
  {
    tft.fillCircle(32, 98 + line * 22, 7, TFT_GREEN);
  }
  #endif
}

// clean the text in the TFT screen
void clean_screen_text()
{
  print_line("", 0, -1);
  print_line("", 1, -1);
  print_line("", 2, -1);
  print_line("", 3, -1);
  print_line("", 4, -1);
}

// Show WiFi signal strength icon at top-right corner
void showWifiStatus()
{
#ifdef ENABLE_LCD
  if (WiFi.status() != WL_CONNECTED) {
    // Draw X for no connection
    tft.drawLine(215, 5, 235, 20, TFT_RED);
    tft.drawLine(235, 5, 215, 20, TFT_RED);
    return;
  }
  
  int rssi = WiFi.RSSI();
  int bars = 0;
  if (rssi > -50) bars = 4;
  else if (rssi > -60) bars = 3;
  else if (rssi > -70) bars = 2;
  else if (rssi > -80) bars = 1;
  
  // Clear area and draw WiFi bars
  tft.fillRect(215, 5, 22, 18, TFT_YELLOW);
  for (int i = 0; i < 4; i++) {
    uint16_t color = (i < bars) ? TFT_GREEN : TFT_DARKGREY;
    int barHeight = 4 + i * 4;
    tft.fillRect(217 + i * 5, 22 - barHeight, 3, barHeight, color);
  }
#endif
}

// Show achievement counter at top-left corner
void showAchievementCounter()
{
#ifdef ENABLE_LCD
  if (total_achievements > 0) {
    char counter[16];
    sprintf(counter, "%d/%d", unlocked_achievements, total_achievements);
    tft.setTextColor(TFT_BLACK, TFT_YELLOW, true);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
    // Clear area first
    tft.fillRect(5, 5, 60, 18, TFT_YELLOW);
    tft.drawString(counter, 8, 8, 2);
  }
#endif
}

// show the title screen
void show_title_screen()
{
  setSemaphore(LED_ON, LED_GREEN);
#ifdef ENABLE_LCD  
  analogWrite(LCD_BRIGHTNESS_PIN, 150); // set the brightness of the TFT screen
  setCpuFrequencyMhz(160);
  tft.setTextColor(TFT_BLACK, TFT_YELLOW, true);

  tft.setTextSize(1);
  tft.setTextPadding(240);
  tft.drawString("", 0, 10, 4);
  tft.drawString("", 0, 40, 4);
  tft.setTextDatum(MC_DATUM);

  // if game name is longer than 18 chars, add ... at the end
  String esp_game_name = game_name;
  if (esp_game_name.length() > 18)
  {
    esp_game_name = esp_game_name.substring(0, 18 - 3) + "...";
  }

  tft.drawString(esp_game_name, 120, 40, 4);
  tft.setTextPadding(0);

  tft.fillRoundRect(20, 80, 200, 120, 12, TFT_BLACK);
  x_pos = 72;
  y_pos = 92;
  String file_name = "/title_" + game_id + ".png";
  int16_t rc = (png != nullptr)
      ? png->open(file_name.c_str(), png_open, png_close, png_read, png_seek, png_draw)
      : PNG_NO_BUFFER;
  if (rc == PNG_SUCCESS)
  {
    tft.startWrite();
    uint32_t dt = millis();
    rc = png->decode(NULL, 0);
    tft.endWrite();
    png->close();
  }
  
  // Show status indicators
  showWifiStatus();
  showAchievementCounter();
  
  already_showed_title_screen = true;
  setCpuFrequencyMhz(80);
#endif
}

// show the achievement screen
void show_achievement(achievements_t achievement)
{
  setSemaphore(LED_BLINK_FAST, LED_GREEN);
  
#ifdef ENABLE_INTERNAL_WEB_APP_SUPPORT
  char aux[256];
  sprintf(aux, "A=%s;%s;%s", "0", achievement.title.c_str(), achievement.url.c_str());
  send_ws_data(aux);
#endif
#ifdef ENABLE_LCD  
  analogWrite(LCD_BRIGHTNESS_PIN, 200); // set the brightness of the TFT screen
  // if achievement title is longer than 26 chars, add ... at the end
  if (achievement.title.length() > 26)
  {
    achievement.title = achievement.title.substring(0, 26 - 3) + "...";
  }

  setCpuFrequencyMhz(160);
  
  // Download achievement image first
  char file_name[64];
  sprintf(file_name, "/achievement_%d.png", achievement.id);
  try_download_file(achievement.url, file_name);
  
  // Helper lambda to redraw the achievement screen with a specific background color
  auto redrawAchievementScreen = [&](uint16_t bgColor) {
    tft.fillRoundRect(20, 80, 200, 120, 12, bgColor);
    print_line_bgcolor("New Achievement Unlocked!", 0, 0, bgColor);
    print_line_bgcolor("", 1, -1, bgColor);
    print_line_bgcolor("", 2, -1, bgColor);
    print_line_bgcolor("", 3, -1, bgColor);
    print_line_bgcolor(achievement.title.c_str(), 4, -1, bgColor);
    
    // Redraw the achievement image
    x_pos = 50;
    y_pos = 110;
    if (png == nullptr) return;
    int16_t rc = png->open(file_name, png_open, png_close, png_read, png_seek, png_draw);
    if (rc == PNG_SUCCESS)
    {
      tft.startWrite();
      png->decode(NULL, 0);
      tft.endWrite();
      png->close();
    }
  };
  
  // Initial draw with black background
  redrawAchievementScreen(TFT_BLACK);
  Serial.println(achievement.title);
  
  // Check if game was mastered (all achievements unlocked)
  //if (unlocked_achievements == 1) //test
  if (unlocked_achievements > 0 && unlocked_achievements == total_achievements)
  {
    // MASTERED! Celebration mode
    Serial.println(F("GAME MASTERED! Playing victory fanfare"));
    
    // Show MASTERED text in larger font over the footer area
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("** MASTERED! **", 120, 225, 4);  // Larger font (4), above footer
    tft.setTextDatum(TL_DATUM);  // Reset datum
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
            
    uint16_t colorBlack = TFT_BLACK;
    uint16_t colorGreen = tft.color565(0, 100, 0);  
    uint16_t colors[] = { colorGreen, colorBlack };
    int colorIndex = 0;
    
    // Flash background a few times before music
    for (int i = 0; i < 6; i++)
    {
      redrawAchievementScreen(colors[colorIndex]);
      colorIndex = (colorIndex + 1) % 2;
      delay(150);
    }
    
    // Play victory fanfare
    play_victory_sound();
    
    // Flash more after music
    for (int i = 0; i < 6; i++)
    {
      redrawAchievementScreen(colors[colorIndex]);
      colorIndex = (colorIndex + 1) % 2;
      delay(150);
    }
    
    // Restore black background
    redrawAchievementScreen(TFT_BLACK);
    
  }
  else
  {
    // Normal achievement sound
    play_sound_achievement_unlocked();
  }
  
  go_back_to_title_screen = true;
  go_back_to_title_screen_timestamp = millis();
  showAchievementCounter();
  setCpuFrequencyMhz(80);
#endif
}


/**
 * functions related to the EEPROM
 */

// initialize the EEPROM
void init_EEPROM(bool force)
{
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(0) != EEPROM_ID_1 || EEPROM.read(1) != EEPROM_ID_2 || force == true)
  {
    EEPROM.write(0, EEPROM_ID_1);
    EEPROM.write(1, EEPROM_ID_2);
    for (int i = 2; i < EEPROM_SIZE; i++)
    {
      EEPROM.write(i, 0);
    }
    EEPROM.commit();
    Serial.print(F("eeprom initialized"));
  }
}

// check if the EEPROM is configured
bool is_eeprom_configured()
{
  return EEPROM.read(2) == 1;
}

// save the RA credentials in the EEPROM
void save_configuration_info_eeprom(String ra_user, String ra_pass)
{
  
  uint8_t user_len = ra_user.length();
  uint8_t pass_len = ra_pass.length();
  EEPROM.write(2, 1);
  EEPROM.write(3, user_len);
  for (int i = 0; i < user_len; i++)
  {
    EEPROM.write(4 + i, ra_user[i]);
  }
  EEPROM.write(4 + user_len, pass_len);
  for (int i = 0; i < pass_len; i++)
  {
    EEPROM.write(5 + user_len + i, ra_pass[i]);
  }
  EEPROM.write(6 + user_len + pass_len, 0);
  EEPROM.commit();
}

// read the RA user from the EEPROM into a provided buffer
// Returns the length of the user string
size_t read_ra_user_from_eeprom(char* buffer, size_t buffer_size)
{
  uint8_t len = EEPROM.read(3);
  if (len >= buffer_size) len = buffer_size - 1;
  for (int i = 0; i < len; i++)
  {
    buffer[i] = (char)EEPROM.read(4 + i);
  }
  buffer[len] = '\0';
  return len;
}

// read the RA pass from the EEPROM into a provided buffer
// Returns the length of the pass string
size_t read_ra_pass_from_eeprom(char* buffer, size_t buffer_size)
{
  uint8_t ra_user_len = EEPROM.read(3);
  uint8_t len = EEPROM.read(4 + ra_user_len);
  if (len >= buffer_size) len = buffer_size - 1;
  for (int i = 0; i < len; i++)
  {
    buffer[i] = (char)EEPROM.read(5 + ra_user_len + i);
  }
  buffer[len] = '\0';
  return len;
}

/**
 * functions related to the wifi manager
 */

void wifi_manager_init(WiFiManager &wm)
{
  // wm.setDebugOutput(true);
  wm.setHostname("snes-ra-adapter");
  wm.setBreakAfterConfig(true);
  wm.setCaptivePortalEnable(true);
  wm.setMinimumSignalQuality(40);
  wm.setConnectTimeout(30);
  wm.setCustomHeadElement(head);
  wm.setDarkMode(true);
  wm.setAPStaticIPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
}


/**
 * functions related to the RA login
 */
String try_login_RA(String ra_user, String ra_pass)
{
  // print_memory_stats("BEFORE LOGIN");
  
  // Construir URL completa em buffer fixo
  char login_url[384];
  snprintf(login_url, sizeof(login_url), "%sr=login&u=%s&p=%s", 
           base_url, ra_user.c_str(), ra_pass.c_str());

  
  // print_memory_stats("BEFORE HTTP REQUEST (login)");
  
  int ret = perform_http_request_buffer(login_url, GET, "", 0, response, true, 3, 5000, 500);
  
  // print_memory_stats("AFTER HTTP REQUEST (login)");
  
  if (ret < 0)
  {
    Serial.print(F("request error: "));
    Serial.println(http_request_result_to_cstr(ret));
    state = STATE_ERROR_LOGIN_FAILED;
    Serial0.print(F("ERROR=253-LOGIN_FAILED\r\n"));
    Serial.print(F("ERROR=253-LOGIN_FAILED\r\n"));
    return String("null");
  }

 
  String ra_token = extractTokenFromBuffer(response);
  ra_token.trim();
  
  Serial.print(F("RA Token: ")); Serial.println(ra_token);
  
  // print_memory_stats("AFTER TOKEN EXTRACT");
  
  response.clear();
  
  // print_memory_stats("AFTER LOGIN COMPLETE");

  return ra_token;
}

// Versão que extrai token do CharBufferStream sem criar cópia
String extractTokenFromBuffer(CharBufferStream &buf)
{
  const char* key = "\"Token\":";
  int start = buf.indexOf(key);
  if (start == -1) return "";

  // Avança até o início do valor (pula aspas)
  start = buf.indexOf("\"", start + 8);
  if (start == -1) return "";

  int end = buf.indexOf("\"", start + 1);
  if (end == -1) return "";

  // Extrai substring diretamente
  char* data = buf.data();
  int len = end - start - 1;
  if (len <= 0 || len > 64) return "";
  
  char token[65];
  memcpy(token, data + start + 1, len);
  token[len] = '\0';
  
  return String(token);
}


/**
 * functions related to reset the adapter configuration
 */

// handle the reset routine
void handle_reset()
{
  if (ENABLE_RESET == 0)
  {
    return;
  }
  unsigned long start = millis();
  setSemaphore(LED_BLINK_FAST, LED_YELLOW);
  while (digitalRead(RESET_PIN) == LOW && (millis() - start) < 10000)
  {
    yield();
    print_line("Resetting in progress...", 0, 1);
  }
  if ((millis() - start) >= RESET_PRESSED_TIME)
  {
    setSemaphore(LED_BLINK_SLOW, LED_YELLOW);
    Serial.print(F("reset"));
    init_EEPROM(true);
    print_line("Reset successful!", 1, 0);
    print_line("Reboot in 5 seconds...", 2, 0);
    delay(5000);
    ESP.restart();
  }
  setSemaphore(LED_BLINK_MEDIUM, LED_YELLOW);
  print_line("Reset aborted!", 0, 1);
}

/**
 * functions related to the littleFS
 */

// retry download a file up to 3 times with exponential backoff
int try_download_file(String url, String file_name)
{
  int attempt = 0;
  int maxRetries = 3;
  int retryDelayMs = 250;
  while (attempt < maxRetries)
  {
    int ret = download_file_to_littleFS(url, file_name);
    if (ret == 0)
    {
      return 0; // File downloaded successfully
    }
    else
    {
      Serial.print(F("Attempt ")); Serial.print(attempt + 1); Serial.println(F(" failed. Retrying..."));
    }
    attempt++;
    if (attempt <= maxRetries)
    {
      delay(retryDelayMs * pow(2, attempt)); // Exponential backoff
    }
  }
  return -1;
}

// Fetch a file from the URL given and save it in LittleFS
// Return 1 if a web fetch was needed or 0 if file already exists
int download_file_to_littleFS(String url, String file_name)
{
  int ret = 0;
  // If it exists then no need to fetch it
  if (LittleFS.exists(file_name) == true)
  {
    Serial.print("Found " + file_name + " in LittleFS\n"); // debug
    return ret;
  }

  // before use LittleFS, check if we have enough space and delete files if necessary
  check_free_space_littleFS();

  Serial.print("Downloading " + file_name + " from " + url + "\n"); // debug

  // Check WiFi connection
  if ((WiFi.status() == WL_CONNECTED))
  {
    NetworkClientSecure client;
    HTTPClient http;
    client.setInsecure();
    http.begin(client, url);

    // Start connection and send HTTP header
    int http_code = http.GET();
    if (http_code == 200)
    {
      fs::File f = LittleFS.open(file_name, "w+");
      if (!f)
      {
        Serial.print(F("file open failed\n")); // debug
        return -1;
      }

      // File found at server
      if (http_code == HTTP_CODE_OK)
      {

        // Get length of document (is -1 when Server sends no Content-Length header)
        int total = http.getSize();
        int len = total;

        // Create buffer for read
        uint8_t buff[128] = {0};

        // Get tcp stream
        WiFiClient *stream = http.getStreamPtr();

        // Read all data from server
        while (http.connected() && (len > 0 || len == -1))
        {
          // Get available data size
          size_t size = stream->available();

          if (size)
          {
            // Read up to 128 bytes
            int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));

            // Write it to file
            f.write(buff, c);

            // Calculate remaining bytes
            if (len > 0)
            {
              len -= c;
            }
          }
          yield();
        }
      }
      f.close();
    }
    else
    {
      Serial.print(F("download failed, error: ")); Serial.println(http.errorToString(http_code)); // debug
      ret = -1;
    }
    http.end();
  }
  return ret;
}

// auxiliary function to implement startsWith for char*
bool prefix(const char *pre, const char *str)
{
  return strncmp(pre, str, strlen(pre)) == 0;
}

// monitor wifi events - connection
void onWiFiConnect(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println(F("WiFi connected."));
}

// monitor wifi events - disconnection
void onWiFiDisconnect(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println(F("WiFi disconnected."));
  WiFi.reconnect();
}

// monitor wifi events - got IP address
void onWiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.print(F("Got IP: "));
  Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
#ifdef ENABLE_INTERNAL_WEB_APP_SUPPORT
  config_mDNS();
#endif
}

// calculate how much free space we have on littleFS in percentage
float get_free_space_littleFS()
{
  float free_space = (float)LittleFS.totalBytes() - (float)LittleFS.usedBytes();
  free_space = (free_space / (float)LittleFS.totalBytes()) * 100.0;
  // Serial.print("Total Bytes: "); //debug
  // Serial.println(LittleFS.totalBytes()); //debug
  // Serial.print("Used Bytes: "); //debug
  // Serial.println(LittleFS.usedBytes()); //debug
  Serial.print(F("Free space: "));
  Serial.print(free_space);
  Serial.println(F("%"));
  return free_space;
}

// remove all images from littleFS
void remove_all_image_files_littleFS()
{
  fs::File root = LittleFS.open("/");
  fs::File file = root.openNextFile();
  while (file)
  {
    String file_name = file.name();
    file.close();
    if (prefix("achievement_", file_name.c_str()) || prefix("title_", file_name.c_str()))
    {
      Serial.print("Removing " + file_name + "\n"); // debug
      file_name = "/" + file_name;
      LittleFS.remove(file_name);
    }
    file = root.openNextFile();
  }
}

// remove all files if we are running out of space
void check_free_space_littleFS()
{
  float free_space = get_free_space_littleFS();
  if (free_space < 3) // each image uses ~0.5%
  {
    Serial.print(F("Free space is less than 3%, removing all image files\n")); // debug
    remove_all_image_files_littleFS();
  }
}

/**
 * functions related with mDNS, websocket and the experimental internal web app
 */

#ifdef ENABLE_INTERNAL_WEB_APP_SUPPORT

// configure mDNS
void config_mDNS()
{
  String mdnsName = "snes-ra-adapter";

  MDNS.end();
  if (!MDNS.begin(mdnsName.c_str()))
  {
    Serial.println(F("error - begin mDNS"));
    return;
  }

  Serial.print(F("mDNS init: "));
  Serial.println(mdnsName + ".local");

  MDNS.addService("nraa", "tcp", 80);
  Serial.println(F("service mDNS announced"));
}

// handle websocket events
void on_websocket_event(AsyncWebSocket *server, AsyncWebSocketClient *client,
                        AwsEventType type, void *arg, uint8_t *data, size_t len)
{

  if (type == WS_EVT_CONNECT)
  {
    // just one simultaneous connection for the time being
    Serial.println(F("new ws connection"));
    if (ws_client != NULL)
    {
      Serial.println(F("closing last connection"));
      ws_client->close();
    }
    ws_client = client;
    client->setCloseClientOnQueueFull(false);
    client->ping();
  }
  else if (type == WS_EVT_PONG)
  {
    Serial.println(F("ws pong"));
    if (game_id != "0") // we have a game running
    {
      // send the game name and image to the client
      char aux[512];
      sprintf(aux, "G=%s;%s;%s;%s", game_session.c_str(), game_id.c_str(), game_name.c_str(), game_image.c_str());
      client->text(aux);
    }
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    Serial.println(F("ws disconnect"));
    if (ws_client != NULL)
    {
      ws_client->close();
      ws_client = NULL;
    }
  }
  else if (type == WS_EVT_DATA)
  {
    data[len] = 0; 
    if (strcmp((char *)data, "ping") == 0)
    {
      Serial.println(F("ws ping"));
      client->text("pong");
    }
  }
}

// send websocket data
void send_ws_data(String data)
{
  if (ws_client != NULL)
  {
    ws_client->text(data);
  }
}

// initialize the websocket server (dynamically allocated to save memory until needed)
void init_websocket()
{
  if (websocket_initialized) {
    Serial.println(F("WebSocket already initialized"));
    return;
  }
  
  // print_memory_stats("BEFORE WEBSOCKET INIT");
  
  // Instantiate dynamically only now (after patch)
  if (ws == nullptr) {
    ws = new (std::nothrow) AsyncWebSocket("/ws");
    if (ws == nullptr) {
      Serial.println(F("Failed to allocate AsyncWebSocket"));
      return;
    }
  }
  
  if (server == nullptr) {
    server = new (std::nothrow) AsyncWebServer(80);
    if (server == nullptr) {
      Serial.println(F("Failed to allocate AsyncWebServer"));
      delete ws;
      ws = nullptr;
      return;
    }
  }
  
  // ws begin
  ws->onEvent(on_websocket_event);
  
  // Serve static files (HTML, CSS, JS)
  server->serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server->serveStatic("/sw.js", LittleFS, "/sw.js")
      .setCacheControl("no-cache, no-store, must-revalidate");
  server->serveStatic("/index.html", LittleFS, "/index.html")
      .setCacheControl("no-cache, no-store, must-revalidate");

  server->serveStatic("/snd.mp3", LittleFS, "/snd.mp3")
      .setCacheControl("max-age=86400");

  // Seletor de jogo (modo sniff): rota /game no server principal
  game_selector_route(server);

  server->addHandler(ws);
  server->begin();
  
#ifdef ENABLE_LCD
  // Initialize PNG decoder now (after patch, when memory is available)
  if (png == nullptr) {
    png = new (std::nothrow) PNG();
    if (png == nullptr) {
      Serial.println(F("Failed to allocate PNG decoder"));
    }
  }
#endif
  
  // Configure mDNS here, along with the websocket (after patch)
  config_mDNS();
  
  websocket_initialized = true;
  Serial.println(F("WebSocket server initialized"));
  // print_memory_stats("AFTER WEBSOCKET INIT");
}
#endif

/**
 * functions related to the HTTP requests, retry, error handling, etc
 */

// HTTP request result codes to const char* (avoids String allocation)
const char* http_request_result_to_cstr(int code)
{
  switch (code)
  {
  case HTTP_SUCCESS:
    return "HTTP_SUCCESS";
  case HTTP_ERR_NO_WIFI:
    return "HTTP_ERR_NO_WIFI";
  case HTTP_ERR_REQUEST_FAILED:
    return "HTTP_ERR_REQUEST_FAILED";
  case HTTP_ERR_HTTP_4XX:
    return "HTTP_ERR_HTTP_4XX";
  case HTTP_ERR_TIMEOUT:
    return "HTTP_ERR_TIMEOUT";
  case HTTP_ERR_REPONSE_TOO_BIG:
    return "HTTP_ERR_REPONSE_TOO_BIG";
  default:
    return "UNKNOWN_ERROR";
  }
}

// perform an HTTP request writing directly to CharBufferStream with retries and exponential backoff
int perform_http_request_buffer(
    const char* url,
    HttpRequestMethod method,
    const char* payload,
    size_t payload_len,
    CharBufferStream &resp,
    bool isIdempotent,
    int maxRetries,
    int timeoutMs,
    int retryDelayMs)
{
  int attempt = 0;
  int wifiRetries = 3;
  int code = HTTP_ERR_REQUEST_FAILED;

  while (attempt <= maxRetries)
  {
    if (attempt != 0) {
      Serial.print(F("Attempt ")); Serial.print(attempt); Serial.println(F(" to request"));
    }
    
    int wifiAttempt = 0;
    while (WiFi.status() != WL_CONNECTED && wifiAttempt < wifiRetries) {
      delay(500);
      wifiAttempt++;
    }

    if (WiFi.status() != WL_CONNECTED) {
      code = HTTP_ERR_NO_WIFI;
      delay(retryDelayMs * pow(2, attempt));
      attempt++;
      continue;
    }

    
    globalSecureClient.setInsecure();
    globalSecureClient.setTimeout(timeoutMs / 1000 + 30); // +30s extra for chunked encoding
    globalHttpClient.setTimeout(timeoutMs);
    
    Serial.print(F("Connecting to: ")); Serial.println(url);
    Serial.print(F("data: ")); Serial.println(payload);
    
    if (!globalHttpClient.begin(globalSecureClient, url)) {
      Serial.println(F("HTTPClient begin failed"));
      attempt++;
      delay(retryDelayMs * pow(2, attempt));
      continue;
    }
    
    const char user_agent[] = "SNES_RA_ADAPTER/0.1 rcheevos/11.6";
    globalHttpClient.setUserAgent(user_agent);

    Serial.println(F("Sending request..."));
    
    if (method == GET) {
      code = globalHttpClient.GET();
    } else {
      globalHttpClient.addHeader("Content-Type", "application/x-www-form-urlencoded");
      // POST with const char* and explicit length
      code = globalHttpClient.POST((uint8_t*)payload, payload_len);
    }

    Serial.print(F("HTTP code: ")); Serial.println(code);

    if (code > 0)
    {
      if (code >= 200 && code < 300)
      {
        WiFiClient* stream = globalHttpClient.getStreamPtr();
        int contentLength = globalHttpClient.getSize();
        bool isChunked = (contentLength == -1);
        
        Serial.print(F("Content-Length: ")); Serial.print(contentLength); Serial.print(F(" (chunked: ")); Serial.print(isChunked); Serial.println(F(")"));
        
        // Check if it fits in the buffer (if Content-Length is known)
        if (contentLength > 0 && contentLength > (int)resp.capacity()) {
          Serial.print(F("Response too big: ")); Serial.print(contentLength); Serial.print(F(" > ")); Serial.println(resp.capacity());
          globalHttpClient.end();
          return HTTP_ERR_REPONSE_TOO_BIG;
        }
        
        resp.clear();
        // Smaller read buffer to reduce stack usage (256 is enough for chunked)
        uint8_t buff[256];
        size_t totalRead = 0;
        unsigned long lastDataTime = millis();
        unsigned long lastProgressTime = millis();
        
        // For chunked encoding, we need to read the chunk size first
        size_t chunkRemaining = 0;
        bool chunkSizeRead = !isChunked; // If not chunked, no need to read size
        
        // Longer timeout for large payloads (60s total, 20s without progress)
        const unsigned long readTimeoutMs = 20000;
        const unsigned long totalTimeoutMs = 60000;
        unsigned long startTime = millis();
        
        while (globalHttpClient.connected() || stream->available())
        {
          // Timeout total
          if (millis() - startTime > totalTimeoutMs) {
            Serial.println(F("Total timeout exceeded (60s)"));
            break;
          }
          
          // Timeout if no data received for a long time
          if (stream->available() == 0) {
            if (millis() - lastDataTime > readTimeoutMs) {
              Serial.print(F("Read timeout - no data for ")); Serial.print(readTimeoutMs/1000); Serial.println(F("s"));
              break;
            }
            // Check if connection is still active
            if (!globalHttpClient.connected()) {
              Serial.println(F("Connection lost while waiting for data"));
              break;
            }
            delay(10);
            yield();
            continue;
          }
          
          lastDataTime = millis();
          
          if (isChunked && !chunkSizeRead) {
            // Read chunk size with timeout - fixed buffer to avoid fragmentation
            char chunkSizeBuf[16];
            int chunkSizeIdx = 0;
            unsigned long chunkSizeStart = millis();
            while (millis() - chunkSizeStart < 5000 && chunkSizeIdx < 15) {
              if (stream->available()) {
                char c = stream->read();
                if (c == '\n') break;
                if (c != '\r' && c != ' ') {
                  chunkSizeBuf[chunkSizeIdx++] = c;
                }
              } else {
                if (!globalHttpClient.connected()) break;
                delay(1);
                yield();
              }
            }
            chunkSizeBuf[chunkSizeIdx] = '\0';
            
            if (chunkSizeIdx == 0) {
              continue; // Empty line, skip
            }
            chunkRemaining = strtoul(chunkSizeBuf, NULL, 16);
            chunkSizeRead = true;
            
            if (chunkRemaining == 0) {
              // Final chunk (size 0) - end of response
              Serial.println(F("Final chunk received"));
              break;
            }
            
            // Progress log every 5 seconds
            if (millis() - lastProgressTime > 5000) {
              Serial.print(F("Progress: ")); Serial.print(totalRead);
              Serial.print(F(" bytes, chunk: ")); Serial.println(chunkRemaining);
              lastProgressTime = millis();
            }
          }
          
          // Determine how much to read
          size_t toRead = sizeof(buff);
          if (isChunked && chunkRemaining > 0) {
            if (toRead > chunkRemaining) toRead = chunkRemaining;
          } else if (!isChunked && contentLength > 0) {
            size_t remaining = contentLength - totalRead;
            if (toRead > remaining) toRead = remaining;
          }
          
          size_t available = stream->available();
          if (toRead > available) toRead = available;
          if (toRead == 0) continue;
          
          size_t readBytes = stream->readBytes(buff, toRead);
          size_t written = resp.write(buff, readBytes);
          totalRead += written;
          
          if (isChunked) {
            chunkRemaining -= readBytes;
            if (chunkRemaining == 0) {
              // End of chunk, read the \r\n that comes after with timeout
              unsigned long crlfStart = millis();
              while (stream->available() < 2 && globalHttpClient.connected()) {
                if (millis() - crlfStart > 5000) {
                  Serial.println(F("Timeout waiting for chunk CRLF"));
                  break;
                }
                delay(1);
                yield();
              }
              if (stream->available() >= 2) {
                stream->read(); // \r
                stream->read(); // \n
              }
              chunkSizeRead = false; 
            }
          }
          
          if (written < readBytes) {
            Serial.println(F("Buffer full during HTTP read"));
            globalHttpClient.end();
            return HTTP_ERR_REPONSE_TOO_BIG;
          }
          
          // Se não é chunked e já leu tudo
          if (!isChunked && contentLength > 0 && totalRead >= (size_t)contentLength) {
            break;
          }
          
          yield();
        }
        
        Serial.print(F("Total read: ")); Serial.print(totalRead); Serial.println(F(" bytes"));
        globalHttpClient.end();
        
        if (totalRead > 0) {
          return HTTP_SUCCESS;
        }
        code = HTTP_ERR_REQUEST_FAILED;
      }
      else if (code >= 400 && code < 500) {
        globalHttpClient.end();
        return HTTP_ERR_HTTP_4XX;
      }
      else {
        globalHttpClient.end();
        if (!isIdempotent && method == HTTP_POST) {
          return HTTP_ERR_REQUEST_FAILED;
        }
      }
    }
    else
    {
      Serial.print(F("HTTP error code: ")); Serial.println(code);
      if (code == HTTPC_ERROR_CONNECTION_REFUSED ||
          code == HTTPC_ERROR_READ_TIMEOUT ||
          code == HTTPC_ERROR_CONNECTION_LOST) {
        code = HTTP_ERR_TIMEOUT;
      } else {
        code = HTTP_ERR_REQUEST_FAILED;
      }
    }

    globalHttpClient.end();
    attempt++;
    if (attempt <= maxRetries) {
      delay(retryDelayMs * pow(2, attempt));
    }
  }

  return code;
}

// ============================================================================
// Serial Command Handlers - Functions optimized for parsing with char*
// ============================================================================

/**
 * Finds the position of a character in a buffer
 * @param buf Buffer to search
 * @param len Size of the buffer
 * @param ch Character to find
 * @return Position of the character or -1 if not found
 */
int find_char(const char* buf, size_t len, char ch) {
  for (size_t i = 0; i < len; i++) {
    if (buf[i] == ch) return (int)i;
  }
  return -1;
}

/**
 * Finds the position of "\r\n" in a buffer
 * @param buf Buffer to search
 * @param len Size of the buffer
 * @return Position of '\r' or -1 if not found
 */
int find_crlf(const char* buf, size_t len) {
  for (size_t i = 0; i + 1 < len; i++) {
    if (buf[i] == '\r' && buf[i + 1] == '\n') return (int)i;
  }
  return -1;
}

/**
 * Checks if the buffer starts with a prefix
 */
bool starts_with(const char* buf, size_t len, const char* prefix) {
  size_t prefix_len = strlen(prefix);
  if (len < prefix_len) return false;
  return memcmp(buf, prefix, prefix_len) == 0;
}

/**
 * Handler for REQ command - HTTP requests from the Pico
 * @param cmd Pointer to the start of the command (after "REQ=")
 * @param cmd_len Length of the command
 */
void handle_req_command(const char* cmd, size_t cmd_len) {
  // Example: FF;M:POST;U:https://retroachievements.org/dorequest.php;D:r=login2&u=user&p=pass
  
  // Find request_id (up to first ';')
  int pos = find_char(cmd, cmd_len, ';');
  if (pos < 0 || pos > 8) {
    Serial.println(F("REQ: invalid request_id"));
    return;
  }
  
  char request_id[16];
  memcpy(request_id, cmd, pos);
  request_id[pos] = '\0';
  
  cmd += pos + 1;
  cmd_len -= pos + 1;
  
  // Find method (M:POST or M:GET)
  if (cmd_len < 3 || cmd[0] != 'M' || cmd[1] != ':') {
    Serial.println(F("REQ: missing method"));
    return;
  }
  
  pos = find_char(cmd, cmd_len, ';');
  if (pos < 0) {
    Serial.println(F("REQ: invalid method format"));
    return;
  }
  
  // MMethod is between cmd+2 and cmd+pos
  // Simplified: assume POST for Pico requests
  cmd += pos + 1;
  cmd_len -= pos + 1;
  
  // Find URL (U:...)
  if (cmd_len < 3 || cmd[0] != 'U' || cmd[1] != ':') {
    Serial.println(F("REQ: missing URL"));
    return;
  }
  
  pos = find_char(cmd, cmd_len, ';');
  if (pos < 0) {
    Serial.println(F("REQ: invalid URL format"));
    return;
  }
  
  // URL is between cmd+2 and cmd+pos
  char url[256];
  size_t url_len = pos - 2;
  if (url_len >= sizeof(url)) url_len = sizeof(url) - 1;
  memcpy(url, cmd + 2, url_len);
  url[url_len] = '\0';
  
  cmd += pos + 1;
  cmd_len -= pos + 1;
  
  // Find data (D:...)
  if (cmd_len < 3 || cmd[0] != 'D' || cmd[1] != ':') {
    Serial.println(F("REQ: missing data"));
    return;
  }
  
  // Data starts at cmd+2, goes to the end (minus \r\n if present)
  const char* data_ptr = cmd + 2;
  size_t data_len = cmd_len - 2;
  
  // Remove trailing \r\n if present
  while (data_len > 0 && (data_ptr[data_len - 1] == '\r' || data_ptr[data_len - 1] == '\n')) {
    data_len--;
  }
  
  // Use fixed buffer for data (avoids fragmentation)
  char data[512];
  if (data_len >= sizeof(data) - 16) data_len = sizeof(data) - 17; // leave space for "&f=3"
  memcpy(data, data_ptr, data_len);
  data[data_len] = '\0';
  
  bool is_patch_request = (strncmp(data, "r=patch", 7) == 0);
  
  // Final URL (can be lambda) - use direct pointer
  const char* final_url = url;
  if (is_patch_request && ENABLE_SHRINK_LAMBDA == 1) {
    Serial.println(F("ENABLED LAMBDA SHRINK"));
    final_url = SHRINK_LAMBDA_URL;
  }
  
  if (is_patch_request) {
    // print_memory_stats("BEFORE PATCH REQUEST");
    
    // Trim trailing spaces; add &f=3 + orcamento p/ o lambda (Fase A:
    // o Pico 2 aceita 90KB; NES antigos nao mandam &max e ficam nos 32KB)
    while (data_len > 0 && data[data_len - 1] == ' ') data_len--;
    strcpy(data + data_len, "&f=3&max=90000");
    data_len += 14;
    
    
  }
  
  // Prepare memory for large download
  if (is_patch_request) {
    Serial.println(F("Preparing memory for large download..."));
    // Disable WiFi power save during large download (more stable)
    esp_wifi_set_ps(WIFI_PS_NONE);
    // Small delay to allow the system to free resources
    delay(50);
    yield();
  }
  
  response.clear();
  print_memory_stats("BEFORE HTTP REQUEST (REQ handler)");
  
  // Longer timeout for patch requests (30s) as the response can be large (30KB+)
  int request_timeout = is_patch_request ? 30000 : 5000;
  
  // Execute HTTP request using char* version directly
  int ret = perform_http_request_buffer(final_url, POST, data, data_len, response, true, 3, request_timeout, 500);
  
  if (ret < 0) {
    Serial.print(F("ERROR ON RESPONSE: "));
    Serial.println(http_request_result_to_cstr(ret));
    if (ret == HTTP_ERR_REPONSE_TOO_BIG) {
      state = STATE_ERROR_RESPONSE_TOO_BIG;
    } else {
      state = STATE_ERROR_CONNECTIVITY;
    }
  } else {
    // Clean JSON in-place
    if (is_patch_request) {
      Serial.print(F("PATCH LENGTH: "));
      Serial.println(response.length());
      
      clean_json_field_str_value_buffer(response, "Description");
      remove_json_field_buffer(response, "Warning");
      remove_json_field_buffer(response, "BadgeLockedURL");
      remove_json_field_buffer(response, "BadgeURL");
      remove_json_field_buffer(response, "ImageIconURL");
      remove_json_field_buffer(response, "Rarity");
      remove_json_field_buffer(response, "RarityHardcore");
      remove_json_field_buffer(response, "Author");
      remove_json_field_buffer(response, "RichPresencePatch");
      clean_json_field_array_value_buffer(response, "Leaderboards");
      remove_achievements_with_flags_5_buffer(response);
      
      if (response.length() > SERIAL_MAX_PICO_BUFFER) {
        Serial.println(F("removing achievements with MemAddr > 4KB"));
        remove_achievements_with_long_MemAddr_buffer(response, 4096);
      }
      if (response.length() > SERIAL_MAX_PICO_BUFFER) {
        Serial.println(F("removing achievements with MemAddr > 2KB"));
        remove_achievements_with_long_MemAddr_buffer(response, 2048);
      }
      if (response.length() > SERIAL_MAX_PICO_BUFFER) {
        Serial.println(F("removing achievements with MemAddr > 1KB"));
        remove_achievements_with_long_MemAddr_buffer(response, 1024);
      }
      if (response.length() > SERIAL_MAX_PICO_BUFFER) {
        Serial.println(F("removing achievements with MemAddr > 768B"));
        remove_achievements_with_long_MemAddr_buffer(response, 768);
      }
      if (response.length() > SERIAL_MAX_PICO_BUFFER) {
        Serial.println(F("removing achievements with MemAddr > 512B"));
        remove_achievements_with_long_MemAddr_buffer(response, 512);
      }
      
      Serial.println(response.c_str());
      Serial.print(F("NEW PATCH LENGTH: "));
      Serial.println(response.length());
    } else if (strncmp(data, "r=login", 7) == 0) {
      remove_json_field_buffer(response, "AvatarUrl");
    }
  }
  
  if (state >= 198 || response.length() >= SERIAL_MAX_PICO_BUFFER) {
    // Falha (resposta grande demais ou erro de conectividade): responde 500
    // vazio ao Pico para o rcheevos falhar rapido em vez de esperar timeout.
    Serial0.print(F("RESP="));
    Serial0.print(request_id);
    Serial0.print(F(";1F4;\r\n"));
    Serial.print(F("RESP too big/error -> 500 enviado ao Pico, len="));
    Serial.println(response.length());
  }
  if (state < 198 && response.length() < SERIAL_MAX_PICO_BUFFER) {
    Serial0.print(F("RESP="));
    Serial0.print(request_id);
    Serial0.print(F(";200;"));
    
    const char *ptr = response.c_str();
    uint32_t len = response.length();
    uint32_t offset = 0;
    
    while (offset < len) {
      uint32_t chunk_len = min((uint32_t)SERIAL_COMM_CHUNK_SIZE, (uint32_t)(len - offset));
      Serial0.write((const uint8_t *)&ptr[offset], chunk_len);
      Serial0.flush();
      offset += chunk_len;
      delay(SERIAL_COMM_TX_DELAY_MS);
    }
    Serial0.print(F("\r\n"));
    
    Serial.print(F("RESP="));
    Serial.print(request_id);
    Serial.println(F(";"));
  }
  
  response.clear();
  
  // After patch, release large buffer and reallocate small buffer
  if (is_patch_request) {
    Serial.println(F("Releasing large buffer and restoring small buffer..."));
    response.release();
    if (response.reserve(SMALL_BUFFER_SIZE)) {
      Serial.print(F("Buffer restored to ")); Serial.print(response.capacity()); Serial.println(F(" bytes"));
    }
    // print_memory_stats("AFTER SHRINK");
    // Restore WiFi power save
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  }
}


/**
 * Handler for CART_INFO command - SNES cartridge metadata sent by Pico.
 *
 * Format from Pico:
 *   CART_INFO=<title>;<rom_kb>;<sram_kb>;<region_hex>;<map_hex>;<type_hex>;<cs_hex>;<md5_hex>
 *
 * Example:
 *   CART_INFO=SUPER MARIOWORLD;512;2;01;20;02;A0DA;cdd3c8c37322978ca8669b34bc89c804
 *
 * Field positions (0-indexed by ';'):
 *   0: title           -- ASCII, trimmed by Pico
 *   1: rom_kb          -- decimal, ROM size in KB
 *   2: sram_kb         -- decimal, SRAM size in KB (0 if absent)
 *   3: region          -- 2 hex digits (header byte $D9)
 *   4: map_mode        -- 2 hex digits (header byte $D5; bit 0 = HiROM)
 *   5: rom_type        -- 2 hex digits (header byte $D6; chip/RAM flags)
 *   6: checksum        -- 4 hex digits (header word; just for logging)
 *   7: md5             -- 32 hex digits (full ROM MD5, lowercase)
 *
 * Unlike the NES (which needed a CRC -> MD5 lookup), we receive the MD5
 * straight from the Pico, so we just stash it and advance state.
 *
 * @param cmd Pointer to data after "CART_INFO="
 * @param cmd_len Length of data (\r\n already stripped by dispatcher)
 */
void handle_cart_info_command(const char* cmd, size_t cmd_len) {
  if (state != STATE_WAITING_CART_INFO) {
    Serial0.print(F("COMMAND_IGNORED_WRONG_STATE\r\n"));
    Serial.print(F("COMMAND_IGNORED_WRONG_STATE\r\n"));
    return;
  }

  // Trim trailing whitespace defensively.
  while (cmd_len > 0 && (cmd[cmd_len - 1] == '\r' ||
                         cmd[cmd_len - 1] == '\n' ||
                         cmd[cmd_len - 1] == ' ')) {
    cmd_len--;
  }

  Serial.print(F("CART_INFO="));
  Serial.write(cmd, cmd_len);
  Serial.println();

  // Walk the 7 ';' separators; we only really need the MD5 at the end, but
  // capturing the title is useful for the TFT.
  const char* fields[8] = {0};
  size_t      lens[8]   = {0};
  size_t      field_idx = 0;
  const char* start     = cmd;

  for (size_t i = 0; i <= cmd_len && field_idx < 8; i++) {
    if (i == cmd_len || cmd[i] == ';') {
      fields[field_idx] = start;
      lens[field_idx]   = (cmd + i) - start;
      field_idx++;
      start = cmd + i + 1;
    }
  }

  if (field_idx < 8 || lens[7] != 32) {
    Serial.println(F("CART_INFO: malformed payload (missing fields or bad MD5 length)"));
    state = STATE_ERROR_CARTRIDGE_NOT_FOUND;
    return;
  }

  // Store MD5.
  memcpy(md5_global, fields[7], 32);
  md5_global[32] = '\0';

  // Optional: log the parsed bits for diagnostics.
  Serial.print(F("CART title=")); Serial.write(fields[0], lens[0]); Serial.println();
  Serial.print(F("CART rom_kb="));  Serial.write(fields[1], lens[1]); Serial.println();
  Serial.print(F("CART sram_kb=")); Serial.write(fields[2], lens[2]); Serial.println();
  Serial.print(F("CART region=0x")); Serial.write(fields[3], lens[3]); Serial.println();
  Serial.print(F("CART map_mode=0x")); Serial.write(fields[4], lens[4]); Serial.println();
  Serial.print(F("CART rom_type=0x")); Serial.write(fields[5], lens[5]); Serial.println();
  Serial.print(F("CART checksum=0x")); Serial.write(fields[6], lens[6]); Serial.println();
  Serial.print(F("CART MD5="));    Serial.println(md5_global);

  state = STATE_CART_INFO_RECEIVED;
}

/**
 * Handler for A= command - Achievement unlocked
 * @param cmd Pointer to data after "A="
 * @param cmd_len Length of data
 */
void handle_achievement_command(const char* cmd, size_t cmd_len) {
  // Format: 123456;Cruise Control;https://media.retroachievements.org/Badge/348421.png
  Serial.print(F("A="));
  Serial.write(cmd, cmd_len);
  Serial.println();
  
  // Find first ';' (end of ID)
  int pos1 = find_char(cmd, cmd_len, ';');
  if (pos1 < 0) {
    Serial.println(F("A: invalid format - no id separator"));
    return;
  }
  
  // Extract ID
  char id_str[16];
  size_t id_len = pos1;
  if (id_len >= sizeof(id_str)) id_len = sizeof(id_str) - 1;
  memcpy(id_str, cmd, id_len);
  id_str[id_len] = '\0';
  
  // Advance after first ';'
  const char* remaining = cmd + pos1 + 1;
  size_t remaining_len = cmd_len - pos1 - 1;
  
  // Find second ';' (end of title)
  int pos2 = find_char(remaining, remaining_len, ';');
  if (pos2 < 0) {
    Serial.println(F("A: invalid format - no title separator"));
    return;
  }
  
  // Extract title
  char title_str[128];
  size_t title_len = pos2;
  if (title_len >= sizeof(title_str)) title_len = sizeof(title_str) - 1;
  memcpy(title_str, remaining, title_len);
  title_str[title_len] = '\0';
  
  // URL is the rest
  const char* url_ptr = remaining + pos2 + 1;
  size_t url_len = remaining_len - pos2 - 1;
  
  // Remove trailing whitespace
  while (url_len > 0 && (url_ptr[url_len - 1] == '\r' || url_ptr[url_len - 1] == '\n' || url_ptr[url_len - 1] == ' ')) {
    url_len--;
  }
  
  char url_str[256];
  if (url_len >= sizeof(url_str)) url_len = sizeof(url_str) - 1;
  memcpy(url_str, url_ptr, url_len);
  url_str[url_len] = '\0';
  
  // Create achievement struct
  achievements_t achievement;
  achievement.id = atoi(id_str);
  achievement.title = String(title_str);
  achievement.url = String(url_str);
  
  // Increment unlocked counter
  unlocked_achievements++;
  
  if (fifo_enqueue(&achievements_fifo, achievement) == false) {
    Serial.print(F("FIFO_FULL\r\n"));
  } else {
    Serial.print(F("ACHIEVEMENT_ADDED\r\n"));
  }
}

/**
 * Handler for GAME_INFO= command - Game information
 * @param cmd Pointer to data after "GAME_INFO="
 * @param cmd_len Length of data
 */
void handle_game_info_command(const char* cmd, size_t cmd_len) {
  // Format: 1496;R.C. Pro-Am;https://media.retroachievements.org/Images/052570.png
  
  // Find first ';' (end of game_id)
  int pos1 = find_char(cmd, cmd_len, ';');
  if (pos1 < 0) {
    Serial.println(F("GAME_INFO: invalid format"));
    return;
  }
  
  // Extract game_id
  char id_str[16];
  size_t id_len = pos1;
  if (id_len >= sizeof(id_str)) id_len = sizeof(id_str) - 1;
  memcpy(id_str, cmd, id_len);
  id_str[id_len] = '\0';
  game_id = String(id_str);
  
  // Reset achievement counters for new game
  total_achievements = 0;
  unlocked_achievements = 0;
  
  // Advance
  const char* remaining = cmd + pos1 + 1;
  size_t remaining_len = cmd_len - pos1 - 1;
  
  // Find second ';' (end of name)
  int pos2 = find_char(remaining, remaining_len, ';');
  if (pos2 < 0) {
    Serial.println(F("GAME_INFO: invalid format - no name separator"));
    return;
  }
  
  // Extrair game_name
  char name_str[128];
  size_t name_len = pos2;
  if (name_len >= sizeof(name_str)) name_len = sizeof(name_str) - 1;
  memcpy(name_str, remaining, name_len);
  name_str[name_len] = '\0';
  game_name = String(name_str);
  
  // URL is the rest
  const char* url_ptr = remaining + pos2 + 1;
  size_t url_len = remaining_len - pos2 - 1;
  
  // Remove trailing whitespace
  while (url_len > 0 && (url_ptr[url_len - 1] == '\r' || url_ptr[url_len - 1] == '\n' || url_ptr[url_len - 1] == ' ')) {
    url_len--;
  }
  
  char url_str[256];
  if (url_len >= sizeof(url_str)) url_len = sizeof(url_str) - 1;
  memcpy(url_str, url_ptr, url_len);
  url_str[url_len] = '\0';
  game_image = String(url_str);
  
  if (game_id == "0") {
    state = STATE_ERROR_CARTRIDGE_NOT_FOUND;
    return;
  }
  
  setSemaphore(LED_ON, LED_GREEN);
  
  char file_name[64];
  sprintf(file_name, "/title_%s.png", game_id.c_str());
  try_download_file(game_image, file_name);
  
  // Extract file name from URL
  int last_slash = game_image.lastIndexOf("/");
  if (last_slash >= 0) {
    game_image = game_image.substring(last_slash + 1);
  }
  
  // Truncate game name if too long
  String esp_game_name = game_name;
  if (esp_game_name.length() > 18) {
    esp_game_name = esp_game_name.substring(0, 15) + "...";
  }
  
  char game_display[32];
  snprintf(game_display, sizeof(game_display), " * %s * ", esp_game_name.c_str());
  
  print_line("Cartridge identified:", 1, 0);
  print_line(game_display, 2, 0);
  print_line("please RESET the console", 4, 0);
  Serial.println(game_name);
  
  // SNES: bus is enabled by the Pico (release_snes() in pico firmware).
  // The ESP32 has nothing to drive here.
}

/**
 * Handler for ACH_SUMMARY= command - Summary of achievements
 * @param cmd Pointer to data after "ACH_SUMMARY="
 * @param cmd_len Length of data
 * Format: unlocked;total
 */
void handle_ach_summary_command(const char* cmd, size_t cmd_len) {
  // Find ';' separator
  int pos = find_char(cmd, cmd_len, ';');
  if (pos < 0) {
    Serial.println(F("ACH_SUMMARY: invalid format"));
    return;
  }
  
  // Extract unlocked
  char unlocked_str[16];
  size_t unlocked_len = pos;
  if (unlocked_len >= sizeof(unlocked_str)) unlocked_len = sizeof(unlocked_str) - 1;
  memcpy(unlocked_str, cmd, unlocked_len);
  unlocked_str[unlocked_len] = '\0';
  unlocked_achievements = (uint16_t)atoi(unlocked_str);
  
  // Extract total
  const char* total_ptr = cmd + pos + 1;
  size_t total_len = cmd_len - pos - 1;
  
  // Remove trailing whitespace
  while (total_len > 0 && (total_ptr[total_len - 1] == '\r' || total_ptr[total_len - 1] == '\n' || total_ptr[total_len - 1] == ' ')) {
    total_len--;
  }
  
  char total_str[16];
  if (total_len >= sizeof(total_str)) total_len = sizeof(total_str) - 1;
  memcpy(total_str, total_ptr, total_len);
  total_str[total_len] = '\0';
  total_achievements = (uint16_t)atoi(total_str);
  
  Serial.print(F("ACH_SUMMARY: "));
  Serial.print(unlocked_achievements);
  Serial.print(F("/"));
  Serial.println(total_achievements);
}

/**
 * Handler for SNES_RESETED command - user reset/powered-on the SNES and the
 * Pico now sees the SNES driving the bus. Refreshes the title screen UI.
 */
void handle_console_reset_command() {
  uint8_t random = (uint8_t)esp_random();
  game_session = String(random);
  
#ifdef ENABLE_INTERNAL_WEB_APP_SUPPORT
  init_websocket();
  char aux[512];
  send_ws_data("RESET");
  sprintf(aux, "G=%s;%s;%s;%s", game_session.c_str(), game_id.c_str(), game_name.c_str(), game_image.c_str());
  if (ws_client) {
    ws_client->text(aux);
  }
#endif
  
  show_title_screen();
  state = STATE_IDLE;
}

// Sync with Pico - sends SYNC command and waits for SYNC_ACK or PICO_READY
// Returns true if sync successful, false if failed after maxRetries
bool syncWithPico(int maxRetries = 3) {
  print_line("Syncing with Pico...", 0, 1);
  
  for (int i = 0; i < maxRetries; i++) {
    // Clear any garbage in receive buffer
    while (Serial0.available() > 0) Serial0.read();
    
    Serial0.print(F("SYNC\r\n"));
    Serial0.flush();
    
    unsigned long start = millis();
    while (millis() - start < 1000) { // 1 second timeout
      if (Serial0.available() > 0) {
        String response = Serial0.readStringUntil('\n');
        response.trim();
        if (response.startsWith("SYNC_ACK") || response.startsWith("PICO_READY")) {
          Serial.println(F("Pico sync OK"));
          return true;
        }
      }
      delay(10);
    }
    
    // Timeout - try reset
    Serial.print(F("Pico sync timeout, attempt "));
    Serial.println(i + 1);
    Serial0.print(F("RESET\r\n"));
    Serial0.flush();
    delay(500); // Wait for Pico to reboot
  }
  
  Serial.println(F("Failed to sync with Pico after all retries"));
  return false;
}

// arduino-esp32 entry point
void setup()
{
 
  setCpuFrequencyMhz(80);
  // disable i2c
  Wire.end();
  // BLEDevice::deinit(true);

  // SNES: bus isolation is handled by the Pico (BUFFER_OE_N via 10k pull-up
  // and GP45). No ESP32-side action needed here.

  //config lcd brightness
  pinMode(LCD_BRIGHTNESS_PIN, OUTPUT);
  analogWrite(LCD_BRIGHTNESS_PIN, 192); // set the brightness of the TFT screen

  // config the led status pin
  pinMode(LED_STATUS_GREEN_PIN, OUTPUT);
  pinMode(LED_STATUS_RED_PIN, OUTPUT);

  setSemaphore(LED_BLINK_MEDIUM, LED_YELLOW);

  Serial.begin(9600);    // initialize the serial port
  Serial0.setTimeout(2); // 2ms timeout for Serial0
  Serial0.begin(115200);
  // consume any garbage in the serial buffer before communicating with the pico
  if (Serial0.available() > 0)
  {
    Serial0.readString();
    delay(10);
  }

  delay(1000);

#ifdef ENABLE_LCD
  // Aloca o decodificador PNG (~48KB contiguos do PNGdec) AGORA, com a
  // heap fresca de boot -- antes do buffer grande de resposta. Alocar
  // tarde (em init_websocket, pos-patch) falhava por fragmentacao: o
  // new(nothrow) retornava nullptr e as imagens sumiam em silencio
  // (era tambem a causa do abort/reboot antes do nothrow).
  if (png == nullptr) {
    png = new (std::nothrow) PNG();
    if (png == nullptr) {
      Serial.println(F("PNG decoder alloc FAILED at boot!"));
    } else {
      Serial.println(F("PNG decoder allocated at boot"));
    }
  }
#endif

  // print_memory_stats("BEFORE LARGE BUFFER ALLOCATION");

  if (response.reserve(LARGE_BUFFER_SIZE)) {
    Serial.print(F("Large response buffer allocated: ")); 
    Serial.print(LARGE_BUFFER_SIZE); 
    Serial.println(F(" bytes"));
  } else {
    // Tentar tamanhos menores se 90KB falhar
    size_t sizes[] = {85000, 80000, 75000, 70000, 65000};
    bool allocated = false;
    for (int i = 0; i < 5 && !allocated; i++) {
      if (response.reserve(sizes[i])) {
        Serial.print(F("Response buffer allocated: ")); 
        Serial.print(sizes[i]); 
        Serial.println(F(" bytes"));
        allocated = true;
      }
    }
    if (!allocated) {
      Serial.println(F("FATAL: Could not allocate response buffer!"));
    }
  }
  
  // print_memory_stats("AFTER LARGE BUFFER ALLOCATION");

  // config reset pin
  pinMode(RESET_PIN, INPUT);

  // Inicializar buffer serial fixo
  serial_buffer_len = 0;
  serial_buffer[0] = '\0';

  // initialize global strings
  game_id = "0";
  game_name = "not identified";
  // base_url agora é const char* global, não precisa inicializar aqui

  // initialize stuff
  init_EEPROM(false); // initialize the EEPROM

  fifo_init(&achievements_fifo); // initialize the achievement fifo

  

  // delay(250);
  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) // Initialise LittleFS
  {
    Serial.print(F("LittleFS initialisation failed!")); // debug
    setSemaphore(LED_BLINK_FAST, LED_RED);
    while (1)
      yield(); // Stay here twiddling thumbs waiting
  }

  char esp_version[32];
  sprintf(esp_version, "ESP32_VERSION=%s\r\n", VERSION);
  Serial0.print(esp_version); // send the version to the pico - debug

#ifdef ENABLE_LCD
  initLCD ();
#endif

  // Sync with Pico - ensures both devices are in sync after ESP32 reset
  if (!syncWithPico()) {
    print_line("Pico sync failed!", 0, 2);
    print_line("Check connection", 1, 2);
    print_line("Power cycle needed", 2, 2);
    play_error_sound();
    setSemaphore(LED_BLINK_FAST, LED_RED);
    // Halt execution - user must power cycle
    while (true) {
      delay(1000);
    }
  }

  // check if the reset button is pressed and handle the reset routine
  handle_reset();


  // Pré-inicializar o cliente SSL global (configura os buffers SSL)
  globalSecureClient.setInsecure();
  globalSecureClient.setTimeout(15);
  httpClientInitialized = true;
  
  Serial.println(F("Global HTTP client initialized"));
  // print_memory_stats("AFTER SSL CLIENT INIT");

  // check if the EEPROM is configured. If not, start the configuration portal
  int wifi_configuration_tries = 0;
  bool wifi_configured = false;
  bool configured = is_eeprom_configured();
  if (!configured)
  {
    response.reserve(SMALL_BUFFER_SIZE);
    WiFiManager wm;
    WiFiManagerParameter custom_p1(html_p1);
    WiFiManagerParameter custom_p2(html_p2);
    WiFiManagerParameter custom_param_1("un", NULL, "", 20, " required autocomplete='off'");
    WiFiManagerParameter custom_p3(html_p3);
    WiFiManagerParameter custom_param_2("up", NULL, "", 255, " type='password' required");
    WiFiManagerParameter custom_s(html_s);
    wifi_manager_init(wm); // initialize the wifi manager
    wm.addParameter(&custom_p1);
    wm.addParameter(&custom_p2);
    wm.addParameter(&custom_param_1);
    wm.addParameter(&custom_p3);
    wm.addParameter(&custom_param_2);
    wm.addParameter(&custom_s);
    while (!configured)
    {
      if (wifi_configuration_tries == 0 && !wifi_configured)
      {
        setSemaphore(LED_BLINK_MEDIUM, LED_YELLOW); 
        print_line(" Configure the Adapter:", 0, 1);
        print_line("Connect to its wifi network", 1, -1, -16);
        print_line("named \"SNES_RA_ADAPTER\",", 2, -1, -16);
        print_line("password: 12345678, and ", 3, -1, -16);
        print_line("then open http://192.168.1.1", 4, -1, -16);
        play_attention_sound();
      }
      else if (wifi_configuration_tries > 0 && !wifi_configured)
      {
        setSemaphore(LED_BLINK_SLOW, LED_RED);
        print_line("Could not connect to wifi", 0, 2);
        print_line("network!", 1, -1, 16);
        print_line("check it and try again", 3, -1, 16);
        play_error_sound();
      }
      else if (wifi_configured)
      {
        setSemaphore(LED_BLINK_MEDIUM, LED_RED);
        print_line("Could not log in", 0, 2);
        print_line("RetroAchievements!", 1, -1, 16);
        print_line("check the credentials", 3, -1, 16);
        print_line("and try again", 4, -1, 16);
        play_error_sound();
      }
      if (wm.startConfigPortal("SNES_RA_ADAPTER", "12345678"))
      {
        setSemaphore(LED_BLINK_SLOW, LED_GREEN);
        Serial.print(F("connected...yeey :)"));
        wifi_configured = true;
        clean_screen_text();
        print_line("Wifi OK!", 0, 0);
        String temp_ra_user = custom_param_1.getValue();
        String temp_ra_pass = custom_param_2.getValue();
        ra_user_token = try_login_RA(temp_ra_user, temp_ra_pass);
        Serial.print(ra_user_token);
        if (ra_user_token.compareTo("") != 0)
        {
          setSemaphore(LED_BLINK_MEDIUM, LED_GREEN);
          print_line("Logged in RA!", 1, 0);
          play_success_sound();
          Serial.println(F("saving configuration info in eeprom"));
          save_configuration_info_eeprom(temp_ra_user, temp_ra_pass);
          // Reset Pico before restarting ESP32 to keep them in sync
          Serial0.print(F("RESET\r\n"));
          Serial0.flush();
          delay(500);
          ESP.restart(); // just to play safe as we are using all the RAM we can get to receive the achievement list
        }
        else
        {
          setSemaphore(LED_BLINK_FAST, LED_RED);
          Serial.print(F("could not log in RA"));
          clean_screen_text();
        }
      }
      else
      {
        setSemaphore(LED_BLINK_MEDIUM, LED_RED);
        Serial.print(F("not connected...booo :("));
        clean_screen_text();
        wifi_configuration_tries += 1;
      }
      configured = is_eeprom_configured();
    }
  }
  else
  {
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_15dBm);
    print_line("Connecting to Wifi...", 0, 1);
    if (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin();
      while (WiFi.status() != WL_CONNECTED)
        delay(500);
    }
    setSemaphore(LED_BLINK_SLOW, LED_GREEN);
    print_line("Wifi OK!", 0, 0); // TODO: implement timeout
    
    // Usar buffers fixos para credenciais
    char ra_user[64];
    char ra_pass[128];
    read_ra_user_from_eeprom(ra_user, sizeof(ra_user));
    read_ra_pass_from_eeprom(ra_pass, sizeof(ra_pass));
    
    print_line("Logging in RA...", 1, 1);
    ra_user_token = try_login_RA(String(ra_user), String(ra_pass));
    if (ra_user_token.compareTo("") != 0)
    {
      setSemaphore(LED_BLINK_MEDIUM, LED_GREEN);
      print_line("Logged in RA!", 1, 0);
      play_success_sound();
      // print_memory_stats("AFTER LOGIN SUCCESS");
    }
    else
    {
      setSemaphore(LED_BLINK_FAST, LED_RED);
      print_line("Wrong credential or RA offline", 2, -1, -22);
      print_line("Consider reset the adapter", 3, -1, -22 );
      state = STATE_ERROR_LOGIN_FAILED;      
      return;
    }
  }

#ifdef ENABLE_INTERNAL_WEB_APP_SUPPORT
  config_mDNS();
#endif
  // handle reconnects
  WiFi.onEvent(onWiFiConnect, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(onWiFiDisconnect, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.onEvent(onWiFiGotIP, ARDUINO_EVENT_WIFI_STA_GOT_IP);

  // print_memory_stats("BEFORE SENDING TOKEN TO PICO");

  // send the token and user to the pico (needed)
  // Ler user para buffer fixo antes de liberar EEPROM
  char ra_user_for_pico[64];
  read_ra_user_from_eeprom(ra_user_for_pico, sizeof(ra_user_for_pico));
  
  char token_and_user[128];
  sprintf(token_and_user, "TOKEN_AND_USER=%s,%s\r\n", ra_user_token.c_str(), ra_user_for_pico);

  delay(250);                   // make sure pico restarted
  Serial.print(token_and_user); // debug
  Serial0.print(token_and_user);  
  state = STATE_IDENTIFY_CARTRIDGE;

  // modem sleep
  WiFi.setSleep(true);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  // esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

  // Liberar buffer da EEPROM para economizar RAM (~256 bytes)
  // Se precisar salvar novamente, chame EEPROM.begin() antes
  EEPROM.end();

  // print_memory_stats("END OF SETUP");

  Serial.print(F("setup end")); // debug
}

// arduino-esp32 main loop
void loop()
{
  maybe_start_game_selector();

  // Troca de jogo via /game: o Pico ja recebeu GAME=<n> e esta rebootando
  // com o indice novo; reinicia o ESP32 tambem para ressincronizar do zero.
  if (game_switch_restart_at != 0 && millis() > game_switch_restart_at)
  {
    ESP.restart();
  }

#ifdef ENABLE_INTERNAL_WEB_APP_SUPPORT
  if (websocket_initialized && ws != nullptr) {
    uint32_t delta_ws_cleanup = millis() - last_ws_cleanup;
    last_ws_cleanup = millis();
    if (delta_ws_cleanup > 1000)
    {
      ws->cleanupClients();
    }
  }

  // #TODO last ping was too long ago, closes the connection
#endif

  // Update WiFi status every 10 seconds after title screen was shown
  if (already_showed_title_screen && (millis() - last_wifi_status_update > 10000))
  {
    last_wifi_status_update = millis();
    showWifiStatus();
  }

  long go_back_to_title_delta_timestamp = millis() - go_back_to_title_screen_timestamp;
  // check if we can go back to the title screen after showing the achievement for 15 seconds
  if (go_back_to_title_screen == true && go_back_to_title_delta_timestamp > 15000)
  {
    go_back_to_title_screen = false;
    if (fifo_is_empty(&achievements_fifo))
    {
      
      show_title_screen();
    }
  }

  // handle errors during the cartridge identification - unified error handling
  if (isErrorState(state))
  {
    setSemaphore(LED_BLINK_FAST, LED_RED);
    const char* errorMsg = getStateErrorMessage(state);
    print_line(errorMsg, 1, 2);
    
    if (state == STATE_ERROR_CONNECTIVITY) {
      print_line("It's not recording achievs", 2, 2);
    }
    
    print_line("Turn off the console", 4, 2);
    play_error_sound();
    state = STATE_IDLE; // do nothing - it will not enable the BUS, so the game will not boot
  }

  // handle the cartridge identification
  if (state == STATE_IDENTIFY_CARTRIDGE)
  {
    setSemaphore(LED_BLINK_FAST, LED_GREEN);
    print_line("Identifying cartridge...", 1, 1);

    // SNES: ask the Pico to identify the cartridge. Pico drives the bus,
    // parses the SNES header, computes the ROM MD5, and replies with one
    // CART_INFO message containing everything we need.
    Serial0.print(F("READ_CART\r\n"));
    Serial.print(F("READ_CART\r\n"));
    delay(250);
    state = STATE_WAITING_CART_INFO;

    // USEFUL FOR TESTING - FORCING A KNOWN GAME (Super Mario World demo MD5)
    // strcpy(md5_global, "cdd3c8c37322978ca8669b34bc89c804");
    // state = STATE_CART_INFO_RECEIVED;
  }

  // inform the pico to start the process to watch the BUS for memory writes
  if (state == STATE_CART_INFO_RECEIVED)
  {
    Serial0.print(F("START_WATCH\r\n"));
    Serial.print(F("START_WATCH\r\n"));
    state = STATE_WATCHING;
  }

  // if there is some achievement to be shown, show it
  if (fifo_is_empty(&achievements_fifo) == false && Serial.available() == 0 && go_back_to_title_screen == false) // show achievements
  {
    achievements_t achievement;
    fifo_dequeue(&achievements_fifo, &achievement);
    show_achievement(achievement);
  }

  // handle the serial communication with the pico - usando buffer fixo
  while (Serial0.available() > 0)
  {
    // Read only up to available space
    size_t available_space = SERIAL_BUFFER_SIZE - serial_buffer_len - 1;
    if (available_space > 0) {
      size_t bytes_read = Serial0.readBytes(serial_buffer + serial_buffer_len, available_space);
      serial_buffer_len += bytes_read;
      serial_buffer[serial_buffer_len] = '\0'; // null-terminate
    }
    
    // Check for overflow
    if (serial_buffer_len >= SERIAL_BUFFER_SIZE - 1)
    {
      serial_buffer_len = 0;
      serial_buffer[0] = '\0';
      Serial0.print(F("BUFFER_OVERFLOW\r\n"));
      Serial.print(F("BUFFER_OVERFLOW\r\n"));
      continue;
    }

    // Process complete commands (terminated by \r\n)
    int crlf_pos;
    while ((crlf_pos = find_crlf(serial_buffer, serial_buffer_len)) >= 0)
    {
      // We have a complete command
      size_t cmd_len = crlf_pos; // does not include \r\n
      
      // Dispatch to the appropriate handler
      if (starts_with(serial_buffer, cmd_len, "REQ=")) {
        handle_req_command(serial_buffer + 4, cmd_len - 4);
      }
      else if (starts_with(serial_buffer, cmd_len, "CART_INFO=")) {
        handle_cart_info_command(serial_buffer + 10, cmd_len - 10);
      }
      else if (starts_with(serial_buffer, cmd_len, "A=")) {
        handle_achievement_command(serial_buffer + 2, cmd_len - 2);
      }
      else if (starts_with(serial_buffer, cmd_len, "GAME_INFO=")) {
        handle_game_info_command(serial_buffer + 10, cmd_len - 10);
      }
      else if (starts_with(serial_buffer, cmd_len, "ACH_SUMMARY=")) {
        handle_ach_summary_command(serial_buffer + 12, cmd_len - 12);
      }
      else if (starts_with(serial_buffer, cmd_len, "SNES_RESETED")) {
        handle_console_reset_command();
      }
      else if (starts_with(serial_buffer, cmd_len, "C=")) {
#ifdef ENABLE_INTERNAL_WEB_APP_SUPPORT
        // Create temporary String only for websocket
        char temp[256];
        size_t temp_len = min(cmd_len, sizeof(temp) - 1);
        memcpy(temp, serial_buffer, temp_len);
        temp[temp_len] = '\0';
        send_ws_data(String(temp));
#endif
      }
      else if (starts_with(serial_buffer, cmd_len, "P=")) {
#ifdef ENABLE_INTERNAL_WEB_APP_SUPPORT
        char temp[256];
        size_t temp_len = min(cmd_len, sizeof(temp) - 1);
        memcpy(temp, serial_buffer, temp_len);
        temp[temp_len] = '\0';
        send_ws_data(String(temp));
#endif
      }
      else {
        Serial.print(F("UNKNOWN="));
        Serial.write(serial_buffer, cmd_len);
        Serial.println();
      }
      
      // Remove processed command from buffer (including \r\n)      
      size_t remove_len = crlf_pos + 2;
      if (remove_len < serial_buffer_len) {
        memmove(serial_buffer, serial_buffer + remove_len, serial_buffer_len - remove_len);
        serial_buffer_len -= remove_len;
      } else {
        serial_buffer_len = 0;
      }
      serial_buffer[serial_buffer_len] = '\0';
    }
  }
  yield();
}