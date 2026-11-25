# snes-esp-firmware

Fork of `nes-esp-firmware` adapted for the Super Nintendo adapter. Builds for the
**ESP32-C3 Super Mini** (prototype v0.1) or the **Waveshare ESP32-S3-Zero** (next revisions, still being validated),
see [Board revisions](#board-revisions). Responsibilities:

- Provides Internet connectivity to the SNES adapter.
- Manages a TFT screen to display unlocked achievements.
- Controls a buzzer for sound effects.
- Stores Wi-Fi and RetroAchievements (RA) credentials in EEPROM.
- Allows configuration via smartphone, connecting to the ESP32's access point
  (`SNES_RA_ADAPTER`).
- Supports configuration reset by holding the reset button for 5 seconds
  during startup.
- Uses LittleFS to cache game/achievement images downloaded from
  RetroAchievements.
- Communicates with a **Raspberry Pi Pico 2 (RP2350B)** via Serial0 to:
  - Trigger cartridge identification (`READ_CART`).
  - Receive parsed header + ROM MD5 in a single `CART_INFO=...` message.
  - Send HTTP requests to RetroAchievements via `REQ=` / `RESP=` envelope.

## Web portal

With `ENABLE_INTERNAL_WEB_APP_SUPPORT` (on by default) the adapter serves a dashboard on the LAN at
**http://snes-ra-adapter.local/** (or its IP, printed in the serial log). It is a port of the Wii RA
Adapter dashboard:

- game header (box art, RA link, hardcore/casual, rich presence, progress bar, points, "beaten");
- the full achievement list with filters (unlocked, beaten, challenges, in progress, locked) — badges,
  descriptions, points, progression / win condition / missable tags and unlock rate;
- live updates over the WebSocket `/ws`: unlock toast with sound and vibration, challenge and progress
  indicators, rich presence;
- **theme switch** (🎨 in the header): the portal uses the colors of the LCD themes (`LcdTheme` in the
  sketch, served by `/api/themes`, so a new theme shows up in both places). Picking one redraws the LCD
  and saves it in the last EEPROM byte; a configuration reset (reset button) goes back to
  `LCD_DEFAULT_THEME`;
- **hidden debug view**: tap the game image 10 times. Shows reset reason, boot counter, heap, Wi-Fi,
  RA request errors, etc

The page lives in LittleFS. Edit the readable source **`web/index.html`**; what goes to LittleFS is the
minified + gzipped build `data/index.html.gz` (~10 KB instead of ~37 KB; ESPAsyncWebServer serves it with
`Content-Encoding: gzip`). Rebuild it after changing the page, then re-upload the `data` folder:

```
cd snes-esp-firmware/web
npm install        # once (html-minifier-terser, pinned in package.json)
npm run build      # web/index.html -> data/index.html.gz
```

`data/` must not contain an `index.html` (it would be served instead of the `.gz`; the build removes it). Pico and ESP32 firmwares must be updated together (an old Pico never answers
`ACH_LIST`, and the page says so).

| Endpoint | |
|---|---|
| `GET /api/state` | game header, rich presence, theme |
| `WS /ws` | `list` / `ping` from the browser; list lines and live events to it |
| `GET /api/themes`, `POST /api/theme` (`i=<index>`) | LCD themes / pick one |
| `GET /api/diag`, `GET /api/log?from=N` (`?boot=1`), `POST /api/reboot`, `POST /api/brightness` (`p=10..100`) | debug view |
| `GET /game` | sniff-mode game selector |

The portal comes up once the game's patch has been forwarded to the Pico (or after an error), not at
boot: until then the 48 KB patch buffer leaves the ESP32-C3 with too little heap to serve the page, and a
browser could starve the TLS handshake of the patch download. Before the patch request the ESP32 also
closes the TLS session left open by the login on retroachievements.org, for the same reason.

The serial log of the debug view is kept only on boards with PSRAM (ESP32-S3, 32 KB ring + 8 KB boot
capture); on the ESP32-C3 the RAM is needed for the patch download.

---

## Board revisions

The firmware picks the board revision (`SNES_BOARD_REV`) from the chip selected in the IDE:

| Rev | Board | Module | LCD (BLK/DC/RES/SDA/SCL) | LED G/R | Buzzer | Button | UART to Pico (TX/RX) |
|---|---|---|---|---|---|---|---|
| 1 | proto v0.1 | ESP32-C3 Super Mini | 7/1/0/4/2 | 5/6 | 10 | 8 | 21/20 (UART0 default) |
| 2 | next revisions (not published yet) | Waveshare ESP32-S3-Zero | 7/8/9/10/11 | 13/1 | 12 | 2 | 5/4 (UART0 via GPIO matrix) |

---

## How to Compile the Firmware for ESP32-C3 Super Mini  

This guide explains how to set up **Arduino IDE (version 2.3.4)** to compile and upload the firmware to an **ESP32-C3 Super Mini**.  

---

### Prerequisites  

1. **Download and install Arduino IDE (2.3.4)**  
   - Get it from: [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)  

2. **Install ESP32 board support in Arduino IDE**  
   - Open **Arduino IDE**  
   - Go to **Tools** → **Board** → **Board Manager**  
   - Search for **esp32** and install the version `v3.0.7` of **ESP32 by Espressif Systems**  
   - Once installed, select **ESP32C3 Dev Module** under **Tools** → **Board**  

3. **Install the required libraries**  

   The project uses the following libraries, which you can install via the **Library Manager** or manually from GitHub:  

   - **WiFiManager** (v2.0.17) → [https://github.com/tzapu/WiFiManager](https://github.com/tzapu/WiFiManager)  
   - **PNGdec** (v1.1.2) → [https://github.com/bitbank2/PNGdec](https://github.com/bitbank2/PNGdec)  
   - **TFT_eSPI** (v2.5.43) → [https://github.com/Bodmer/TFT_eSPI](https://github.com/Bodmer/TFT_eSPI)
   - **ESP Async WebServer** (v.3.7.6) -> [https://github.com/ESP32Async/ESPAsyncWebServer](https://github.com/ESP32Async/ESPAsyncWebServer)
   - **Async TCP** (v3.4.0) -> [https://github.com/ESP32Async/AsyncTCP](https://github.com/ESP32Async/AsyncTCP)

   **To install via Library Manager:**  
   - Open **Tools** → **Library Manager**  
   - Search for the library name and click **Install**  

4. **Install the Arduino LittleFS Upload Plugin**

   Visit the [plug-in site](https://github.com/earlephilhower/arduino-littlefs-upload) and follow its instructions. This is used to upload the contents of the `data` folder (the web portal page and its sound) to the LittleFS.

---

### TFT_eSPI Library 

Copy `misc/TFT_eSPI/User_Setup.h` (repo root) into the library folder (usually `/user/Documents/Arduino/libraries/TFT_eSPI`). It holds the LCD pins of both boards (ESP32-C3 and ESP32-S3, picked by `CONFIG_IDF_TARGET_ESP32S3`) and is the same file the NES RA adapter uses, so one copy serves both projects.

### Configuring Arduino IDE for ESP32-C3  

Before compiling, adjust the settings in Arduino IDE:  

1. **Go to** Tools → Board → **ESP32C3 Dev Module**  
2. Configure the following options:  

   - **USB CDC On Boot**: Enabled
   - **CPU Frequency**: 80MHz (WiFi)
   - **Core Debug Level**: None  
   - **Erase All Flash Before Sketch Upload**: Disabled  
   - **Flash Size**: 4MB (32Mb)  
   - **Partition Scheme**: No OTA (2MB APP/2MB SPIFFS)   
   - **Upload Speed**: 921600  

---

### Compiling and Uploading to ESP32-C3  

1. **Connect the ESP32-C3** to your PC via USB  
2. **Select the correct port** under **Tools** → **Port**  
3. **Compile the code** by clicking the ✔️ (Verify) button  
4. **Upload the firmware** to the ESP32 by clicking the 🔼 (Upload) button  
5. **Upload the data** using the Arduino LittleFS Upload plug-in. `[Ctrl]` + `[Shift]` + `[P]`, then "Upload LittleFS to Pico/ESP8266/ESP32".

If everything is set up correctly, the firmware will be flashed, and the ESP32-C3 will be ready to run the project!  

---

### Troubleshooting  

If you encounter issues:  
- **Check the IDE console for errors** and ensure all required libraries are correctly installed.  
- **Confirm the correct board and port** are selected in the Arduino IDE.  
- **Try using a different USB cable or port** if the ESP32 is not detected.  
- **Press the "Boot" button on the ESP32-C3 during upload** if flashing fails.  
- **If you couldn't upload the data folder using the plug-in** follow the Troubleshooting found on the [plug-in site](https://github.com/earlephilhower/arduino-littlefs-upload). 

