# snes-pico-firmware

Firmware for the **RP2350B** (WeAct Studio RP2350B Core Board, 48 GPIO) of the SNES RetroAchievements Adapter. It reads and identifies the cartridge, then listens to the console bus, keeps a copy of the memory and runs [rcheevos](https://github.com/RetroAchievements/rcheevos) to evaluate the achievements. All Internet traffic and the screen are handled by the ESP32 ([snes-esp-firmware](../snes-esp-firmware)), connected over UART.

It is based on the architecture of the [NES RA Adapter](https://github.com/odelot/nes-ra-adapter) `nes-pico-firmware`. It was adapted for the SNES: a wider bus (24-bit address), a faster clock and 128 KB of WRAM.

---

## How it works

### 1. Cartridge identification

At power-on the Pico:

1. Holds the console in reset (`GP44` → Q1 → `/RESET`).
2. Opens the bus switches (`GP45`), so the console is isolated from the cartridge.
3. Drives A0–A23, /CART and /RD itself and reads the cartridge:
   - It parses the internal header (title, LoROM/HiROM, ROM/SRAM size, region).
   - It computes the **MD5 of the whole ROM**, the same hash RetroAchievements uses for SNES ROM files. ROMs whose size is not a power of two (10/12/20/24 Mbit, e.g. Super Metroid, Mega Man X) show up mirrored on the bus, and the firmware detects that so the hash matches the real dump.
4. Sends `CART_INFO=` (header + MD5) to the ESP32, closes the switches and releases the console.

### 2. Bus capture (core 1)

The bus is sampled by PIO state machines, **one word per write cycle**, at a fixed point inside the data-valid window. A DMA channel streams each one into a ring buffer, and core 1 drains the rings and updates the mirrors.

| PIO / SM | Trigger | What it captures |
|---|---|---|
| PIO0 SM0 | /WRAM + /WR + PHI2 | Writes to the console WRAM (A-bus) |
| PIO0 SM1 | /CART + /WR + PHI2 | Writes to cartridge space (LoROM SRAM) |
| PIO0 SM2 | /WR, /WRAM inactive | Writes to the $6000–7FFF window (HiROM SRAM) |
| PIO1 SM0 | /PAWR | B-bus writes to the WRAM port $2180–$2183 (DMA into WRAM) |

- **Two PIOs, one window:** the RP2350B lets PIO1 see GPIO 16–47 (`pio_set_gpio_base(pio1, 16)`). The shared signals (D0–D7, PHI2, /WR, /WRAM, /CART…) live in the overlap window GPIO 16–31, so both PIOs see them on the same pins and no GPIO is duplicated.
- **Mirrors:**
  - The firmware keeps a full copy of the **128 KB WRAM** and up to **32 KB of cartridge SRAM**.
  - A full copy (instead of tracking only some addresses) is needed for achievements that use AddAddress / pointers, because their targets are only known at runtime.
  - Bank resolution: A22 (on GP29) tells a WRAM mirror access ($00–$3F:0000–1FFF) from a direct $7E/$7F access.
- **WRAM port:** a lot of data reaches the WRAM through the $2180 port (for example DMA from ROM). That never puts the WRAM address on the A-bus, so the firmware keeps a shadow of the port pointer ($2181–$2183) and its auto-increment.

### 3. Frames and achievements (core 0)

- **Frame pacing:** `rc_client_do_frame` runs on the **real vblank**. The firmware detects it as the burst of the OAM DMA (544 writes to $2104) that games run in the NMI handler. If no vblank arrives in 25 ms (menus, force-blank), a timer takes over. NTSC/PAL comes from the cartridge header.
- **Atomic evaluation:** each frame is evaluated on a snapshot of the addresses rcheevos uses, so core 1 never changes a value in the middle of an evaluation.
- **Console reset:** the `/RESET` line is monitored (`GP43`). When you press RESET on the console, the mirrors are cleared and `rc_client_reset()` runs, the same as an emulator does.

### rcheevos memory map

```
0x000000 – 0x01FFFF   WRAM  (128 KB, banks $7E/$7F)
0x020000 – 0x027FFF   SRAM  (up to 32 KB)
```

---

## Pinout (prototype v0.1, after the mod)

```
GPIO   Signal          Notes
──────────────────────────────────────────────────────────────────
 0-16  A0–A16          cartridge side of the switches (output only during cart ID)
17-24  D0–D7           cartridge side of the switches
 25    PHI2            direct tap
 26    /WR             cartridge side
 27    /WRAM           direct tap
 28    /CART           cartridge side
 29    A22             cartridge side (v0.1 needs the mod; see hardware/README.md)
 30    /REFRESH        direct tap
 31    /PAWR           direct tap
32-36  A17–A21         cartridge side
 37    /RD             cartridge side (only used during cart ID)
 38    A23             cartridge side
39-41  PA0, PA1, PA7   direct taps (B-bus address, partial decode of $2180–83)
 42    /PARD           direct tap
 43    /RESET          monitor
 44    RESET_DRIVE     output → R2 → Q1 (holds the console in reset)
 45    BUS_CTRL        output → enable of all 74HC4066 (HIGH = connected)
 46    UART0 TX        → ESP32 RX
 47    UART0 RX        ← ESP32 TX
```

The RP2350 inputs are 5 V tolerant, so the bus taps have no level shifter. The Pico only drives the cartridge (at 3.3 V) while the console is isolated.

Full board connections: [hardware/docs/netlist.md](../hardware/docs/netlist.md).

---

## Compile-time options (`src/main.c`)

| Define | Default | Use |
|---|---|---|
| `A22_ON_GP29` | on | A22 on GP29 and /RD on GP37 (v0.1 **with** the mod, and later boards). Comment it out only for an unmodified v0.1. Cart ID still works, but WRAM bank resolution reads /RD as A22. |
| `BUS_SWITCH_4066` | on | Bus switches are 74HC4066 (active-HIGH enable). Comment it out for boards with '245 / SN74CBT3245 switches (active-LOW `/OE`). |
| `BUS_SWITCH_4066_OPEN_DRAIN` | off | Plan B for 4066 parts that don't close with a 3.3 V enable. Also move JP1 to 2-3 (pull-up to 5 V). |
| `RA_HARDCORE` | 0 | 0 = casual (softcore) mode. |
| `SNIFF_ONLY_MODE` | off | Bring-up mode for wires soldered straight to the console: no cart ID, the game is picked from a fixed list. |
| `MIRROR_UART_TO_USB` | 1 | Copies all Pico → ESP32 traffic to the USB serial (credentials are masked). |

---

## Building

Requirements:

- [Pico SDK 2.x](https://github.com/raspberrypi/pico-sdk) (with RP2350 support)
- Arm GNU toolchain
- CMake ≥ 3.19

rcheevos v11.6.0 is downloaded by CMake.

The `pico` preset in `CMakePresets.json` points to the SDK and toolchain paths of the author's machine (`C:\VSARM\...`). Change `PICO_SDK_PATH`, `toolchainFile` and `CMAKE_MAKE_PROGRAM` to match yours. The board is `solderparty_rp2350_stamp_xl`, a generic RP2350B (48 GPIO) definition that works for the WeAct board. Don't use `pico2`: that is the RP2350A, with 30 GPIO.

```sh
cmake --preset pico
cmake --build --preset build_pico
```

Output: `build_pico/src/snes-pico-firmware.uf2`.

### Flashing

1. Hold the **BOOT** button of the WeAct board.
2. Plug it into the computer over USB.
3. A drive called `RP2350` appears. Copy `snes-pico-firmware.uf2` to it, and the board reboots by itself.

Flash it with the board **out of the console**.

### Unit tests (host)

```sh
cmake --preset unit_tests
cmake --build --preset build_tests
ctest --preset test_all
```

The tests cover the PIO word decoding, the capture consumers, the $2180 port tracker, the header parser and the address translation. They re-declare that logic because `main.c` depends on the Pico SDK, so keep them in sync with `src/main.c`.

---

## Debug console (USB serial)

The WeAct USB serial accepts the same commands as the UART link (end lines with `\n` or `\r`), plus some debug commands. You can test the board without the ESP32:

| Command | What it does |
|---|---|
| `SYNC` / `RESET` | Handshake / reboot the Pico |
| `READ_CART` | Run cartridge identification (header + MD5 on the console) |
| `DBG` | Report of the capture pipeline: wires → PIO → DMA → consumers, vblank, SRAM windows |
| `PEEK=<addr hex>` | Read a byte from the mirror (rcheevos address: `0x000000+` WRAM, `0x020000+` SRAM) |
| `WATCH=<addr hex>` / `WATCH=OFF` | Print a byte every time it changes |
| `GREP=<addr hex>` / `GREP=OFF` | Count and show the captures that wrote to one address |
| `VSYNC=ON` / `VSYNC=OFF` | Frame pacing by the detected vblank (default) or by a free-running timer |
| `RESETSNES` | Pulse the console reset |
| `GAME?` / `GAME=<n>` | List / pick a game (only in `SNIFF_ONLY_MODE`) |

The serial protocol with the ESP32 is described in a comment in `src/main.c` (section *ESP32 serial protocol*).

---

## Known limitations

- **SA-1 games** (Super Mario RPG, Kirby Super Star, Kirby's Dream Land 3…): the SA-1 runs the game from its own memory inside the cartridge, and that data never reaches the connector. This is a hardware limit.
- **Super FX games** (Star Fox, Yoshi's Island) work, because the game state lives in the console WRAM.
- **Cartridge SRAM:**
  - Only LoROM SRAM up to 32 KB and HiROM SRAM up to 8 KB are tracked. Larger SRAM (Star Ocean, Tales of Phantasia, Dragon Quest VI) needs A17–A23 in the capture (PIO1 SM1, not enabled yet).
  - The SRAM of LoROM + DSP boards (e.g. Super Mario Kart) is only mapped in `SNIFF_ONLY_MODE` for now.
- **ExHiROM** and the other enhancement chips are untested.
- **Flash carts** are not supported: the adapter would hash the flash cart's menu, not the game.

## License

GPL-3.0, the same as the NES project.
