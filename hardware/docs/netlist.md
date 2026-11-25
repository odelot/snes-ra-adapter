# SNES RA Adapter: netlist (prototype v0.1)

> Connections of `proto-v0.1` as fabricated. **Every v0.1 board gets a mod** (see the [hardware README](../README.md#required-mod-2-cuts--2-wires)) that swaps two Pico pins. After the mod, **A22 goes to GP29** and **/RD goes to GP37**. The tables below show the board **before** the mod.

## Cartridge connector (J1 = edge → console, J2 = straddle slot ← cartridge)

Pin 1 is on the front face, on the left when you look at the front. Pins 1–31 are on the front (F.Cu) and 32–62 on the back (B.Cu), with 32 behind 1. The pitch is 2.50 mm, with 7.5 mm gaps between 4|5 and 27|28. J1 and J2 have the same X for each pin.

| Pin | Face | X (mm) | Signal | Path |
|---:|:---:|---:|---|---|
| 1 | F | -42.5 | SYSCLK | direct J1↔J2 |
| 2 | F | -40.0 | EXPAND | direct J1↔J2 |
| 3 | F | -37.5 | PA6 | direct J1↔J2 |
| 4 | F | -35.0 | /PARD | direct J1↔J2 · Pico GP42 |
| 5 | F | -27.5 | GND | direct (power) |
| 6 | F | -25.0 | A11 | J1 `C_A11` ↔ **U1 ch1** ↔ J2 `A11` · Pico GP11 |
| 7 | F | -22.5 | A10 | J1 `C_A10` ↔ **U1 ch3** ↔ J2 `A10` · Pico GP10 |
| 8 | F | -20.0 | A9 | J1 `C_A9` ↔ **U2 ch1** ↔ J2 `A9` · Pico GP9 |
| 9 | F | -17.5 | A8 | J1 `C_A8` ↔ **U2 ch3** ↔ J2 `A8` · Pico GP8 |
| 10 | F | -15.0 | A7 | J1 `C_A7` ↔ **U3 ch1** ↔ J2 `A7` · Pico GP7 |
| 11 | F | -12.5 | A6 | J1 `C_A6` ↔ **U3 ch3** ↔ J2 `A6` · Pico GP6 |
| 12 | F | -10.0 | A5 | J1 `C_A5` ↔ **U4 ch1** ↔ J2 `A5` · Pico GP5 |
| 13 | F | -7.5 | A4 | J1 `C_A4` ↔ **U4 ch3** ↔ J2 `A4` · Pico GP4 |
| 14 | F | -5.0 | A3 | J1 `C_A3` ↔ **U5 ch1** ↔ J2 `A3` · Pico GP3 |
| 15 | F | -2.5 | A2 | J1 `C_A2` ↔ **U5 ch3** ↔ J2 `A2` · Pico GP2 |
| 16 | F | +0.0 | A1 | J1 `C_A1` ↔ **U6 ch1** ↔ J2 `A1` · Pico GP1 |
| 17 | F | +2.5 | A0 | J1 `C_A0` ↔ **U6 ch3** ↔ J2 `A0` · Pico GP0 |
| 18 | F | +5.0 | /IRQ | direct J1↔J2 |
| 19 | F | +7.5 | D0 | J1 `C_D0` ↔ **U7 ch2** ↔ J2 `D0` · Pico GP17 |
| 20 | F | +10.0 | D1 | J1 `C_D1` ↔ **U7 ch4** ↔ J2 `D1` · Pico GP18 |
| 21 | F | +12.5 | D2 | J1 `C_D2` ↔ **U8 ch2** ↔ J2 `D2` · Pico GP19 |
| 22 | F | +15.0 | D3 | J1 `C_D3` ↔ **U8 ch4** ↔ J2 `D3` · Pico GP20 |
| 23 | F | +17.5 | /RD | J1 `C_RD_N` ↔ **U9 ch2** ↔ J2 `RD_N` · Pico GP29 |
| 24 | F | +20.0 | CIC0 | direct J1↔J2 |
| 25 | F | +22.5 | CIC1 | direct J1↔J2 |
| 26 | F | +25.0 | /RESET | direct J1↔J2 · Pico GP43 |
| 27 | F | +27.5 | +5V | direct (power) |
| 28 | F | +35.0 | PA0 | direct J1↔J2 · Pico GP39 |
| 29 | F | +37.5 | PA2 | direct J1↔J2 |
| 30 | F | +40.0 | PA4 | direct J1↔J2 |
| 31 | F | +42.5 | AUDIO_L | direct J1↔J2 |
| 32 | B | -42.5 | /WRAM | direct J1↔J2 · Pico GP27 |
| 33 | B | -40.0 | /REFRESH | direct J1↔J2 · Pico GP30 |
| 34 | B | -37.5 | PA7 | direct J1↔J2 · Pico GP41 |
| 35 | B | -35.0 | /PAWR | direct J1↔J2 · Pico GP31 |
| 36 | B | -27.5 | GND | direct (power) |
| 37 | B | -25.0 | A12 | J1 `C_A12` ↔ **U1 ch2** ↔ J2 `A12` · Pico GP12 |
| 38 | B | -22.5 | A13 | J1 `C_A13` ↔ **U1 ch4** ↔ J2 `A13` · Pico GP13 |
| 39 | B | -20.0 | A14 | J1 `C_A14` ↔ **U2 ch2** ↔ J2 `A14` · Pico GP14 |
| 40 | B | -17.5 | A15 | J1 `C_A15` ↔ **U2 ch4** ↔ J2 `A15` · Pico GP15 |
| 41 | B | -15.0 | A16 | J1 `C_A16` ↔ **U3 ch2** ↔ J2 `A16` · Pico GP16 |
| 42 | B | -12.5 | A17 | J1 `C_A17` ↔ **U3 ch4** ↔ J2 `A17` · Pico GP32 |
| 43 | B | -10.0 | A18 | J1 `C_A18` ↔ **U4 ch2** ↔ J2 `A18` · Pico GP33 |
| 44 | B | -7.5 | A19 | J1 `C_A19` ↔ **U4 ch4** ↔ J2 `A19` · Pico GP34 |
| 45 | B | -5.0 | A20 | J1 `C_A20` ↔ **U5 ch2** ↔ J2 `A20` · Pico GP35 |
| 46 | B | -2.5 | A21 | J1 `C_A21` ↔ **U5 ch4** ↔ J2 `A21` · Pico GP36 |
| 47 | B | +0.0 | A22 | J1 `C_A22` ↔ **U6 ch2** ↔ J2 `A22` · Pico GP37 |
| 48 | B | +2.5 | A23 | J1 `C_A23` ↔ **U6 ch4** ↔ J2 `A23` · Pico GP38 |
| 49 | B | +5.0 | /CART | J1 `C_CART_N` ↔ **U7 ch1** ↔ J2 `CART_N` · Pico GP28 |
| 50 | B | +7.5 | D4 | J1 `C_D4` ↔ **U7 ch3** ↔ J2 `D4` · Pico GP21 |
| 51 | B | +10.0 | D5 | J1 `C_D5` ↔ **U8 ch1** ↔ J2 `D5` · Pico GP22 |
| 52 | B | +12.5 | D6 | J1 `C_D6` ↔ **U8 ch3** ↔ J2 `D6` · Pico GP23 |
| 53 | B | +15.0 | D7 | J1 `C_D7` ↔ **U9 ch1** ↔ J2 `D7` · Pico GP24 |
| 54 | B | +17.5 | /WR | J1 `C_WR_N` ↔ **U9 ch3** ↔ J2 `WR_N` · Pico GP26 |
| 55 | B | +20.0 | CIC2 | direct J1↔J2 |
| 56 | B | +22.5 | CIC3 | direct J1↔J2 |
| 57 | B | +25.0 | PHI2 | direct J1↔J2 · Pico GP25 |
| 58 | B | +27.5 | +5V | direct (power) |
| 59 | B | +35.0 | PA1 | direct J1↔J2 · Pico GP40 |
| 60 | B | +37.5 | PA3 | direct J1↔J2 |
| 61 | B | +40.0 | PA5 | direct J1↔J2 |
| 62 | B | +42.5 | AUDIO_R | direct J1↔J2 |

## Analog switches (74HC4066, U1–U9)

The **Y** side goes to the console (`C_*`), and the **Z** side to the cartridge and the Pico. All `nE` pins are on `BUS_CTRL` (high = connected). The channels follow the order of the edge pins (U1–U5 on the left, U6–U9 on the right) to keep the traces short. The firmware does not depend on this map, because all switches open and close together.

| Chip | ch1 (1Y/1Z, E=13) | ch2 (2Y/2Z, E=5) | ch3 (3Y/3Z, E=6) | ch4 (4Y/4Z, E=12) |
|---|---|---|---|---|
| U1 | A11 | A12 | A10 | A13 |
| U2 | A9 | A14 | A8 | A15 |
| U3 | A7 | A16 | A6 | A17 |
| U4 | A5 | A18 | A4 | A19 |
| U5 | A3 | A20 | A2 | A21 |
| U6 | A1 | A22 | A0 | A23 |
| U7 | CART_N | D0 | D4 | D1 |
| U8 | D5 | D2 | D6 | D3 |
| U9 | D7 | RD_N | WR_N | *unused (GND)* |

## Pico (U10, WeAct RP2350B)

| GPIO | Net | GPIO | Net |
|---:|---|---:|---|
| GP0 | A0 | GP24 | D7 |
| GP1 | A1 | GP25 | PHI2 |
| GP2 | A2 | GP26 | WR_N |
| GP3 | A3 | GP27 | WRAM_N |
| GP4 | A4 | GP28 | CART_N |
| GP5 | A5 | GP29 | RD_N |
| GP6 | A6 | GP30 | REFRESH_N |
| GP7 | A7 | GP31 | PAWR_N |
| GP8 | A8 | GP32 | A17 |
| GP9 | A9 | GP33 | A18 |
| GP10 | A10 | GP34 | A19 |
| GP11 | A11 | GP35 | A20 |
| GP12 | A12 | GP36 | A21 |
| GP13 | A13 | GP37 | A22 |
| GP14 | A14 | GP38 | A23 |
| GP15 | A15 | GP39 | PA0 |
| GP16 | A16 | GP40 | PA1 |
| GP17 | D0 | GP41 | PA7 |
| GP18 | D1 | GP42 | PARD_N |
| GP19 | D2 | GP43 | RESET_N |
| GP20 | D3 | GP44 | RESET_DRIVE |
| GP21 | D4 | GP45 | BUS_CTRL |
| GP22 | D5 | GP46 | PICO_TX |
| GP23 | D6 | GP47 | PICO_RX |

`5V` ← `+5V_MOD` (after D1). VBUS, VREF, 3V3, RUN and EN are not connected.

## ESP32-C3 Super Mini (U11) and peripherals

| Pin | Net | Connection |
|---|---|---|
| 5V | +5V_MOD |  |
| G | GND |  |
| 3.3 | +3V3_ESP | LCD VCC, R5, BZ1+ |
| 0 | LCD_RES | J3.5 RES |
| 1 | LCD_DC | J3.6 DC |
| 2 | LCD_SCL | J3.3 SCL |
| 4 | LCD_SDA | J3.4 SDA |
| 5 | LED_GREEN | R6 100R → D2 green |
| 6 | LED_RED | R7 100R → D2 red |
| 7 | LCD_BLK | J3.7 BLK (brightness PWM) |
| 8 | BTN_CFG | SW1 → GND, R5 10k pull-up |
| 10 | BUZZER | R4 1k → Q2 BC548 → BZ1 |
| 20 | PICO_TX | ← Pico GP46 |
| 21 | PICO_RX | → Pico GP47 |

GPIO3 and GPIO9 are free. The map comes from the `TFT_eSPI` User_Setup (MOSI 4, SCLK 2, DC 1, RST 0, no CS) and from `snes-esp-firmware.ino`.

## Power, reset and control

- `+5V` (console, pins 27/58) powers the VCC of U1–U9, C1–C9 (100n) and C10 (47u).
- D1 (1N5817): `+5V` → `+5V_MOD` (Pico 5V, ESP 5V, C11 10u). This keeps the USB of a module from powering the console.
- Reset: `GP44` (`RESET_DRIVE`) → R2 1k → Q1 BC548 (R3 10k base→GND). The collector pulls `RESET_N` (pin 26) low.
- `BUS_CTRL` (`GP45`) → R1 10k → JP1:
  - Bridge 1-2 (default): pull-down, so the bus stays isolated at power-on.
  - Bridge 2-3: pull-up to 5V, for the firmware's open-drain plan B.
- J4: UART debug between Pico and ESP (1 GND, 2 `PICO_TX`, 3 `PICO_RX`).
