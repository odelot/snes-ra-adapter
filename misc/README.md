# misc

Auxiliary files used by the firmwares.

- **[TFT_eSPI/User_Setup.h](TFT_eSPI/User_Setup.h):** LCD configuration for the [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) library.
  - Copy it into your `TFT_eSPI` library folder (usually `Documents/Arduino/libraries/TFT_eSPI`) before you compile `snes-esp-firmware`.
  - It has the pins of both ESP32-C3 and ESP32-S3 boards, picked automatically by the chip selected in the IDE.
  - It is the same file the NES RA Adapter uses, so one copy serves both projects.

