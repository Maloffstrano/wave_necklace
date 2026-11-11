# Wave Necklace

Wearable display on a **Waveshare ESP32-S3-Touch-LCD-1.85** (round 360x360). The goal is an ocean wave simulation that responds to necklace movement (via the onboard QMI8658 IMU).
This folder is dedicated to the project: write code and project notes here.

## Layout
```
CLAUDE.md                     these notes
libraries/NecklaceBoard/      shared board support (display, touch, backlight); symlinked into ~/Documents/Arduino/libraries
hello_world_test/             first test sketch: rainbow text, edge dot, tap-to-dim
ocean_wave/                   the real project (currently a placeholder)
tools/                        convert_waveshare_init.py (regenerates the panel init header)
backup/                       factory firmware dump (git-ignored)
```
- Each sketch folder has its own `sketch.yaml` (FQBN + port).
- Board code goes in NecklaceBoard. Sketches keep only app logic (e.g. brightness levels, drawing).
- The `~/Documents/Arduino/libraries/NecklaceBoard` symlink must exist on any machine that builds this (recreate after cloning).

## Toolchain
- No `arduino-cli` on PATH. Use the copy bundled with Arduino IDE, always with the IDE's config:
  ```sh
  CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
  "$CLI" --config-file ~/.arduinoIDE/arduino-cli.yaml compile -u ocean_wave   # build + upload a sketch folder
  ```
- ESP32 core esp32:esp32 3.3.12. GFX Library for Arduino 1.6.9 (in ~/Documents/Arduino/libraries).
- esptool: `~/Library/Arduino15/packages/esp32/tools/esptool_py/<ver>/esptool`. The version changes on core upgrades, so glob for it.

## Board quirks
- **Partition scheme:** the board's default `esp_sr_16` fails to upload (expects a speech model `srmodels.bin`). Use `PartitionScheme=app3M_fat9M_16MB`.
- `CDCOnBoot=cdc` so Serial goes over native USB (`/dev/cu.usbmodem101`).
- A large `read-flash` at `-b 921600` failed with serial corruption, but the default baud works.
- Chip: ESP32-S3R8 (8MB PSRAM, OPI), 16MB flash.

## Pinout (from Waveshare wiki; also constants in NecklaceBoard.h)
| Function | Pin |
|---|---|
| LCD QSPI CS / SCK | 21 / 40 |
| LCD D0..D3 | 46, 45, 42, 41 |
| LCD TE | 18 |
| LCD backlight | 5 |
| I2C `Wire` SDA / SCL (expander, IMU QMI8658, RTC PCF85063) | 11 / 10 |
| Touch CST816 `Wire1` SDA / SCL / INT | 1 / 3 / 4 |
| TCA9554 expander (0x20) | EXIO1 = touch reset (bit0), EXIO2 = LCD reset (bit1), EXIO3 = SD CS, EXIO4/5 = IMU INT2/INT1 |

- LCD and touch resets are on the expander. NecklaceBoard::begin() pulses them before display init.
- **The GFX library's built-in ST77916 init tables do NOT work** (vertical coloured stripes, plus temporary image retention). Use `st77916_waveshare_init.h`, generated from Waveshare's demo with COLMOD 16-bit sent first.
- Waveshare ships two panel revisions. Register 0x04 is read over QSPI via esp_lcd before Arduino_GFX takes the bus: `00 02 7F 7F` means rev2, otherwise rev1.
- Reference drivers are in Waveshare's demo zip (Google Drive link on the wiki, about 200 MB), under `Arduino/examples/LVGL_Arduino/`. It also has QMI8658 IMU code (Gyro_QMI8658.cpp), which will be useful for ocean_wave.
- Touch: CST816 at 0x15. Register 0x02 = finger count. Write non-zero to 0xFE to disable auto-sleep.
- Backlight: LEDC on GPIO5, 20 kHz, 10-bit; duty 1024 = fully on.
- Fonts: the built-in font looks blocky when scaled. Use GFXfont files (Adafruit FreeFonts with the `#include <Adafruit_GFX.h>` line removed). They draw no background, so recolouring identical text is flicker-free, but changing the text needs an erase.

## Status and next steps (as of 2026-11-10)
- hello_world_test works: rainbow text, dot every 1 s, tap cycles 4 backlight levels.
- User feedback: the dimmest level (duty 10/1024) is right for a dark room. Keep it.
- Likely change: 3 brightness levels (3 taps per cycle). Slide-to-dim was judged overkill.
- The user is creating a **public** GitHub repo from this whole folder. The README must credit **Waveshare** (panel init sequences derived from their demo) and **Adafruit** (FreeSansBold18pt7b font, BSD licence).
- Next: build ocean_wave (wave simulation driven by the IMU).

## Backup
- `backup/factory_firmware_16MB.bin`: full flash dump of the factory firmware (2026-11-10), not in git. To restore it:
  `esptool --port /dev/cu.usbmodem101 write-flash 0 backup/factory_firmware_16MB.bin`
