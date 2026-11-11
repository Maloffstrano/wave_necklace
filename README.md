# Wave Necklace

A wearable necklace built on the **Waveshare ESP32-S3-Touch-LCD-1.85**, a round 360×360 touch display with an ESP32-S3, an IMU and a battery connector. The goal is an ocean wave on the round screen that sloshes and rolls as the wearer moves.

> **Status:** early. The board support and a hello-world test sketch work. The ocean wave is next.

## Hardware

- [Waveshare ESP32-S3-Touch-LCD-1.85](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.85)
  - ESP32-S3R8 (8 MB PSRAM), 16 MB flash
  - 1.85" round IPS display, 360×360, ST77916 controller over QSPI
  - CST816 capacitive touch
  - QMI8658 6-axis IMU (for wave motion)
  - LiPo battery connector with charging

## Repository layout

```
libraries/NecklaceBoard/   board support: display, touch, backlight
hello_world_test/          test sketch: rainbow text, edge dot, tap to change brightness
ocean_wave/                the necklace itself (work in progress)
tools/                     helper scripts
```

## Building

Requirements:
- Arduino IDE 2.x, or `arduino-cli`
- ESP32 Arduino core (`esp32:esp32`) 3.3.x
- [GFX Library for Arduino](https://github.com/moononournation/Arduino_GFX) 1.6.x

The `NecklaceBoard` library has to be visible to Arduino. Symlink it into your Arduino libraries folder:

```sh
ln -s "$PWD/libraries/NecklaceBoard" ~/Documents/Arduino/libraries/NecklaceBoard
```

Each sketch folder has a `sketch.yaml` with the board settings, so `arduino-cli` can build and upload directly:

```sh
arduino-cli compile -u hello_world_test
```

The upload port in `sketch.yaml` is `/dev/cu.usbmodem101` (macOS). Change it to match your machine.

If you use the Arduino IDE, select **Waveshare ESP32-S3-Touch-LCD-1.85**, and change **Partition Scheme** to *16M Flash (3MB APP/9.9MB FATFS)* and **USB CDC On Boot** to *Enabled*.

### Board gotchas

- **The default partition scheme won't upload.** The board's default (*ESP SR 16M*) expects a speech-recognition model file that ordinary sketches never build. Use the partition scheme above.
- **Use Waveshare's display init sequence.** GFX Library for Arduino's built-in ST77916 sequences give a screen of coloured vertical stripes on this panel. `NecklaceBoard` uses Waveshare's own sequences instead, and detects which of the two panel revisions is fitted.
- **The display and touch resets are on an I/O expander** (TCA9554 at I2C 0x20), not on GPIOs. `NecklaceBoard::begin()` handles them.

## Using NecklaceBoard

```cpp
#include <NecklaceBoard.h>

Arduino_GFX *gfx;

void setup() {
  gfx = NecklaceBoard::begin();      // display cleared, backlight on, touch ready
  NecklaceBoard::setBacklight(10);   // 0 .. NecklaceBoard::BACKLIGHT_MAX (1024)
  gfx->println("Hello");
}

void loop() {
  if (NecklaceBoard::isTouched()) { /* ... */ }
}
```

## Credits

- **Waveshare**: the ST77916 panel init sequences in `libraries/NecklaceBoard/src/st77916_waveshare_init.h`, and the panel revision detection, are derived from Waveshare's ESP32-S3-Touch-LCD-1.85 demo code ([wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.85)). `tools/convert_waveshare_init.py` converts them from Waveshare's format.
- **Adafruit**: the GFX fonts (`hello_world_test/FreeSansBold18pt7b.h`, `ocean_wave/FreeSans12pt7b.h`) come from the [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library) (BSD licence), which converted them from GNU FreeFont.
- **moononournation**: [GFX Library for Arduino](https://github.com/moononournation/Arduino_GFX) drives the display.

## Licence

MIT. See [LICENSE](LICENSE). The third-party material credited above keeps its original licence.
