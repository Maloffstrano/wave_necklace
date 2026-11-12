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

## Git / GitHub
- Public repo: https://github.com/Maloffstrano/wave_necklace (the user's **personal** account Maloffstrano, never a work account).
- `gh` is at `~/bin/gh` (not on PATH), signed in as Maloffstrano.
- Repo-local git config: identity `Maloffstrano <Maloffstrano@users.noreply.github.com>`, and a credential helper using `~/bin/gh auth git-credential`. There is no global git config; keep it that way.
- MIT licence. README credits Waveshare and Adafruit; keep them when editing.

## Board quirks
- **Partition scheme:** the board's default `esp_sr_16` fails to upload (expects a speech model `srmodels.bin`). Use `PartitionScheme=app3M_fat9M_16MB`.
- `CDCOnBoot=cdc` so Serial goes over native USB. The port name depends on which Mac USB socket is used (seen: `/dev/cu.usbmodem101`, `/dev/cu.usbmodem1101`). Update `default_port` in each sketch.yaml if uploads say the port doesn't exist.
- Reading serial without the IDE: `stty -f <port> 115200 raw; cat <port>` (kill it after a few seconds). `arduino-cli monitor` exits immediately when stdin isn't a terminal.
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
- **Fast frame output (NecklaceBoard::drawFrame / drawRows):** a second SPI device on the display bus, rendering strips of 20 rows into two DMA buffers in internal RAM, overlapped with sending. Pixels must be in panel byte order (`panelColor()`). Requires the GFX bus to be created with `is_shared_interface = true`, otherwise the library holds the bus forever and our device hangs. After using the fast path, don't draw on the panel through the Arduino_GFX object (its cached address window goes stale); use `clearScreen()` etc. instead.
- Performance (2026-11-11): wave test pattern 82 FPS (12.2 ms: 3.2 prepare + 9.0 render/send overlapped, of which 7.9 ms CPU). Diagnostics screen 41 FPS (its PSRAM canvas clear/draw costs about 12.7 ms). The second core is still unused.
- **IMU reads run in a FreeRTOS task on core 0** (NecklaceBoard::startMotionTask, 250 Hz). Use latestMotion(m, peakJolt); don't call readMotion() from the loop once it's running. "Jolt" = distance of acceleration from its ~0.1 s average, all axes, peak since last call.
- **The QMI8658 accelerometer low-pass filter must stay OFF** (CTRL5 = 0x10). Its narrowest mode (~7 Hz at 250 Hz) flattened a fist bang from >2 g to 0.01 g. Smoothing for tilt is done in software.
- Measured jolts on the desk: rest ~0, typing nearby 0.07 g, three-finger tap 0.2 g, fist bang >2 g. Splash threshold 0.1 g. Walking while worn is still untested (footsteps may be 0.1-0.5 g).
- **Material (ocean_wave/Material.h/.cpp):** everything about the fluid (wave physics, splashes, curl, foam, colours, pool ripples) is one struct of floats; WATER holds the tuned values. SideView/PoolView::setMaterial() applies one (and rebuilds colour tables, cheap enough per frame). Material::mix() blends two field by field. Bowl constants (lean levels, grid, steps, safety limits) stay in the views.
- Side view (ocean_wave/SideView.cpp): 1D shallow-water sloshing in a frame aligned with smoothed gravity (0.35 s), numerical diffusion against the zig-zag instability, square-root boost for small motions, ambient swell, ambient drops only once the water is visibly still (all flow < 6 px/s for 2 s and 3 s since the last disturbance), splashes from jolts, anti-aliased waterline, checkerboard dither against RGB565 banding. Wave speed 165 px/s ("slow and stately", halved from 330; splash gain lowered to match). Wall curl is a thick tongue that starts in the water's own colour and thins and pales as it climbs (launch gain 2.8, no launch below 220 px/s, emit rate 0.04); falling pieces turn white and land as flecks that ride the surface for 0.8 s; froth fades to the water colour within 50 px of the wall. About 60 FPS (render ~14.5 ms). The file is compiled with -O3.
- Keep trig arguments small: wrap time-based phases with `fmodf(..., 2*PI)`. `sinf()` slows down a lot for large arguments, which made the frame rate decay from 82 to 50 FPS over a few minutes.
- Pool (ocean_wave/PoolView.cpp, lean > 80, back below 72): 2D wave equation on a 91x91 grid of 4 px cells at 120 steps/s, shaded by slope, resting colour = the full side view's depth gradient oriented by the flat necklace's slight tip. Calm on entry; bumps ignored for 1.5 s (setting down) and around taps (a tap shakes a hand-held board: jolts are held back one frame). One ripple per tap, a ring from the rim for jolts > 0.6 g; smaller bumps (handling it) and random drops do nothing. The light area swings round at most 1.5 rad/s, so it never jumps across when the tip reverses. About 45 FPS.
- Brightness: touch and hold 0.6 s -> dimmest, held there 0.8 s, then ramps (exponentially) to full over 2.5 s; release keeps it. Quick taps are free (pool ripples).
- Touch: CST816 at 0x15. NecklaceBoard::readTouch(x, y) gives the position. Register 0x02 = finger count. Write non-zero to 0xFE to disable auto-sleep.
- Backlight: LEDC on GPIO5, 20 kHz, 10-bit; duty 1024 = fully on.
- Buttons: BOOT = GPIO0 (active low, free after boot); PWR = GPIO6 (active low); RST is a hardware reset. **Power latch GPIO7 must be HIGH or the board turns off on battery when PWR is released.** NecklaceBoard::begin() sets it; powerOff() clears it.
- ocean_wave controls: BOOT short press toggles Wave/Diagnostics screens; short PWR press calms the water (SideView::reset); hold PWR 2 s = power off.
- Fonts: the built-in font looks blocky when scaled. Use GFXfont files (Adafruit FreeFonts with the `#include <Adafruit_GFX.h>` line removed). They draw no background, so recolouring identical text is flicker-free, but changing the text needs an erase.

## Status and next steps (as of 2026-11-10)
- hello_world_test works: rainbow text, dot every 1 s, tap cycles 4 backlight levels.
- User feedback: the dimmest level (duty 10/1024) is right for a dark room. Keep it.
- Likely change: 3 brightness levels (3 taps per cycle). Slide-to-dim was judged overkill.
- **This board's accelerometer was stuck (2026-11-11):** ay was pinned at full scale on every range (+4/+8/+16 g) and ax/az were wrong. Waveshare's factory firmware showed the same, so it's hardware. It later came unstuck (|a| reads about 0.98 g at rest) after tapping, a factory-firmware run (8 kHz ODR) and soft resets, which fits MEMS stiction. It may stick again after a knock. The code should sanity-check |a| (about 1 g at rest) and fall back to the gyro if the accelerometer is implausible. The gyro has always worked.
- **Orientation design (decided 2026-11-11, details in things-I-learned.md):** "lean" = tilt from vertical (0 upright, +90 flat face up, -90 face down; face down reads az = +1 g). Worn lean measured +26 to +34 upright, +50 leaning back in a chair, +66 big lean. Side view (1D sloshing); from lean 40 to 80 the water level rises until only a sliver of sky is left (replaces the planned crossfade, 2026-11-11); top-down ripple pool above 80; return to side view below 72. Lean averaged over about 0.5 s; sloshing uses raw readings. Face down: blank and wake when picked up (done: smoothed lean < -70 blanks, > -50 wakes). Still and flat for about 30 s: rest (done: in the pool, 30 s without touch or a bump > 0.08 g fades the backlight out over 3 s and stops drawing; touch, bump or lifting wakes it with a 0.3 s fade-in to the chosen brightness). All backlight changes go through updateBacklight() in ocean_wave.ino. Ripples when flat come from table bumps, touch (ripple at the finger) and random drops.
- The user is collecting photos and videos for a future design-process document; things-I-learned.md doubles as its source (may be renamed). Keep adding design decisions and measurements there.
- Next: build ocean_wave (wave simulation driven by the IMU). Show frame time and FPS from the first version. The user may switch to Claude Fable 5.2 if more performance is needed, so decide on measured numbers.

## Backup
- `backup/factory_firmware_16MB.bin`: full flash dump of the factory firmware (2026-11-10), not in git. To restore it:
  `esptool --port /dev/cu.usbmodem101 write-flash 0 backup/factory_firmware_16MB.bin`
