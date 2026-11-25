// Board support for the necklace's round display boards:
//   - Waveshare ESP32-S3-Touch-LCD-1.85    (360x360 LCD)
//   - Waveshare ESP32-S3-Touch-AMOLED-1.75 (466x466 AMOLED, not written yet)
//
// The board is chosen by the board selected for the build (the FQBN), so
// there's no switch to edit: each board's constants (size, pins, ...) are in
// board_<name>.h, and its start-up, touch, backlight and power code in
// board_<name>.cpp. The motion sensor, the fast frame path and the buttons
// are shared (NecklaceBoard.cpp).
//
//   Arduino_GFX *gfx = NecklaceBoard::begin();   // display ready, backlight 100%, touch ready
//   NecklaceBoard::setBacklight(10);             // 0..NecklaceBoard::BACKLIGHT_MAX
//   if (NecklaceBoard::isTouched()) { ... }
#pragma once
#include <Arduino_GFX_Library.h>

#if defined(ARDUINO_WAVESHARE_ESP32_S3_TOUCH_LCD_185)
#define NECKLACE_BOARD_LCD185 1
#include "board_lcd185.h"
#elif defined(ARDUINO_WAVESHARE_ESP32_S3_TOUCH_AMOLED_143)  // stands in for the 1.75 (no core entry)
#define NECKLACE_BOARD_AMOLED175 1
#include "board_amoled175.h"
#else
#error "NecklaceBoard: select the Waveshare ESP32-S3-Touch-LCD-1.85 (or, for the AMOLED 1.75, the AMOLED-1.43) board"
#endif

namespace NecklaceBoard {

// Brings up I2C buses, resets the panel and touch, starts the display
// (cleared to black), turns the backlight fully on and prepares touch and the
// motion sensor. Returns the display, or nullptr on failure.
Arduino_GFX *begin();

// Panel revision, for boards that ship with more than one (the 1.85" has two
// ST77916 panels needing different init sequences): 1 or 2. Otherwise 1.
int panelRevision();

// Screen brightness, 0 (off) .. BACKLIGHT_MAX (full).
void setBacklight(uint16_t duty);

// True while a finger is on the screen.
bool isTouched();

// Reads the touch controller: true while a finger is down, with its
// position in screen pixels (0..WIDTH-1, 0..HEIGHT-1).
bool readTouch(int16_t &x, int16_t &y);

// ---- Buttons and power ----
// Cuts battery power. On USB power the board keeps running.
void powerOff();

// A push button (active low). Presses are caught by a GPIO interrupt, so a
// quick tap is never missed, however long a frame takes.
class Button {
 public:
  explicit Button(int pin) : pin_(pin) {}
  // Starts watching the pin. Call once, after NecklaceBoard::begin().
  void begin();
  // True once for each press (debounced).
  bool pressed();
  // True while the button has been held for at least ms milliseconds.
  // Returns true only once per hold.
  bool heldFor(uint32_t ms);

 private:
  static void IRAM_ATTR onEdge(void *arg);
  int pin_;
  volatile uint32_t pressCount_ = 0;
  volatile uint32_t lastEdgeMs_ = 0;   // any edge, press or release
  volatile uint32_t pressedAtMs_ = 0;
  uint32_t seenCount_ = 0;
  uint32_t holdHandledCount_ = UINT32_MAX;
};

// ---- Fast frame output ----
// Frames are rendered in horizontal strips of STRIP_ROWS rows, into buffers in
// fast internal RAM. Each strip is sent to the panel by DMA while the CPU
// renders the next one, so rendering and sending overlap.
//
// Note: after using these, don't draw on the panel through the Arduino_GFX
// object returned by begin(); its cached address window would be stale.
constexpr int16_t STRIP_ROWS = 20;  // 360 x 20 x 2 bytes = 14,400 bytes per strip

// Fills `pixels` (rows x WIDTH, row-major) for screen rows y0 .. y0 + rows - 1.
// Pixels are RGB565 in the panel's byte order: convert with panelColor().
using StripRenderer = void (*)(uint16_t *pixels, int16_t y0, int16_t rows, void *ctx);

// Renders and sends screen rows y0 .. y1 - 1 (full width).
void drawRows(int16_t y0, int16_t y1, StripRenderer render, void *ctx = nullptr);
inline void drawFrame(StripRenderer render, void *ctx = nullptr) { drawRows(0, HEIGHT, render, ctx); }

// Fills the whole screen with one colour (normal RGB565).
void clearScreen(uint16_t rgb565 = 0);

// Normal RGB565 -> the panel's byte order (big-endian).
constexpr uint16_t panelColor(uint16_t rgb565) { return (uint16_t)((rgb565 >> 8) | (rgb565 << 8)); }

// ---- Motion (QMI8658 6-axis IMU) ----
// Raw sensor axes, as the chip reports them. How they map onto the screen
// is decided by the sketch.
struct Motion {
  float ax, ay, az;  // acceleration in g (gravity alone reads 1 g)
  float gx, gy, gz;  // rotation rate in degrees per second
};

// True if the IMU answered during begin().
bool imuReady();

// Reads the latest accelerometer and gyro sample (250 Hz). Returns false on an I2C error.
// Don't call this once the motion task is running (it owns the I2C reads).
bool readMotion(Motion &m);

// Starts a background task on core 0 that reads the IMU at its full 250 Hz,
// so short shocks between frames aren't missed. Returns false if the IMU
// isn't available.
bool startMotionTask();

// Latest sample from the motion task, plus the biggest jolt since the
// previous call: how far the acceleration jumped away from its recent
// average, in g, over all three axes. Returns false if no sample yet.
bool latestMotion(Motion &m, float &peakJolt);

}  // namespace NecklaceBoard
