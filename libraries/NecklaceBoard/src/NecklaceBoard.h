// Board support for the Waveshare ESP32-S3-Touch-LCD-1.85.
//
//   Arduino_GFX *gfx = NecklaceBoard::begin();   // display ready, backlight 100%, touch ready
//   NecklaceBoard::setBacklight(10);             // 0..NecklaceBoard::BACKLIGHT_MAX
//   if (NecklaceBoard::isTouched()) { ... }
#pragma once
#include <Arduino_GFX_Library.h>

namespace NecklaceBoard {

// ---- Display ----
constexpr int16_t WIDTH = 360;
constexpr int16_t HEIGHT = 360;

// ---- Pins (Waveshare wiki) ----
constexpr int PIN_LCD_CS = 21;
constexpr int PIN_LCD_SCK = 40;
constexpr int PIN_LCD_D0 = 46;
constexpr int PIN_LCD_D1 = 45;
constexpr int PIN_LCD_D2 = 42;
constexpr int PIN_LCD_D3 = 41;
constexpr int PIN_LCD_TE = 18;
constexpr int PIN_LCD_BL = 5;
constexpr int PIN_I2C_SDA = 11;  // Wire: TCA9554 expander, QMI8658 IMU, PCF85063 RTC
constexpr int PIN_I2C_SCL = 10;
constexpr int PIN_TP_SDA = 1;    // Wire1: CST816 touch
constexpr int PIN_TP_SCL = 3;
constexpr int PIN_TP_INT = 4;
constexpr int PIN_BTN_BOOT = 0;  // BOOT button, LOW when pressed (free after boot)
constexpr int PIN_BTN_PWR = 6;   // PWR button, LOW when pressed
constexpr int PIN_PWR_LATCH = 7; // HIGH keeps the board powered from the battery

// QSPI clock for the display. Waveshare's demo uses 80 MHz; a full
// 360x360 RGB565 frame then takes about 6.5 ms on the wire.
constexpr int32_t DISPLAY_SPI_HZ = 80000000;

// ---- Backlight ----
constexpr uint32_t BACKLIGHT_FREQ = 20000;
constexpr uint8_t BACKLIGHT_BITS = 10;
constexpr uint16_t BACKLIGHT_MAX = 1 << BACKLIGHT_BITS;  // fully on

// Brings up I2C buses, resets the panel and touch via the expander, detects
// the panel revision, starts the display (cleared to black), turns the
// backlight fully on and prepares touch. Returns the display, or nullptr on failure.
Arduino_GFX *begin();

// 1 or 2: which of Waveshare's two ST77916 panel revisions was detected.
int panelRevision();

// Backlight PWM duty, 0 (off) .. BACKLIGHT_MAX (fully on).
void setBacklight(uint16_t duty);

// True while a finger is on the screen.
bool isTouched();

// Reads the touch controller: true while a finger is down, with its
// position in screen pixels (0..WIDTH-1, 0..HEIGHT-1).
bool readTouch(int16_t &x, int16_t &y);

// ---- Buttons and power ----
// Cuts battery power (releases the power latch). On USB power the board keeps running.
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
