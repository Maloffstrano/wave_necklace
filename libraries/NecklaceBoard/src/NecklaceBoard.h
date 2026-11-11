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

}  // namespace NecklaceBoard
