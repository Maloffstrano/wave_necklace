// Board constants: Waveshare ESP32-S3-Touch-LCD-1.85 (round 360x360 LCD, ST77916).
// Included by NecklaceBoard.h when this board is selected in the IDE / FQBN.
#pragma once
#include <stdint.h>

namespace NecklaceBoard {

constexpr const char *BOARD_NAME = "ESP32-S3-Touch-LCD-1.85";

// ---- Display ----
constexpr int16_t WIDTH = 360;
constexpr int16_t HEIGHT = 360;
constexpr int16_t COL_OFFSET = 0;  // first visible column/row in the controller's memory
constexpr int16_t ROW_OFFSET = 0;

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

// ---- Backlight (LEDC PWM on PIN_LCD_BL) ----
constexpr uint32_t BACKLIGHT_FREQ = 20000;
constexpr uint8_t BACKLIGHT_BITS = 10;
constexpr uint16_t BACKLIGHT_MAX = 1 << BACKLIGHT_BITS;  // fully on

}  // namespace NecklaceBoard
