// Hello World test for the wave necklace board
// (Waveshare ESP32-S3-Touch-LCD-1.85, via the NecklaceBoard library)
//
// - "Hello world!" cycling through rainbow colours
// - a dot stepping around the edge once per second (60 steps = 1 lap)
// - tap the screen to cycle the backlight: 100% -> dimmest -> ... -> 100%

#include <NecklaceBoard.h>
#include "FreeSansBold18pt7b.h"

Arduino_GFX *gfx = nullptr;

// ---- Layout ----
const char *MESSAGE = "Hello world!";
const int16_t CX = NecklaceBoard::WIDTH / 2;
const int16_t CY = NecklaceBoard::HEIGHT / 2;
const int16_t DOT_ORBIT = 166;  // distance of dot from centre
const int16_t DOT_RADIUS = 7;
const uint8_t DOT_STEPS = 60;   // positions per lap
const uint16_t DOT_COLOR = RGB565_WHITE;

// ---- Backlight levels ----
// Spaced evenly in *perceived* brightness (gamma 2.2), from the dimmest
// (good for a dark room) to fully on.
const uint16_t BL_LEVELS[] = {10, 147, 479, NecklaceBoard::BACKLIGHT_MAX};
const uint8_t BL_LEVEL_COUNT = sizeof(BL_LEVELS) / sizeof(BL_LEVELS[0]);

uint8_t blLevel = BL_LEVEL_COUNT - 1;  // start at 100%, first tap goes to dimmest
bool wasTouched = false;
uint8_t dotIndex = 0;
uint32_t lastDotMove = 0;
uint16_t hue = 0;  // 0..359
int16_t textX, textY;

// HSV (s = v = 1) to RGB565
uint16_t hueToRGB565(uint16_t h) {
  uint8_t region = h / 60;
  uint8_t rem = (h % 60) * 255 / 60;
  uint8_t r, g, b;
  switch (region) {
    case 0:  r = 255;       g = rem;       b = 0;         break;
    case 1:  r = 255 - rem; g = 255;       b = 0;         break;
    case 2:  r = 0;         g = 255;       b = rem;       break;
    case 3:  r = 0;         g = 255 - rem; b = 255;       break;
    case 4:  r = rem;       g = 0;         b = 255;       break;
    default: r = 255;       g = 0;         b = 255 - rem; break;
  }
  return gfx->color565(r, g, b);
}

void drawDot(uint8_t index, uint16_t color) {
  // Start at 12 o'clock and move clockwise
  float angle = (index * 2.0f * PI / DOT_STEPS) - PI / 2.0f;
  int16_t x = CX + (int16_t)roundf(cosf(angle) * DOT_ORBIT);
  int16_t y = CY + (int16_t)roundf(sinf(angle) * DOT_ORBIT);
  gfx->fillCircle(x, y, DOT_RADIUS, color);
}

void layoutMessage() {
  gfx->setFont(&FreeSansBold18pt7b);
  gfx->setTextSize(1);
  int16_t x1, y1;
  uint16_t w, h;
  gfx->getTextBounds(MESSAGE, 0, 0, &x1, &y1, &w, &h);
  textX = CX - (int16_t)w / 2 - x1;
  textY = CY - (int16_t)h / 2 - y1;
}

void drawMessage(uint16_t color) {
  // Custom fonts draw only the glyph pixels (no background). Since the text
  // never changes, redrawing in a new colour covers exactly the same pixels.
  gfx->setTextColor(color);
  gfx->setCursor(textX, textY);
  gfx->print(MESSAGE);
}

void setup() {
  Serial.begin(115200);
  delay(1500);  // give the USB serial monitor a moment to connect

  gfx = NecklaceBoard::begin();
  if (!gfx) {
    while (true) delay(1000);
  }
  NecklaceBoard::setBacklight(BL_LEVELS[blLevel]);

  layoutMessage();
  drawDot(dotIndex, DOT_COLOR);
  lastDotMove = millis();
}

void loop() {
  uint32_t now = millis();

  // A tap = finger goes down (holding doesn't repeat)
  bool touched = NecklaceBoard::isTouched();
  if (touched && !wasTouched) {
    blLevel = (blLevel + 1) % BL_LEVEL_COUNT;
    NecklaceBoard::setBacklight(BL_LEVELS[blLevel]);
    Serial.printf("Backlight level %d (duty %d)\n", blLevel, BL_LEVELS[blLevel]);
  }
  wasTouched = touched;

  if (now - lastDotMove >= 1000) {
    lastDotMove += 1000;
    drawDot(dotIndex, RGB565_BLACK);
    dotIndex = (dotIndex + 1) % DOT_STEPS;
    drawDot(dotIndex, DOT_COLOR);
  }

  drawMessage(hueToRGB565(hue));
  hue = (hue + 2) % 360;
  delay(20);
}
