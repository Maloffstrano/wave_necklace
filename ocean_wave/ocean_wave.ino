// Ocean wave necklace: an ocean wave that responds to necklace movement.
// Board: Waveshare ESP32-S3-Touch-LCD-1.85 (via the NecklaceBoard library).
//
// Starting point only: brings the board up and shows a placeholder.

#include <NecklaceBoard.h>

Arduino_GFX *gfx = nullptr;

void setup() {
  Serial.begin(115200);

  gfx = NecklaceBoard::begin();
  if (!gfx) {
    while (true) delay(1000);
  }
  NecklaceBoard::setBacklight(10);  // dimmest, comfortable in a dark room

  gfx->setTextColor(RGB565_CYAN);
  gfx->setTextSize(2);
  gfx->setCursor(NecklaceBoard::WIDTH / 2 - 6 * 2 * 5, NecklaceBoard::HEIGHT / 2 - 8);
  gfx->print("ocean_wave");
}

void loop() {
}
