// Ocean wave necklace: water sloshing inside a round "fish bowl" display,
// driven by the motion sensor.
// Board: Waveshare ESP32-S3-Touch-LCD-1.85 (via the NecklaceBoard library).
//
// Screens (BOOT button toggles):
// - Wave: the necklace itself (placeholder for now)
// - Diagnostics: raw accel/gyro, gravity direction and frame timing.
//   A blue dot on the rim marks "down" (where the water will pool),
//   a white dot marks "up" (where the sky will be).
// Hold PWR for 2 s to power off (when running on battery).
//
// Every frame is drawn into a framebuffer in PSRAM, then pushed to the panel.

#include <NecklaceBoard.h>
#include "FreeSans12pt7b.h"

using namespace NecklaceBoard;

Arduino_GFX *panel = nullptr;      // the physical display
Arduino_Canvas *frame = nullptr;   // 360x360 framebuffer we draw into

const int16_t CX = WIDTH / 2;
const int16_t CY = HEIGHT / 2;
const int16_t RIM = 168;           // radius of the direction markers

// Mapping from sensor axes to screen axes (screen x = right, y = down).
// Measured on the board: with the USB-C port at the bottom, gravity reads
// along sensor +X, so the sensor is rotated 90 degrees from the screen.
float screenGravityX(const Motion &m) { return -m.ay; }
float screenGravityY(const Motion &m) { return m.ax; }

// ---- Screens and buttons ----
enum class Screen { Wave, Diagnostics };
Screen screen = Screen::Diagnostics;  // start on diagnostics while developing
Button bootButton(PIN_BTN_BOOT);
Button pwrButton(PIN_BTN_PWR);
const uint32_t POWER_OFF_HOLD_MS = 2000;

// ---- Timing ----
uint32_t drawUs = 0, flushUs = 0, frameUs = 0;
float fpsSmoothed = 0;

void drawHud(const Motion &m, bool motionOk) {
  frame->setFont(&FreeSans12pt7b);
  frame->setTextColor(RGB565_WHITE);

  const int16_t left = CX - 110;
  int16_t y = CY - 95;
  const int16_t line = 28;
  char buf[48];

  if (!motionOk) {
    frame->setCursor(left, y);
    frame->print("IMU not responding");
    y += line;
  } else {
    snprintf(buf, sizeof(buf), "ax %+.2f   gx %+6.1f", m.ax, m.gx);
    frame->setCursor(left, y); frame->print(buf); y += line;
    snprintf(buf, sizeof(buf), "ay %+.2f   gy %+6.1f", m.ay, m.gy);
    frame->setCursor(left, y); frame->print(buf); y += line;
    snprintf(buf, sizeof(buf), "az %+.2f   gz %+6.1f", m.az, m.gz);
    frame->setCursor(left, y); frame->print(buf); y += line;

    float gx = screenGravityX(m), gy = screenGravityY(m);
    float tiltDeg = atan2f(gx, gy) * 180.0f / PI;  // 0 = straight down the screen
    float inPlane = sqrtf(gx * gx + gy * gy);       // ~1 g when the screen is vertical
    snprintf(buf, sizeof(buf), "tilt %+4.0f   |g| %.2f", tiltDeg, inPlane);
    frame->setCursor(left, y); frame->print(buf); y += line;
  }

  y += 8;
  frame->setTextColor(RGB565_YELLOW);
  snprintf(buf, sizeof(buf), "%.1f fps  (%.1f ms)", fpsSmoothed, frameUs / 1000.0f);
  frame->setCursor(left, y); frame->print(buf); y += line;
  snprintf(buf, sizeof(buf), "draw %.1f  flush %.1f ms", drawUs / 1000.0f, flushUs / 1000.0f);
  frame->setCursor(left, y); frame->print(buf);
}

void drawDirectionMarkers(const Motion &m) {
  float gx = screenGravityX(m), gy = screenGravityY(m);
  float len = sqrtf(gx * gx + gy * gy);
  if (len < 0.15f) return;  // screen nearly flat: no clear "down"
  gx /= len;
  gy /= len;
  frame->fillCircle(CX + gx * RIM, CY + gy * RIM, 9, RGB565_BLUE);   // down: water
  frame->fillCircle(CX - gx * RIM, CY - gy * RIM, 9, RGB565_WHITE);  // up: sky
}

void drawWavePlaceholder() {
  frame->setFont(&FreeSans12pt7b);
  frame->setTextColor(RGB565_CYAN);
  frame->setCursor(CX - 60, CY + 8);
  frame->print("ocean_wave");
  char buf[24];
  snprintf(buf, sizeof(buf), "%.0f fps", fpsSmoothed);
  frame->setTextColor(RGB565_DARKGREY);
  frame->setCursor(CX - 30, HEIGHT - 40);
  frame->print(buf);
}

void handleButtons() {
  if (bootButton.pressed()) {
    screen = (screen == Screen::Wave) ? Screen::Diagnostics : Screen::Wave;
  }
  pwrButton.pressed();  // short PWR press: not used yet, just discard it
  if (pwrButton.heldFor(POWER_OFF_HOLD_MS)) {
    panel->fillScreen(RGB565_BLACK);
    powerOff();  // on USB power the board keeps running; the screen just goes dark
    Serial.println("Power off requested");
  }
}

void setup() {
  Serial.begin(115200);

  panel = NecklaceBoard::begin();
  if (!panel) {
    while (true) delay(1000);
  }

  frame = new Arduino_Canvas(WIDTH, HEIGHT, panel);
  if (!frame->begin(GFX_SKIP_OUTPUT_BEGIN)) {  // panel is already running
    Serial.println("Framebuffer allocation failed");
    while (true) delay(1000);
  }
  Serial.printf("Framebuffer in %s\n",
                esp_ptr_external_ram(frame->getFramebuffer()) ? "PSRAM" : "internal RAM");

  NecklaceBoard::setBacklight(BACKLIGHT_MAX);  // full brightness while developing
  bootButton.begin();
  pwrButton.begin();
}

void loop() {
  static uint32_t lastFrameStart = 0;  // 0 = no previous frame yet
  static uint32_t lastLog = 0;
  uint32_t t0 = micros();

  handleButtons();

  Motion m = {};
  bool motionOk = readMotion(m);

  frame->fillScreen(RGB565_BLACK);
  if (screen == Screen::Diagnostics) {
    drawDirectionMarkers(m);
    drawHud(m, motionOk);
  } else {
    drawWavePlaceholder();
  }
  uint32_t t1 = micros();

  frame->flush();
  uint32_t t2 = micros();

  drawUs = t1 - t0;
  flushUs = t2 - t1;
  if (lastFrameStart != 0) {  // the first frame has no interval to measure
    frameUs = t0 - lastFrameStart;
    float fps = 1e6f / frameUs;
    fpsSmoothed = fpsSmoothed == 0 ? fps : fpsSmoothed * 0.9f + fps * 0.1f;
  }
  lastFrameStart = t0;

  if (millis() - lastLog >= 500) {
    lastLog = millis();
    Serial.printf("a %+.2f %+.2f %+.2f  g %+6.1f %+6.1f %+6.1f  | %.1f fps draw %.1f flush %.1f ms\n",
                  m.ax, m.ay, m.az, m.gx, m.gy, m.gz, fpsSmoothed, drawUs / 1000.0f, flushUs / 1000.0f);
  }
}
