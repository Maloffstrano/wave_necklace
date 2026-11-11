// Ocean wave necklace: water sloshing inside a round "fish bowl" display,
// driven by the motion sensor.
// Board: Waveshare ESP32-S3-Touch-LCD-1.85 (via the NecklaceBoard library).
//
// Screens (BOOT button toggles):
// - Wave: the necklace itself: water sloshing in the bowl (SideView.cpp).
// - Diagnostics: raw accel/gyro, gravity direction and frame timing.
//   A blue dot on the rim marks "down" (where the water will pool),
//   a white dot marks "up" (where the sky will be).
// Short PWR press: calm the water. Hold PWR for 2 s to power off (on battery).

#include <NecklaceBoard.h>
#include "FreeSans12pt7b.h"
#include "SideView.h"

using namespace NecklaceBoard;

Arduino_GFX *panel = nullptr;     // the physical display (used only during setup)
Arduino_Canvas *hud = nullptr;    // 360x360 text framebuffer for the diagnostics screen (PSRAM)
Arduino_Canvas *label = nullptr;  // small overlay for the wave screen's FPS label

const int16_t CX = WIDTH / 2;
const int16_t CY = HEIGHT / 2;
const int16_t RIM = 168;  // radius of the direction markers

// Mapping from sensor axes to screen axes (screen x = right, y = down).
// Measured on the board: with the USB-C port at the bottom, gravity reads
// along sensor +X, so the sensor is rotated 90 degrees from the screen.
float screenGravityX(const Motion &m) { return -m.ay; }
float screenGravityY(const Motion &m) { return m.ax; }

// Tilt of the screen away from vertical, in degrees:
// 0 = hanging upright, +90 = flat face up, -90 = flat face down.
// Measured: face down reads az = +1 g, so face up is az = -1 g.
float screenLeanDeg(const Motion &m) {
  float len = sqrtf(m.ax * m.ax + m.ay * m.ay + m.az * m.az);
  if (len < 0.2f) return 0;  // free fall or a bad reading: no meaningful answer
  float s = fminf(fmaxf(-m.az / len, -1.0f), 1.0f);
  return asinf(s) * 180.0f / PI;
}

// ---- Screens and buttons ----
enum class Screen { Wave, Diagnostics };
Screen screen = Screen::Wave;
Button bootButton(PIN_BTN_BOOT);
Button pwrButton(PIN_BTN_PWR);
const uint32_t POWER_OFF_HOLD_MS = 2000;

// ---- Timing ----
uint32_t drawUs = 0, flushUs = 0, frameUs = 0;
uint32_t renderCpuUs = 0;  // wave screen: CPU time spent inside the strip renderer
float fpsSmoothed = 0;
float peakJoltShown = 0;  // diagnostics: biggest jolt recently (decays)

// ============================================================================
// Diagnostics screen: text drawn with Arduino_GFX into a PSRAM canvas, then
// sent through the fast strip pipeline (copy + byte swap per strip).
// ============================================================================

void drawHud(const Motion &m, bool motionOk) {
  hud->setFont(&FreeSans12pt7b);
  hud->setTextColor(RGB565_WHITE);

  const int16_t left = CX - 110;
  int16_t y = CY - 95;
  const int16_t line = 28;
  char buf[48];

  if (!motionOk) {
    hud->setCursor(left, y);
    hud->print("IMU not responding");
    y += line;
  } else {
    snprintf(buf, sizeof(buf), "ax %+.2f   gx %+6.1f", m.ax, m.gx);
    hud->setCursor(left, y); hud->print(buf); y += line;
    snprintf(buf, sizeof(buf), "ay %+.2f   gy %+6.1f", m.ay, m.gy);
    hud->setCursor(left, y); hud->print(buf); y += line;
    snprintf(buf, sizeof(buf), "az %+.2f   gz %+6.1f", m.az, m.gz);
    hud->setCursor(left, y); hud->print(buf); y += line;

    float gx = screenGravityX(m), gy = screenGravityY(m);
    float rollDeg = atan2f(gx, gy) * 180.0f / PI;  // rotation in the screen plane, 0 = USB down
    float leanDeg = screenLeanDeg(m);
    snprintf(buf, sizeof(buf), "roll %+4.0f   lean %+3.0f", rollDeg, leanDeg);
    hud->setCursor(left, y); hud->print(buf); y += line;
    snprintf(buf, sizeof(buf), "jolt %.2f g", peakJoltShown);
    hud->setCursor(left, y); hud->print(buf); y += line;
  }

  y += 8;
  hud->setTextColor(RGB565_YELLOW);
  snprintf(buf, sizeof(buf), "%.1f fps  (%.1f ms)", fpsSmoothed, frameUs / 1000.0f);
  hud->setCursor(left, y); hud->print(buf); y += line;
  snprintf(buf, sizeof(buf), "draw %.1f  flush %.1f ms", drawUs / 1000.0f, flushUs / 1000.0f);
  hud->setCursor(left, y); hud->print(buf);
}

void drawDirectionMarkers(const Motion &m) {
  float gx = screenGravityX(m), gy = screenGravityY(m);
  float len = sqrtf(gx * gx + gy * gy);
  if (len < 0.15f) return;  // screen nearly flat: no clear "down"
  gx /= len;
  gy /= len;
  hud->fillCircle(CX + gx * RIM, CY + gy * RIM, 9, RGB565_BLUE);   // down: water
  hud->fillCircle(CX - gx * RIM, CY - gy * RIM, 9, RGB565_WHITE);  // up: sky
}

// Copies rows from the canvas into a strip, swapping to the panel's byte order.
void copyCanvasStrip(uint16_t *pixels, int16_t y0, int16_t rows, void *ctx) {
  const uint16_t *src = static_cast<Arduino_Canvas *>(ctx)->getFramebuffer() + (size_t)y0 * WIDTH;
  for (size_t i = 0, n = (size_t)rows * WIDTH; i < n; i++) pixels[i] = panelColor(src[i]);
}

void diagnosticsFrame(const Motion &m, bool motionOk, uint32_t t0) {
  hud->fillScreen(RGB565_BLACK);
  drawDirectionMarkers(m);
  drawHud(m, motionOk);
  uint32_t t1 = micros();
  drawFrame(copyCanvasStrip, hud);
  uint32_t t2 = micros();
  drawUs = t1 - t0;
  flushUs = t2 - t1;
}

// ============================================================================
// Wave screen: the sloshing side view (SideView.cpp), rendered straight into
// the DMA strips, with a small FPS label laid over it.
// ============================================================================

SideView water;
const int16_t LABEL_W = 170, LABEL_H = 26;
const int16_t LABEL_X = CX - LABEL_W / 2, LABEL_Y = HEIGHT - 70;
bool showFpsLabel = true;

void renderWaveStrip(uint16_t *pixels, int16_t y0, int16_t rows, void *) {
  uint32_t start = micros();
  SideView::renderStrip(pixels, y0, rows, &water);
  if (showFpsLabel && y0 < LABEL_Y + LABEL_H && y0 + rows > LABEL_Y) {
    const uint16_t *labelPx = label->getFramebuffer();
    for (int16_t r = 0; r < rows; r++) {
      int16_t y = y0 + r;
      if (y < LABEL_Y || y >= LABEL_Y + LABEL_H) continue;
      const uint16_t *src = labelPx + (size_t)(y - LABEL_Y) * LABEL_W;
      uint16_t *row = pixels + (size_t)r * WIDTH + LABEL_X;
      for (int16_t i = 0; i < LABEL_W; i++) {
        if (src[i]) row[i] = panelColor(src[i]);
      }
    }
  }
  renderCpuUs += micros() - start;
}

void updateFpsLabel() {
  static uint32_t lastLabel = 0;
  if (millis() - lastLabel < 250) return;  // 4 updates a second is plenty
  lastLabel = millis();
  char buf[32];
  label->fillScreen(RGB565_BLACK);
  label->setFont(&FreeSans12pt7b);
  label->setTextColor(RGB565_YELLOW);
  snprintf(buf, sizeof(buf), "%.1f fps  cpu %.1f", fpsSmoothed, renderCpuUs / 1000.0f);
  label->setCursor(4, LABEL_H - 7);
  label->print(buf);
}

void waveFrame(const Motion &m, bool motionOk, float jolt, uint32_t t0) {
  static uint32_t lastUpdate = 0;
  float dt = lastUpdate ? (t0 - lastUpdate) / 1e6f : 0.0f;
  lastUpdate = t0;
  if (motionOk) water.update(screenGravityX(m), screenGravityY(m), dt, jolt);
  if (showFpsLabel) updateFpsLabel();

  uint32_t t1 = micros();
  renderCpuUs = 0;
  drawFrame(renderWaveStrip);
  uint32_t t2 = micros();
  drawUs = t1 - t0;
  flushUs = t2 - t1;  // render + send, overlapped
}

// ============================================================================

void handleButtons() {
  if (bootButton.pressed()) {
    screen = (screen == Screen::Wave) ? Screen::Diagnostics : Screen::Wave;
  }
  if (pwrButton.pressed()) water.reset();  // short PWR press: calm the water
  if (pwrButton.heldFor(POWER_OFF_HOLD_MS)) {
    clearScreen(RGB565_BLACK);
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

  hud = new Arduino_Canvas(WIDTH, HEIGHT, nullptr);
  label = new Arduino_Canvas(LABEL_W, LABEL_H, nullptr);
  if (!hud->begin(GFX_SKIP_OUTPUT_BEGIN) || !label->begin(GFX_SKIP_OUTPUT_BEGIN)) {
    Serial.println("Canvas allocation failed");
    while (true) delay(1000);
  }
  water.begin();

  NecklaceBoard::setBacklight(BACKLIGHT_MAX);  // full brightness while developing
  bootButton.begin();
  pwrButton.begin();
  startMotionTask();  // reads the IMU at 250 Hz on core 0
}

void loop() {
  static uint32_t lastFrameStart = 0;  // 0 = no previous frame yet
  static uint32_t lastLog = 0;
  uint32_t t0 = micros();

  handleButtons();

  Motion m = {};
  float jolt = 0;
  bool motionOk = latestMotion(m, jolt);
  if (jolt > peakJoltShown) peakJoltShown = jolt;

  if (screen == Screen::Diagnostics) {
    diagnosticsFrame(m, motionOk, t0);
  } else {
    waveFrame(m, motionOk, jolt, t0);
  }

  peakJoltShown *= 0.97f;  // let the diagnostics readout fade over about a second

  if (lastFrameStart != 0) {  // the first frame has no interval to measure
    frameUs = t0 - lastFrameStart;
    float fps = 1e6f / frameUs;
    fpsSmoothed = fpsSmoothed == 0 ? fps : fpsSmoothed * 0.9f + fps * 0.1f;
  }
  lastFrameStart = t0;

  if (millis() - lastLog >= 500) {
    lastLog = millis();
    Serial.printf("%s | %.1f fps  draw %.1f  flush %.1f  render-cpu %.1f ms\n",
                  screen == Screen::Wave ? "wave" : "diag", fpsSmoothed,
                  drawUs / 1000.0f, flushUs / 1000.0f, renderCpuUs / 1000.0f);
  }
}
