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
// Face down: the screen goes dark until the necklace is picked up again.
// Flat face up: the top-down pool (PoolView.cpp); touch it to make ripples.
// Touch and hold (anywhere): the backlight drops to dimmest and ramps up
// toward full; release to keep that brightness. A quick tap is left free
// (in the pool it makes a ripple).

#include <NecklaceBoard.h>
#include "FreeSans12pt7b.h"
#include "SideView.h"
#include "PoolView.h"

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

// The two views of the water (see SideView.cpp and PoolView.cpp).
SideView water;
PoolView pool;

// ---- Orientation ----
// Lean averaged over about half a second, so a quick swing or a bump doesn't
// flip the mode. Face down (lean near -90) blanks the screen; it takes a clear
// lift to wake it, so it doesn't flicker on and off at the threshold.
const float LEAN_SMOOTHING = 0.5f;    // s
const float FACE_DOWN_ENTER = -70.0f; // degrees: blank below this...
const float FACE_DOWN_EXIT = -50.0f;  // ...and wake above this
const float POOL_ENTER = 80.0f;       // degrees: flat enough for the top-down pool...
const float POOL_EXIT = 72.0f;        // ...and back to the side view below this
float leanSmoothed = 0;
bool faceDown = false;
bool poolMode = false;

// Brightness by touch and hold. The ramp is exponential (each step
// multiplies the brightness), which is how the eye judges it; a linear ramp
// would look "already bright" after the first moment. The dimmest suits a dark room.
const uint32_t DIM_HOLD_MS = 600;     // hold this long to start adjusting (drops to dimmest)
const uint32_t DIM_PAUSE_MS = 800;    // then stay at dimmest this long, so it's easy to pick
const float DIM_RAMP_S = 2.5f;        // dimmest -> full
const uint16_t BRIGHTNESS_MIN = 10;
uint16_t brightness = BACKLIGHT_MAX;

void updateOrientation(const Motion &m, float dt) {
  float a = 1.0f - expf(-dt / LEAN_SMOOTHING);
  leanSmoothed += (screenLeanDeg(m) - leanSmoothed) * a;
  if (!faceDown && leanSmoothed < FACE_DOWN_ENTER) {
    faceDown = true;
    setBacklight(0);
    clearScreen(RGB565_BLACK);
  } else if (faceDown && leanSmoothed > FACE_DOWN_EXIT) {
    faceDown = false;
    setBacklight(brightness);
  }
  if (!poolMode && leanSmoothed > POOL_ENTER) {
    poolMode = true;
    pool.reset();  // fresh, calm water (the side view's was nearly still at this lean)
    pool.setDown(water.downX(), water.downY());  // carry on the side view's colours
  }
  else if (poolMode && leanSmoothed < POOL_EXIT) poolMode = false;
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
    snprintf(buf, sizeof(buf), "roll %+4.0f  lean %+3.0f (%+3.0f)", rollDeg, leanDeg, leanSmoothed);
    hud->setCursor(left, y); hud->print(buf); y += line;
    snprintf(buf, sizeof(buf), "jolt %.2f g  %s", peakJoltShown, poolMode ? "pool" : "side");
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

const int16_t LABEL_W = 230, LABEL_H = 26;  // fits inside the circle at this height
const int16_t LABEL_X = CX - LABEL_W / 2, LABEL_Y = HEIGHT - 70;
bool showFpsLabel = true;

void renderWaveStrip(uint16_t *pixels, int16_t y0, int16_t rows, void *) {
  uint32_t start = micros();
  if (poolMode) PoolView::renderStrip(pixels, y0, rows, &pool);
  else SideView::renderStrip(pixels, y0, rows, &water);
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
  char buf[40];
  label->fillScreen(RGB565_BLACK);
  label->setFont(&FreeSans12pt7b);
  label->setTextColor(RGB565_YELLOW);
  // render = CPU time spent drawing this frame's pixels (not waiting on DMA)
  snprintf(buf, sizeof(buf), "%.0f fps  render %.1fms", fpsSmoothed, renderCpuUs / 1000.0f);
  label->setCursor(4, LABEL_H - 7);
  label->print(buf);
}

void waveFrame(const Motion &m, bool motionOk, float jolt, uint32_t t0) {
  static uint32_t lastUpdate = 0;
  float dt = lastUpdate ? (t0 - lastUpdate) / 1e6f : 0.0f;
  lastUpdate = t0;
  water.setLean(leanSmoothed);
  if (poolMode) {
    int16_t tx, ty;
    if (touchDown(tx, ty)) pool.touch(tx, ty);
    else pool.release();
    pool.update(dt, jolt, screenGravityX(m), screenGravityY(m));
  } else if (motionOk) {
    water.update(screenGravityX(m), screenGravityY(m), dt, jolt);
  }
  if (showFpsLabel) updateFpsLabel();

  uint32_t t1 = micros();
  renderCpuUs = 0;
  drawFrame(renderWaveStrip);
  uint32_t t2 = micros();
  drawUs = t1 - t0;
  flushUs = t2 - t1;  // render + send, overlapped
}

// ============================================================================
// Touch: read once per frame. In the pool a finger makes ripples; elsewhere a
// tap (finger landing) steps the backlight.
// ============================================================================

bool fingerDown = false;
int16_t fingerX = 0, fingerY = 0;
uint32_t fingerDownMs = 0;

void readFinger() {
  bool was = fingerDown;
  fingerDown = readTouch(fingerX, fingerY);
  if (!fingerDown) return;
  if (!was) fingerDownMs = millis();
  uint32_t held = millis() - fingerDownMs;
  if (held < DIM_HOLD_MS) return;
  // Adjusting: dimmest for DIM_PAUSE_MS, then ramp to full over DIM_RAMP_S.
  uint32_t ramp = held - DIM_HOLD_MS;
  float t = ramp < DIM_PAUSE_MS ? 0.0f : fminf((ramp - DIM_PAUSE_MS) / 1000.0f / DIM_RAMP_S, 1.0f);
  brightness = (uint16_t)(BRIGHTNESS_MIN * powf((float)BACKLIGHT_MAX / BRIGHTNESS_MIN, t) + 0.5f);
  setBacklight(brightness);
}

bool touchDown(int16_t &x, int16_t &y) {
  x = fingerX;
  y = fingerY;
  return fingerDown;
}

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
  pool.begin();

  NecklaceBoard::setBacklight(brightness);
  bootButton.begin();
  pwrButton.begin();
  startMotionTask();  // reads the IMU at 250 Hz on core 0
}

void loop() {
  static uint32_t lastFrameStart = 0;  // 0 = no previous frame yet
  static uint32_t lastLog = 0;
  uint32_t t0 = micros();

  handleButtons();
  readFinger();

  Motion m = {};
  float jolt = 0;
  bool motionOk = latestMotion(m, jolt);
  if (jolt > peakJoltShown) peakJoltShown = jolt;

  static uint32_t lastOrientation = 0;
  if (motionOk) updateOrientation(m, lastOrientation ? (t0 - lastOrientation) / 1e6f : 0.0f);
  lastOrientation = t0;

  if (faceDown) {
    // Dark and idle: skip drawing, check the sensor ~20 times a second.
    // The water simply resumes when the necklace is picked up.
    lastFrameStart = 0;  // don't count the pause in the FPS
    delay(50);
    return;
  }

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
