// PoolView: the bowl seen from above, when the necklace lies flat face up.
//
// ---- The physics ----
// The 2D wave equation on a grid of 4 px cells, stepped at a fixed rate:
//   next = h + (h - prev) * (1 - damping*dt) + K * (sum of 4 neighbours - 4h)
// K = (c * dt / cell)^2 sets the ripple speed c; it must stay below 0.5 or
// the grid becomes unstable. Cells outside the circle hold 0, so ripples
// reflect off the rim (upside down, as from a fixed edge).
//
// Ripples come from:
//   - a finger: one drop where it lands
//   - a sharp jab: a ring in from the whole rim that focuses at the centre
//   - lava only: a gas bubble now and then (Material::poolBloop)
// (No random drops in water: the user found taps and jabs entertaining
// enough, and still water in a still bowl is right.)
//
// ---- The look ----
// The resting colour is exactly what the side view shows when the bowl is
// full: light near "up", deepening to navy towards "down". The necklace is
// never perfectly flat, so "down" follows whichever way it tips (heavily
// smoothed): the light area drifts around the top as it's tilted. That way
// the hand-over from the full side view is seamless.
// On top of that, water shows its ripples through light: a slope facing the
// light (upper left) is brighter, one facing away darker, and steep lit
// slopes run into white glints.
//
// Rendering interpolates the shade between cell centres for every pixel
// (bilinear, in 8.8 fixed point), then dithers like the side view against
// RGB565 banding.
#pragma GCC optimize("O3")
#include "PoolView.h"

using namespace NecklaceBoard;

namespace {

// ---- The bowl ----
// The fluid's tuning (ripple speed, damping, lighting, tap and jab strength,
// colours) is in Material.h / Material.cpp.
const float STEP_RATE = 120.0f;       // physics steps per second
const float RING_JOLT = 0.6f;         // g: a jab harder than this rings the whole rim
// Gas bubbles (lava only, Material::poolBloop): now and then, every
// Material::dropInterval on average, the surface lifts at a random spot.
const float SETTLE_TIME = 1.5f;       // s after the pool appears: ignore bumps (it's being set down)
const float TOUCH_QUIET = 0.3f;       // s after a finger lands: ignore bumps (the tap shook the board)

const float DOWN_SMOOTHING = 1.0f;    // s: how slowly "down" follows the tip when flat
const float MIN_TIP = 0.01f;          // g: flatter than this, keep the last "down"
const float TURN_RATE = 1.5f;         // rad/s: fastest the light area swings round (about 90 deg/s)

// ---- Colours, built from the material ----
// The middle of the palette (DEEP..SURFACE) is the side view's depth ramp,
// from deep to the colour just under the surface. Below it, shadow; above
// it, glints.
const int PALETTE = 128;
const int DEEP = 16, SURFACE = 111;   // palette range of the side view's colours
uint16_t palette[2][PALETTE];         // two dither phases
// Must match SideView's full level (195 px above centre).
const float FULL_LEVEL = 195.0f;

uint16_t rgb565(float r, float g, float b) {  // components 0..255
  auto c = [](float v, int maxv) { int i = (int)(v + 0.5f); return i < 0 ? 0 : (i > maxv ? maxv : i); };
  return (uint16_t)((c(r * 31 / 255, 31) << 11) | (c(g * 63 / 255, 63) << 5) | c(b * 31 / 255, 31));
}

void buildPalette(const Material &m) {
  for (int i = 0; i < PALETTE; i++) {
    float r, g, b;
    if (i < DEEP) {  // shadow: darker than the deepest water
      float t = (float)i / DEEP;
      r = m.poolShadow.r + (m.deep.r - m.poolShadow.r) * t;
      g = m.poolShadow.g + (m.deep.g - m.poolShadow.g) * t;
      b = m.poolShadow.b + (m.deep.b - m.poolShadow.b) * t;
    } else if (i <= SURFACE) {  // SideView's depth colours (near = 0 deep .. 1 surface)
      float near = (float)(i - DEEP) / (SURFACE - DEEP);
      r = m.deep.r + (m.surface.r - m.deep.r) * powf(near, m.curve.r);
      g = m.deep.g + (m.surface.g - m.deep.g) * powf(near, m.curve.g);
      b = m.deep.b + (m.surface.b - m.deep.b) * powf(near, m.curve.b);
    } else {  // glints: surface -> glint
      float t = (float)(i - SURFACE) / (PALETTE - 1 - SURFACE);
      r = m.surface.r + (m.poolGlint.r - m.surface.r) * t;
      g = m.surface.g + (m.poolGlint.g - m.surface.g) * t;
      b = m.surface.b + (m.poolGlint.b - m.surface.b) * t;
    }
    for (int ph = 0; ph < 2; ph++) {
      float off = ph ? 2.0f : -2.0f;  // about +-1/4 of an RGB565 step, in 0..255 units
      palette[ph][i] = panelColor(rgb565(r + off, g + off, b + off));
    }
  }
}

uint32_t rngState = 0x9E3779B9;
float random01() {
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return (rngState & 0xFFFFFF) / 16777216.0f;
}

}  // namespace

void PoolView::setMaterial(const Material &m) {
  m_ = m;
  float courant = m.rippleSpeed / STEP_RATE / CELL;
  k_ = courant * courant;  // must stay below 0.5 (ripple speed below ~680 px/s)
  buildPalette(m);
}

void PoolView::begin() {
  setMaterial(WATER);
  for (int j = 0; j < GRID; j++) {
    for (int i = 0; i < GRID; i++) {
      float dx = i - CENTRE, dy = j - CENTRE;
      inside_[j][i] = dx * dx + dy * dy <= RADIUS_CELLS * RADIUS_CELLS;
    }
  }
  reset();
}

void PoolView::reset() {
  for (int j = 0; j < GRID; j++)
    for (int i = 0; i < GRID; i++) h_[j][i] = prev_[j][i] = 0;
  settleTimer_ = SETTLE_TIME;
  bloopTimer_ = m_.dropInterval;
}

void PoolView::drop(float cx, float cy, float radius, float strength) {
  // A smooth bump (cosine shaped), so it doesn't excite grid-sized noise.
  int r = (int)ceilf(radius);
  for (int j = (int)cy - r; j <= (int)cy + r + 1; j++) {
    for (int i = (int)cx - r; i <= (int)cx + r + 1; i++) {
      if (i < 0 || j < 0 || i >= GRID || j >= GRID || !inside_[j][i]) continue;
      float d = sqrtf((i - cx) * (i - cx) + (j - cy) * (j - cy)) / radius;
      if (d >= 1) continue;
      h_[j][i] -= strength * 0.5f * (1 + cosf(PI * d));  // pushed down, like a drop landing
    }
  }
}

void PoolView::ring(float strength) {
  // Lift a band just inside the rim: it runs inward as a ring and focuses at the centre.
  for (int j = 0; j < GRID; j++) {
    for (int i = 0; i < GRID; i++) {
      if (!inside_[j][i]) continue;
      float dx = i - CENTRE, dy = j - CENTRE;
      float d = (RADIUS_CELLS - sqrtf(dx * dx + dy * dy)) / 3.0f;  // 0 at the rim
      if (d < 1) h_[j][i] += strength * 0.5f * (1 + cosf(PI * d));
    }
  }
}

void PoolView::touch(int16_t x, int16_t y) {
  float cx = (float)x / CELL, cy = (float)y / CELL;
  // One ripple where the finger lands (a held or dragged finger adds nothing:
  // single ripples invite more taps to watch them cross).
  if (!touching_) {
    drop(cx, cy, 3.0f, m_.touchDrop);
    touchQuiet_ = TOUCH_QUIET;
    pendingJolt_ = 0;  // the jolt held back from last frame was this tap
  }
  touching_ = true;
}

void PoolView::setDown(float x, float y) {
  float n = sqrtf(x * x + y * y);
  if (n < 1e-3f) return;
  downX_ = x / n;
  downY_ = y / n;
  tipX_ = downX_ * MIN_TIP;
  tipY_ = downY_ * MIN_TIP;
}

void PoolView::update(float dt, float jolt, float gx, float gy) {
  if (dt > 0.05f) dt = 0.05f;

  // "Down" from the slight tip of a nearly flat necklace: smooth the in-plane
  // gravity a lot (it's tiny and noisy), and only steer by it when it's
  // clearly bigger than noise.
  float a = 1.0f - expf(-dt / DOWN_SMOOTHING);
  tipX_ += (gx - tipX_) * a;
  tipY_ += (gy - tipY_) * a;
  // Turn "down" towards the tip at a limited speed. Tipping the other way
  // passes through "exactly flat", where the direction is lost; without the
  // limit, the light area would freeze there and then jump across.
  float tip = sqrtf(tipX_ * tipX_ + tipY_ * tipY_);
  if (tip > MIN_TIP) {
    float tx = tipX_ / tip, ty = tipY_ / tip;
    float angle = atan2f(downX_ * ty - downY_ * tx, downX_ * tx + downY_ * ty);  // signed, -PI..PI
    float maxTurn = TURN_RATE * dt;
    angle = fminf(fmaxf(angle, -maxTurn), maxTurn);
    float c = cosf(angle), sn = sinf(angle);
    float nx = downX_ * c - downY_ * sn, ny = downX_ * sn + downY_ * c;
    float n = sqrtf(nx * nx + ny * ny);
    downX_ = nx / n;
    downY_ = ny / n;
  }

  // A tap on a hand-held necklace shakes it: that jolt is the tap, not a
  // bump. The touch chip is read once a frame, so the jolt can arrive a frame
  // before the touch does: hold each jolt back one frame (17 ms, invisible)
  // and drop it if a touch began meanwhile (see touch()).
  float j = pendingJolt_;
  pendingJolt_ = jolt;
  jolt = j;
  if (touching_ || touchQuiet_ > 0) {
    touchQuiet_ -= dt;
    jolt = pendingJolt_ = 0;
  }
  if (settleTimer_ > 0) {
    settleTimer_ -= dt;
    jolt = 0;  // still being set down: that bump isn't a splash
  }
  // Only a sharp jab disturbs the pool: smaller bumps come from handling it
  // (turning it on the table), and drops scattered by those looked random.
  if (jolt > RING_JOLT) ring(m_.ringGain * sqrtf(jolt));

  if (m_.poolBloop > 0) {
    bloopTimer_ -= dt;
    if (bloopTimer_ <= 0) {
      bloopTimer_ = m_.dropInterval * (0.3f + 1.4f * random01());
      float a = random01() * 2 * PI, r = sqrtf(random01()) * (RADIUS_CELLS - 6);
      drop(CENTRE + r * cosf(a), CENTRE + r * sinf(a), 2.0f, -m_.poolBloop * (0.6f + 0.8f * random01()));  // negative: lifts
    }
  }


  stepTimer_ += dt;
  const float stepDt = 1.0f / STEP_RATE;
  while (stepTimer_ >= stepDt) {
    step();
    stepTimer_ -= stepDt;
  }
  prepareRender();
}

void PoolView::step() {
  const float keep = 1 - m_.rippleDamping / STEP_RATE;
  // prev_ becomes next in place: next = h + (h - prev)*keep + K*laplacian.
  for (int j = 1; j < GRID - 1; j++) {
    for (int i = 1; i < GRID - 1; i++) {
      if (!inside_[j][i]) continue;
      float hc = h_[j][i];
      float lap = h_[j][i - 1] + h_[j][i + 1] + h_[j - 1][i] + h_[j + 1][i] - 4 * hc;
      prev_[j][i] = hc + (hc - prev_[j][i]) * keep + k_ * lap;
    }
  }
  // Swap the buffers (cheap enough at 91x91 to copy; keeps indexing simple).
  for (int j = 0; j < GRID; j++) {
    for (int i = 0; i < GRID; i++) {
      float t = h_[j][i];
      h_[j][i] = prev_[j][i];
      prev_[j][i] = t;
    }
  }
}

void PoolView::prepareRender() {
  const float gain = m_.lightGain * 256;
  // Resting shade per cell: the full side view's depth below the surface.
  // Height above the centre along "up" is v = -(offset . down).
  for (int j = 0; j < GRID; j++) {
    for (int i = 0; i < GRID; i++) {
      float v = -((i - CENTRE) * downX_ + (j - CENTRE) * downY_) * CELL;
      float near = 1 - fminf((FULL_LEVEL - v) / m_.depthRange, 1.0f);
      base_[j][i] = (int16_t)((DEEP + near * (SURFACE - DEEP)) * 256);
    }
  }
  for (int j = 0; j < GRID; j++) {
    for (int i = 0; i < GRID; i++) {
      int s = base_[j][i];
      if (inside_[j][i] && i > 0 && j > 0 && i < GRID - 1 && j < GRID - 1) {
        // Slope towards the light (upper left): rising towards the upper left = facing it.
        float slope = (h_[j][i - 1] - h_[j][i + 1]) + (h_[j - 1][i] - h_[j + 1][i]);
        s += (int)(slope * gain);
      }
      if (s < 0) s = 0;
      if (s > (PALETTE - 1) * 256) s = (PALETTE - 1) * 256;
      shade_[j][i] = (int16_t)s;
    }
  }
}

void PoolView::renderStrip(uint16_t *pixels, int16_t y0, int16_t rows, void *ctx) {
  static_cast<const PoolView *>(ctx)->render(pixels, y0, rows);
}

void PoolView::render(uint16_t *pixels, int16_t y0, int16_t rows) const {
  const float R = WIDTH / 2.0f;
  int32_t rowShade[GRID];
  for (int16_t r = 0; r < rows; r++) {
    int16_t y = y0 + r;
    uint16_t *row = pixels + (size_t)r * WIDTH;

    float dyc = y - HEIGHT / 2 + 0.5f;
    float half2 = R * R - dyc * dyc;
    int16_t half = half2 > 0 ? (int16_t)ceilf(sqrtf(half2)) : 0;
    int16_t xStart = WIDTH / 2 - half, xEnd = WIDTH / 2 + half;
    if (xStart < 0) xStart = 0;
    if (xEnd > WIDTH) xEnd = WIDTH;
    for (int16_t x = 0; x < xStart; x++) row[x] = 0;
    for (int16_t x = xEnd; x < WIDTH; x++) row[x] = 0;
    if (xStart >= xEnd) continue;

    // Blend the two grid rows around this pixel row (weight in 1/8ths: pixel
    // centres sit at 1/8, 3/8, 5/8, 7/8 of a 4 px cell).
    int j0 = y / CELL;
    int wy = 2 * (y - j0 * CELL) + 1;  // 1, 3, 5, 7
    int i0 = xStart / CELL, i1 = (xEnd - 1) / CELL + 1;
    if (i1 > GRID - 1) i1 = GRID - 1;
    for (int i = i0; i <= i1; i++)
      rowShade[i] = (shade_[j0][i] * (8 - wy) + shade_[j0 + 1][i] * wy) >> 3;

    const uint16_t *dither[2] = {palette[y & 1], palette[(y & 1) ^ 1]};
    for (int16_t x = xStart; x < xEnd; x++) {
      int i = x / CELL;
      int wx = 2 * (x - i * CELL) + 1;
      int32_t s = (rowShade[i] * (8 - wx) + rowShade[i + 1] * wx) >> 3;
      row[x] = dither[x & 1][s >> 8];
    }
  }
}
