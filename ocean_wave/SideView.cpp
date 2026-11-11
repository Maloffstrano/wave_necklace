// SideView: water sloshing in the round "fish bowl", seen from the side.
//
// ---- The physics ----
// Linearized 1D shallow-water equations on a staggered grid: the standard
// approach for sloshing in a container.
//   - 91 cells across the bowl, 4 px apart, each storing the surface height h
//   - between neighbouring cells sits a horizontal flow velocity q, fixed at
//     zero at both walls
//   - each substep:  q += dt * (-G * dh/du + a_side)   flow speeds up downhill
//                                                      and with a sideways push
//                    h -= dt * DEPTH * dq/du           water moves between cells
// Volume is conserved exactly, waves travel at c = sqrt(G * DEPTH) and
// reflect off the walls, and a steady sideways push settles into a tilted
// surface. That's sloshing, with no special cases.
//
// ---- Where the sloshing comes from ----
// The water is drawn in a frame aligned with *smoothed* gravity (averaged
// over about 0.35 s). The difference between the raw accelerometer reading
// and the smoothed direction is a_side. Rotate the necklace quickly, or jolt
// it sideways, and the raw "down" runs ahead of the smoothed one, so the
// water gets pushed toward the new low side and sloshes until it settles.
//
// ---- Rendering ----
// For each pixel we need its position across the bowl (u) and its height (v)
// in the water frame. Both are linear along a screen row, so each pixel costs
// two fixed-point additions. Each frame, the surface height is precomputed
// for every 1 px step of u (interpolated from the physics cells), so the
// pixel loop is just: look up the surface, subtract v to get the depth, then
// pick sky, foam, or a colour by depth. Foam builds up per cell where the flow
// is fast or a crest is sharp, then fades. Only the visible circle is
// rendered; the corners of the 360x360 square are filled with black.

// Pixel loops: Arduino builds with -Os (small code), which is slow for tight
// inner loops. Optimise this file for speed instead.
#pragma GCC optimize("O3")

#include "SideView.h"

using namespace NecklaceBoard;

namespace {

// ---- Tuning ----
const float REST_LEVEL = -14.0f;      // rest surface height (px above the bowl centre): about 45% full
const float WAVE_SPEED = 330.0f;      // px/s: a wave crosses the bowl in about 1.1 s
const float DEPTH = 100.0f;           // effective water depth for the wave equation, px
const float G = WAVE_SPEED * WAVE_SPEED / DEPTH;  // so that c = sqrt(G * DEPTH)
const float DAMPING = 0.9f;           // 1/s: how quickly sloshing dies down
const float GRAVITY_SMOOTHING = 0.35f;  // s: how long the water takes to follow a new "down"
const float SUBSTEP = 1.0f / 240.0f;  // s: keeps c * dt / dx well below 1 (stable)
const float MAX_HEIGHT = 150.0f;      // clamp so extreme shakes can't blow up
// Numerical diffusion: each substep nudges every cell toward the average of
// its neighbours. It strongly damps the shortest wave the grid can hold
// (neighbouring cells alternating up/down), which otherwise can grow into
// spiky noise, while barely touching the big visible sloshing waves.
const float SMOOTHING = 0.06f;

// Small motions: quick changes in sideways acceleration are boosted with a
// square-root curve, so taps and footsteps make visible ripples while big
// swings stay physical. Below the dead zone it's sensor noise: ignored.
const float FAST_SMOOTHING = 0.15f;   // s: separates quick changes from steady tilt
const float SENSITIVITY = 0.35f;      // boost = SENSITIVITY * sqrt(|change| in g)
const float DEAD_ZONE = 0.004f;       // g
const float MAX_BOOST = 0.25f;        // g

// Ambient life, so still water never looks frozen.
const float SWELL_AMPLITUDE = 0.012f; // g: a lazy, slowly drifting sideways push
const float SWELL_PERIOD = 2.5f;      // s: how often the swell picks a new target
const float DROP_INTERVAL = 1.8f;     // s: average time between small random disturbances
const float DROP_STRENGTH = 35.0f;    // px/s of flow added at one spot
const float QUIET_BEFORE_DROPS = 3.0f;  // s without splashes or real motion before drops start
const float ACTIVE_BOOST = 0.02f;     // g: a motion boost above this counts as real motion
const float STILL_FLOW = 6.0f;        // px/s: every cell's flow below this counts as still
const float STILL_BEFORE_DROPS = 2.0f;  // s the water must stay still before a drop

// Splashes: a jolt in any direction (a bang on the table, a bump while
// walking) kicks the bowl's natural sloshing shapes and whips up foam.
// Measured on the desk: rest ~0 g, typing nearby 0.07 g, three-finger tap
// 0.2 g, fist on the table > 2 g. 0.1 ignores typing but splashes on taps.
const float JOLT_THRESHOLD = 0.1f;    // g
const float SPLASH_GAIN = 220.0f;     // px/s of flow per sqrt(g) of jolt
const float MAX_SPLASH = 260.0f;      // px/s

// ---- Colours (panel byte order) ----
// The panel is RGB565: only 32 levels of red and blue. A smooth depth gradient
// would show visible bands, so there are two tables offset by half a step,
// alternated in a checkerboard (ordered dithering).
const int MAX_DEPTH = 400;
uint16_t depthColor[2][MAX_DEPTH];
const int EDGE_LEVELS = 16;
uint16_t edgeColor[EDGE_LEVELS];  // foam blended into the black sky, for anti-aliasing
const uint16_t FOAM_COLOR = panelColor(RGB565(235, 245, 255));
const uint16_t SKY_COLOR = 0;

uint16_t rgb565(float r, float g, float b) {
  auto c = [](float v, float maxv) { int i = (int)(v + 0.5f); return i < 0 ? 0 : (i > maxv ? (int)maxv : i); };
  return (uint16_t)((c(r, 31) << 11) | (c(g, 63) << 5) | c(b, 31));
}

void buildColors() {
  for (int d = 0; d < MAX_DEPTH; d++) {
    // Light, slightly green-blue near the surface; deep navy further down.
    float t = fminf(d / 190.0f, 1.0f);
    float near = 1 - t;
    float r = 10 + 40 * near * near;  // 0..255
    float g = 30 + 150 * near;
    float b = 70 + 170 * near;
    for (int k = 0; k < 2; k++) {
      float off = k ? 0.25f : -0.25f;  // +-1/4 of a step: the two dither phases
      depthColor[k][d] = panelColor(rgb565(r * 31 / 255 + off, g * 63 / 255 + off, b * 31 / 255 + off));
    }
  }
  for (int k = 0; k < EDGE_LEVELS; k++) {
    float a = (k + 0.5f) / EDGE_LEVELS;  // share of the pixel covered by water
    edgeColor[k] = panelColor(rgb565(235 * a * 31 / 255, 245 * a * 63 / 255, 255 * a * 31 / 255));
  }
}

uint32_t rngState = 0x12345678;
float random01() {  // xorshift: fast, good enough for ripples
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return (rngState & 0xFFFFFF) / 16777216.0f;
}

}  // namespace

void SideView::begin() {
  buildColors();
  rngState ^= esp_random();
}

void SideView::reset() {
  for (int i = 0; i < N; i++) h_[i] = foam_[i] = 0;
  for (int j = 0; j <= N; j++) q_[j] = 0;
  slowSide_ = swell_ = swellTarget_ = 0;
  quietTime_ = stillTime_ = 0;
}

void SideView::splash(float jolt) {
  // The square root makes small taps visible without big bangs exploding.
  float strength = fminf(SPLASH_GAIN * sqrtf(jolt - JOLT_THRESHOLD), MAX_SPLASH);
  // Kick two of the bowl's natural sloshing shapes (standing waves that fit
  // between the walls), each with a random strength and direction. That
  // looks like water being thrown about, rather than random noise.
  for (int n = 0; n < 2; n++) {
    int mode = 1 + (int)(random01() * 5);  // 1..5 half-waves across the bowl
    float amp = strength * (0.4f + 0.6f * random01()) * (random01() < 0.5f ? -1 : 1);
    for (int j = 1; j < N; j++) {
      q_[j] += amp * sinf(PI * mode * j / N);
    }
  }
  float foam = fminf(strength / MAX_SPLASH, 1.0f);
  for (int i = 0; i < N; i++) foam_[i] = fmaxf(foam_[i], foam * (0.5f + 0.5f * random01()));
}

void SideView::update(float gx, float gy, float dt, float jolt) {
  if (dt > 0.05f) dt = 0.05f;  // after a stall, don't take a giant step
  quietTime_ += dt;
  if (jolt > JOLT_THRESHOLD) {
    splash(jolt);
    quietTime_ = 0;
  }

  // Smoothed "down": follows the raw gravity direction with a lag.
  float len = sqrtf(gx * gx + gy * gy);
  if (len > 0.3f) {  // ignore when the screen is near flat (no clear in-plane "down")
    float a = 1.0f - expf(-dt / GRAVITY_SMOOTHING);
    float nx = downX_ + (gx / len - downX_) * a;
    float ny = downY_ + (gy / len - downY_) * a;
    float n = sqrtf(nx * nx + ny * ny);
    if (n > 1e-3f) {
      downX_ = nx / n;
      downY_ = ny / n;
    }
    lastGx_ = gx;
    lastGy_ = gy;
  }

  // Sideways push in the water frame: how much the raw gravity points along
  // the water's horizontal axis (h = (downY, -downX), "right" when upright), in g.
  float side = lastGx_ * downY_ - lastGy_ * downX_;

  // Boost quick small changes (see SENSITIVITY).
  slowSide_ += (side - slowSide_) * (1.0f - expf(-dt / FAST_SMOOTHING));
  float change = side - slowSide_;
  float mag = fabsf(change) - DEAD_ZONE;
  float boost = mag > 0 ? fminf(SENSITIVITY * sqrtf(mag), MAX_BOOST) : 0;
  if (change < 0) boost = -boost;
  if (fabsf(boost) > ACTIVE_BOOST) quietTime_ = 0;  // the necklace is really moving

  // Ambient swell: drift toward a random target that changes now and then.
  swellTimer_ -= dt;
  if (swellTimer_ <= 0) {
    swellTimer_ = SWELL_PERIOD * (0.6f + 0.8f * random01());
    swellTarget_ = (random01() * 2 - 1) * SWELL_AMPLITUDE;
  }
  swell_ += (swellTarget_ - swell_) * (1.0f - expf(-dt / SWELL_PERIOD));

  // Ambient drops: a small push of flow at one random spot. Only once the
  // water has been left alone for a while: they're there to bring still
  // water to life, not to compete with real motion.
  dropTimer_ -= dt;
  bool settled = quietTime_ >= QUIET_BEFORE_DROPS && stillTime_ >= STILL_BEFORE_DROPS;
  if (dropTimer_ <= 0 && !settled) {
    dropTimer_ = 0;  // ready to drop as soon as it's been quiet long enough
  } else if (dropTimer_ <= 0) {
    dropTimer_ = DROP_INTERVAL * (0.3f + 1.4f * random01());
    int j = 1 + (int)(random01() * (N - 1));
    q_[j] += (random01() < 0.5f ? -1 : 1) * DROP_STRENGTH * (0.5f + random01());
  }

  float aSide = (side + boost + swell_) * G;

  for (float t = 0; t < dt; t += SUBSTEP) {
    step(fminf(SUBSTEP, dt - t), aSide);
  }
  // How long the water has been visibly still (all flow slow).
  float maxFlow = 0;
  for (int j = 1; j < N; j++) maxFlow = fmaxf(maxFlow, fabsf(q_[j]));
  stillTime_ = maxFlow < STILL_FLOW ? stillTime_ + dt : 0;

  updateFoam(dt);
  prepareRender();
}

void SideView::step(float dt, float aSide) {
  // Flow speeds up downhill and with the sideways push.
  float keep = 1.0f - DAMPING * dt;
  for (int j = 1; j < N; j++) {
    float slope = (h_[j] - h_[j - 1]) / DX;
    q_[j] = (q_[j] + dt * (-G * slope + aSide)) * keep;
  }
  q_[0] = q_[N] = 0;  // walls

  // Water moves between cells.
  for (int i = 0; i < N; i++) {
    h_[i] -= dt * DEPTH * (q_[i + 1] - q_[i]) / DX;
  }

  // Numerical diffusion (see SMOOTHING). The walls mirror, so no water is
  // lost or gained.
  float prev = h_[0];
  for (int i = 0; i < N; i++) {
    float left = prev;
    float right = h_[i < N - 1 ? i + 1 : i];
    prev = h_[i];
    float hi = h_[i] + SMOOTHING * (left + right - 2 * h_[i]);
    h_[i] = hi > MAX_HEIGHT ? MAX_HEIGHT : (hi < -MAX_HEIGHT ? -MAX_HEIGHT : hi);
  }
}

void SideView::updateFoam(float dt) {
  float decay = expf(-dt / 0.6f);  // foam fades over about half a second
  for (int i = 0; i < N; i++) {
    // Fast flow and sharp crests whip up foam.
    float flow = fabsf(q_[i] + q_[i + 1]) * 0.5f;
    float left = h_[i > 0 ? i - 1 : i], right = h_[i < N - 1 ? i + 1 : i];
    float crest = h_[i] - 0.5f * (left + right);  // positive at a peak
    float target = flow * 0.012f + (crest > 0 ? crest * 0.25f : 0);
    if (target > 1) target = 1;
    foam_[i] = fmaxf(foam_[i] * decay, target);
  }
}

void SideView::prepareRender() {
  // Surface height and foam thickness for every 1 px step across the bowl,
  // interpolated from the physics cells. Index = u + TABLE_OFFSET; the table
  // is wider than the bowl so rotated corner pixels never index outside it.
  for (int k = 0; k < TABLE_SIZE; k++) {
    float cell = (k - TABLE_OFFSET + RADIUS) / DX;
    float h, foam;
    if (cell <= 0) {
      h = h_[0];
      foam = foam_[0];
    } else if (cell >= N - 1) {
      h = h_[N - 1];
      foam = foam_[N - 1];
    } else {
      int i = (int)cell;
      float f = cell - i;
      h = h_[i] + (h_[i + 1] - h_[i]) * f;
      foam = foam_[i] + (foam_[i + 1] - foam_[i]) * f;
    }
    surfaceQ_[k] = (int32_t)((REST_LEVEL + h) * 65536.0f);
    foamPx_[k] = (uint8_t)(1 + foam * 9);  // always at least a thin line
  }
  // Water-frame axes: horizontal h = (downY, -downX), down d = (downX, downY).
  hxQ_ = (int32_t)(downY_ * 65536.0f);
  hyQ_ = (int32_t)(-downX_ * 65536.0f);
  dxQ_ = (int32_t)(downX_ * 65536.0f);
  dyQ_ = (int32_t)(downY_ * 65536.0f);
}

void SideView::renderStrip(uint16_t *pixels, int16_t y0, int16_t rows, void *ctx) {
  static_cast<const SideView *>(ctx)->render(pixels, y0, rows);
}

void SideView::render(uint16_t *pixels, int16_t y0, int16_t rows) const {
  const int32_t offsetQ = (int32_t)TABLE_OFFSET << 16;
  const int32_t du = hxQ_, dv = -dxQ_;

  for (int16_t r = 0; r < rows; r++) {
    int16_t y = y0 + r;
    uint16_t *row = pixels + (size_t)r * WIDTH;

    // Visible span of this row: the round display only shows pixels inside
    // the circle, so the corners just get black.
    float dyc = y - HEIGHT / 2 + 0.5f;
    float half2 = RADIUS * RADIUS - dyc * dyc;
    int16_t half = half2 > 0 ? (int16_t)ceilf(sqrtf(half2)) : 0;
    int16_t xStart = WIDTH / 2 - half, xEnd = WIDTH / 2 + half;
    if (xStart < 0) xStart = 0;
    if (xEnd > WIDTH) xEnd = WIDTH;
    for (int16_t x = 0; x < xStart; x++) row[x] = SKY_COLOR;
    for (int16_t x = xEnd; x < WIDTH; x++) row[x] = SKY_COLOR;

    // Pixel centre relative to the bowl centre, in 16.16.
    int32_t py = ((int32_t)y - HEIGHT / 2) * 65536 + 32768;
    int32_t px = ((int32_t)xStart - WIDTH / 2) * 65536 + 32768;
    // u = px*hx + py*hy (across), v = -(px*dx + py*dy) (up). Both are linear
    // along the row, so step them by a constant per pixel.
    int32_t u = (int32_t)(((int64_t)px * hxQ_ + (int64_t)py * hyQ_) >> 16) + offsetQ;
    int32_t v = (int32_t)(-(((int64_t)px * dxQ_ + (int64_t)py * dyQ_) >> 16));

    const uint16_t *dither[2] = {depthColor[y & 1], depthColor[(y & 1) ^ 1]};
    for (int16_t x = xStart; x < xEnd; x++, u += du, v += dv) {
      int k = u >> 16;
      int32_t depthQ = surfaceQ_[k] - v;  // 16.16: how far below the surface
      if (depthQ < 0) {
        // Sky, except for the pixel straddling the surface: anti-alias it by
        // how much of it is water.
        row[x] = depthQ > -65536 ? edgeColor[(depthQ + 65536) >> 12] : SKY_COLOR;
        continue;
      }
      int32_t depth = depthQ >> 16;
      if (depth < foamPx_[k]) {
        row[x] = FOAM_COLOR;
      } else {
        row[x] = dither[x & 1][depth < MAX_DEPTH ? depth : MAX_DEPTH - 1];
      }
    }
  }
}
