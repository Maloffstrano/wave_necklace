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

// ---- How the tuning constants affect each other ----
// (The full version, with a table, is in things-I-learned.md.)
// WAVE_SPEED is the hub: G = WAVE_SPEED^2 / DEPTH.
//   - Tilting: the sideways push is scaled by G, so how far the surface tilts
//     does NOT depend on WAVE_SPEED. SENSITIVITY and SWELL_AMPLITUDE (in g)
//     behave the same at any speed.
//   - Splashes and drops add flow (px/s). Wave height ~ flow * DEPTH / speed,
//     so halving WAVE_SPEED doubles splash heights: SPLASH_GAIN, MAX_SPLASH
//     and DROP_STRENGTH must come down with it.
//   - How fast water climbs the wall (the rise rate) scales with WAVE_SPEED,
//     and everything driven by it follows:
//       curl launch speed = rise * CURL_SPEED_GAIN; flight height ~ speed^2,
//       so halving the waves quarters curl height unless the gain doubles
//       curl density = rise * CURL_EMIT_RATE (also halves)
//       crest spray (SPRAY_MIN_RISE) and whipped-up foam (flow * 0.012) fade
//   - STILL_FLOW (when drops may start) is a flow speed: slower waves count as
//     "still" sooner.
//   - SUBSTEP must keep WAVE_SPEED * SUBSTEP / DX well below 1 (now 0.17).
// Independent of WAVE_SPEED:
//   - DAMPING (settling time in seconds; slower waves cross the bowl fewer
//     times before they settle, so they feel heavier)
//   - PARTICLE_GRAVITY (curl height for a given launch speed, fall speed,
//     and the loop-the-loop limit CURL_MAX_SPEED ~ sqrt(gravity * radius))
//   - the lean level (REST_LEVEL..FULL_LEVEL), colours, foam fade.
// PoolView copies FULL_LEVEL and the depth colour ramp: change them together.

// Pixel loops: Arduino builds with -Os (small code), which is slow for tight
// inner loops. Optimise this file for speed instead.
#pragma GCC optimize("O3")

#include "SideView.h"

using namespace NecklaceBoard;

namespace {

// ---- Tuning ----
const float REST_LEVEL = -14.0f;      // rest surface height (px above the bowl centre): about 45% full
// Leaning back shows more of the water (like looking into a glass from above):
// the level rises from REST_LEVEL at LEAN_RISE_START to FULL_LEVEL at
// LEAN_RISE_END, filling the bowl completely, and the top-down pool takes over.
const float LEAN_RISE_START = 40.0f;  // degrees: normal wear (+26..+34) stays unchanged
const float LEAN_RISE_END = 80.0f;    // degrees
const float FULL_LEVEL = 195.0f;      // px: above the rim (180), so the bowl is full before the pool takes over
const float WAVE_SPEED = 165.0f;      // px/s: a wave crosses the bowl in about 2.2 s ("slow and stately")
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
const float DROP_STRENGTH = 30.0f;    // px/s of flow added at one spot
const float QUIET_BEFORE_DROPS = 3.0f;  // s without splashes or real motion before drops start
const float ACTIVE_BOOST = 0.02f;     // g: a motion boost above this counts as real motion
const float STILL_FLOW = 6.0f;        // px/s: every cell's flow below this counts as still
const float STILL_BEFORE_DROPS = 2.0f;  // s the water must stay still before a drop

// Splashes: a jolt in any direction (a bang on the table, a bump while
// walking) kicks the bowl's natural sloshing shapes and whips up foam.
// Measured on the desk: rest ~0 g, typing nearby 0.07 g, three-finger tap
// 0.2 g, fist on the table > 2 g. 0.1 ignores typing but splashes on taps.
const float JOLT_THRESHOLD = 0.1f;    // g
const float SPLASH_GAIN = 90.0f;      // px/s of flow per sqrt(g) of jolt (lower now that waves are slower)
const float MAX_SPLASH = 110.0f;      // px/s

// Safety limits for violent shaking. Chosen so that normal tilting, rolling,
// taps and bangs are unaffected; to be fine-tuned once it's worn on battery.
const float PUSH_KNEE = 0.8f;         // g: sideways push is linear up to here...
const float PUSH_CEILING = 1.5f;      // g: ...then bends smoothly toward this
const float PARTICLES_PER_SECOND = 400.0f;  // emission budget
const float PARTICLE_BURST = 120.0f;  // most particles that can be emitted at once
const float CALM_RMS = 15.0f;         // px: below this RMS surface height, normal damping
const float CHURN_RMS = 55.0f;        // px: at this RMS, full extra damping
const float TURBULENT_DAMPING = 2.5f; // 1/s extra (churning water loses energy faster)

// Spray and the wall curl. Water racing up the round wall above the bowl's
// middle (where the wall curves back inward) is launched along the wall's
// tangent: it shoots up, already heading inward, and gravity bends it over
// into an arc that lands back on the water. A stream of those is the curl.
const float PARTICLE_GRAVITY = 1000.0f;  // px/s^2 (independent of the wave model's G; sets curl height and fall speed)
const float CURL_MIN_HEIGHT = -25.0f;    // px: contact point must be above this (v)
const float CURL_SPEED_GAIN = 2.8f;      // launch speed = rise rate * this (doubled when the waves were halved)
// A slow climb just runs smoothly up the wall (the surface shows that); only
// launch a tongue when it will clearly leave the water: 220 px/s flies about
// 24 px (v^2 / 2g) before falling back.
const float CURL_MIN_SPEED = 220.0f;     // px/s
// Above about sqrt(g * R) (~420 px/s) particles ride the wall over the top of
// the bowl ("loop-the-loop") instead of peeling off into a curl.
const float CURL_MAX_SPEED = 410.0f;     // px/s: just under the loop-the-loop limit
const float CURL_EMIT_RATE = 0.04f;      // particles per frame per px/s of rise (doubled with the slower waves)
// The water running up the wall is a tongue: thick at the base, stretching
// thin as it climbs. Particles start spread over CURL_THICKNESS px in from the
// wall; the ones nearest the wall are fastest (they're riding the wall), the
// inner ones slower, so they fall back first and the tip stretches out. Each
// starts as a fat blob and shrinks to a thin sheet as it ages.
const float CURL_THICKNESS = 12.0f;      // px: how far in from the wall the tongue starts
const float CURL_INNER_SLOWDOWN = 0.35f; // innermost particles launch this much slower
const float CURL_LIFE = 2.0f;            // s
const float CURL_BLOB_TIME = 0.45f;      // s: blobs shrink from fat to thin over this
const int CURL_BLOB_R = 6;               // px: radius at launch
const int CURL_SHEET_R = 2;              // px: radius once stretched out
const float SPRAY_MIN_RISE = 220.0f;     // px/s: crests rising faster throw droplets
const float SPRAY_CHANCE = 0.0003f;      // per frame, per px/s above the minimum
const int MAX_SPRAY_PER_FRAME = 1;       // across the whole surface
const float WALL_BOUNCE = 0.25f;         // share of speed kept bouncing off the wall
// Friction is per second, not per contact: a particle sliding along the wall
// touches it every frame. With gentle friction the sheet keeps its speed,
// slows under gravity as it climbs, and detaches by itself once it is too
// slow to stay pressed against the curve (speed^2 / R < the part of gravity
// pulling it off the wall). That detachment is the curl.
const float WALL_FRICTION = 1.2f;        // 1/s while sliding along the wall
const float FOAM_MAX_PX = 3.0f;          // foam line: 1 px + up to this much more
const float LANDING_FOAM = 0.6f;         // foam a landing particle leaves (0..1): about 2 px thicker
// Spray doesn't vanish when it lands: it becomes a fleck of foam riding the
// surface (up and down with it, drifting with the flow), then fades.
const float FLECK_LIFE = 0.8f;           // s

// ---- Colours (panel byte order) ----
// The panel is RGB565: only 32 levels of red and blue. A smooth depth gradient
// would show visible bands, so there are two tables offset by half a step,
// alternated in a checkerboard (ordered dithering).
const int MAX_DEPTH = 400;
uint16_t depthColor[2][MAX_DEPTH];
const int EDGE_LEVELS = 16;
// Near the wall the froth fades from blue-white into the colour of the water
// just under the surface, which is also the colour of the curl tongue at its
// base, so the tongue grows out of the water body instead of out of a white
// line. FOAM_SHADES steps from 0 (water, at the wall) to the last (full froth).
const int FOAM_SHADES = 8;
const float WALL_FADE_PX = 50.0f;  // width of the fade, measured from the wall
uint16_t foamShade[FOAM_SHADES];
uint16_t edgeColor[FOAM_SHADES][EDGE_LEVELS];  // each shade blended into the black sky, for anti-aliasing
const uint16_t DROPLET_COLOR = panelColor(RGB565(240, 248, 255));
// The curl tongue is water: it starts in the colour of the water just under
// the surface (so its overlapping blobs merge into one smooth shape with the
// water body), turns to the paler sheet colour as it thins and climbs, then
// towards white as it flies apart as spray. Falling pieces are always pale.
const int CURL_SHADES = 12;
const float CURL_SHADE_TIME = 0.9f;   // s from water colour to spray white
uint16_t curlShade[CURL_SHADES];
const int CURL_FALLING_SHADE = CURL_SHADES - 1;  // falling pieces: spray white
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
  // Water just under the surface (depth 0 above), the base of everything below.
  const float WR = 50, WG = 180, WB = 240;
  // Curl shades: water -> sheet blue (140, 205, 245) -> spray (200, 235, 255).
  for (int c = 0; c < CURL_SHADES; c++) {
    float t = (float)c / (CURL_SHADES - 1) * 2;  // 0..1 water->sheet, 1..2 sheet->spray
    float r, g, b;
    if (t <= 1) { r = WR + (140 - WR) * t; g = WG + (205 - WG) * t; b = WB + (245 - WB) * t; }
    else { t -= 1; r = 140 + 60 * t; g = 205 + 30 * t; b = 245 + 10 * t; }
    curlShade[c] = panelColor(rgb565(r * 31 / 255, g * 63 / 255, b * 31 / 255));
  }
  // Froth shades: water colour at the wall -> froth (205, 232, 252).
  for (int f = 0; f < FOAM_SHADES; f++) {
    float t = (float)f / (FOAM_SHADES - 1);
    float r = WR + (205 - WR) * t, g = WG + (232 - WG) * t, b = WB + (252 - WB) * t;
    foamShade[f] = panelColor(rgb565(r * 31 / 255, g * 63 / 255, b * 31 / 255));
    for (int k = 0; k < EDGE_LEVELS; k++) {
      float a = (k + 0.5f) / EDGE_LEVELS;  // share of the pixel covered by water
      edgeColor[f][k] = panelColor(rgb565(r * a * 31 / 255, g * a * 63 / 255, b * a * 31 / 255));
    }
  }
}

uint32_t rngState = 0x12345678;
float random01() {  // xorshift: fast, good enough for ripples
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return (rngState & 0xFFFFFF) / 16777216.0f;
}

// Linear up to PUSH_KNEE, then bends smoothly toward PUSH_CEILING.
float softClip(float x) {
  float a = fabsf(x);
  if (a <= PUSH_KNEE) return x;
  float range = PUSH_CEILING - PUSH_KNEE;
  float y = PUSH_KNEE + range * tanhf((a - PUSH_KNEE) / range);
  return x < 0 ? -y : y;
}

}  // namespace

void SideView::begin() {
  buildColors();
  rngState ^= esp_random();
}

void SideView::setDown(float x, float y) {
  float n = sqrtf(x * x + y * y);
  if (n < 1e-3f) return;
  downX_ = x / n;
  downY_ = y / n;
  // Gravity "as last seen" along the same line, so no sideways push is
  // felt from the switch itself; the water then follows the real tilt.
  lastGx_ = downX_;
  lastGy_ = downY_;
  slowSide_ = 0;
}

void SideView::setLean(float leanDeg) {
  float t = (leanDeg - LEAN_RISE_START) / (LEAN_RISE_END - LEAN_RISE_START);
  t = fminf(fmaxf(t, 0.0f), 1.0f);
  t = t * t * (3 - 2 * t);  // smoothstep: eases in and out of the rise
  restLevel_ = REST_LEVEL + (FULL_LEVEL - REST_LEVEL) * t;
}

void SideView::reset() {
  for (int i = 0; i < N; i++) h_[i] = foam_[i] = 0;
  for (int j = 0; j <= N; j++) q_[j] = 0;
  slowSide_ = swell_ = swellTarget_ = 0;
  quietTime_ = stillTime_ = 0;
  damping_ = DAMPING;
  particleBudget_ = PARTICLE_BURST;
  for (auto &p : particles_) p.kind = 0;
  for (int i = 0; i < N; i++) hPrev_[i] = rate_[i] = 0;
  dotCount_ = 0;
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

  // A burst of droplets thrown up from random spots on the surface.
  int drops = (int)(25 * foam);
  for (int n = 0; n < drops; n++) {
    float u = (random01() * 2 - 1) * RADIUS * 0.9f;
    float up = strength * (0.8f + 1.2f * random01());
    emit(u, surfaceAt(u) + 1, (random01() * 2 - 1) * strength * 0.6f, up, 2, 1.5f);
  }
}

// ---- Particles ----

float SideView::surfaceAt(float u) const {
  float cell = (u + RADIUS) / DX;
  if (cell <= 0) return restLevel_ + h_[0];
  if (cell >= N - 1) return restLevel_ + h_[N - 1];
  int i = (int)cell;
  float f = cell - i;
  return restLevel_ + h_[i] + (h_[i + 1] - h_[i]) * f;
}

void SideView::emit(float u, float v, float vu, float vv, uint8_t kind, float life) {
  if (particleBudget_ < 1) return;  // over budget this moment: skip
  particleBudget_ -= 1;
  // Take the next free slot. Only if the whole pool is in flight, reuse the
  // oldest slot (that particle vanishes mid-air, so keep the pool big enough).
  int slot = nextParticle_;
  for (int k = 0; k < MAX_PARTICLES; k++) {
    int i = (nextParticle_ + k) % MAX_PARTICLES;
    if (!particles_[i].kind) {
      slot = i;
      break;
    }
  }
  nextParticle_ = (slot + 1) % MAX_PARTICLES;
  particles_[slot] = {u, v, vu, vv, life, kind};
}

void SideView::emitWallCurl(int side, float dt) {
  // Where does the surface meet the wall on this side? The wall at height v
  // is at |u| = sqrt(R^2 - v^2); a few iterations converge.
  float u = side * (RADIUS - 2);
  float v = surfaceAt(u);
  for (int k = 0; k < 3; k++) {
    float w2 = RADIUS * RADIUS - v * v;
    u = side * (w2 > 0 ? sqrtf(w2) - 2 : 0);
    v = surfaceAt(u);
  }
  if (v < CURL_MIN_HEIGHT) return;

  int cell = (int)((u + RADIUS) / DX + 0.5f);
  if (cell < 0) cell = 0;
  if (cell > N - 1) cell = N - 1;
  float rise = rate_[cell];
  if (rise * CURL_SPEED_GAIN < CURL_MIN_SPEED) return;

  // Tangent to the wall, pointing up: (-v, u)/R on the right, (v, -u)/R on the left.
  float tu = side > 0 ? -v / RADIUS : v / RADIUS;
  float tv = side > 0 ? u / RADIUS : -u / RADIUS;
  float speed = fminf(rise * CURL_SPEED_GAIN, CURL_MAX_SPEED);
  float count = rise * CURL_EMIT_RATE * (dt * 60);  // about the same per second at any FPS
  int n = (int)count + (random01() < count - (int)count ? 1 : 0);
  // Inward normal (towards the bowl centre) at the contact point.
  float nu = -u / RADIUS, nv = -v / RADIUS;
  for (int k = 0; k < n; k++) {
    float depth = random01();  // 0 = against the wall, 1 = innermost
    float s = speed * (1 - CURL_INNER_SLOWDOWN * depth) * (0.95f + 0.1f * random01());
    float in = 1.5f + depth * CURL_THICKNESS;
    float along = random01() * 4;  // spread a little along the wall
    emit(u + nu * in + tu * along, v + nv * in + tv * along,
         tu * s, tv * s, 1, CURL_LIFE);
  }
  foam_[cell] = fmaxf(foam_[cell], fminf(rise / 600, 0.5f));
}

void SideView::emitCrestSpray(float dt) {
  int emitted = 0;
  for (int i = 2; i < N - 2 && emitted < MAX_SPRAY_PER_FRAME; i++) {
    float rise = rate_[i];
    if (rise < SPRAY_MIN_RISE) continue;
    if (h_[i] < h_[i - 1] || h_[i] < h_[i + 1]) continue;  // only from crests
    float chance = (rise - SPRAY_MIN_RISE) * SPRAY_CHANCE * (dt * 60);
    if (random01() > chance) continue;
    float u = -RADIUS + i * DX;
    float flow = 0.5f * (q_[i] + q_[i + 1]);
    emit(u, surfaceAt(u) + 1, flow * 0.8f + (random01() * 2 - 1) * 30,
         rise * (0.6f + 0.6f * random01()), 2, 1.2f);
    emitted++;
  }
}

void SideView::updateParticles(float dt) {
  const float limit = RADIUS - 1.5f;
  for (int k = 0; k < MAX_PARTICLES; k++) {
    Particle &p = particles_[k];
    if (!p.kind) continue;
    p.life -= dt;
    if (p.life <= 0) {
      p.kind = 0;
      continue;
    }
    if (p.kind == 3) {
      // A fleck riding the surface: drift with the flow underneath.
      int face = (int)((p.u + RADIUS) / DX + 0.5f);
      if (face >= 0 && face <= N) p.u += q_[face] * dt;
      if (p.u > limit) p.u = limit;
      if (p.u < -limit) p.u = -limit;
      p.v = surfaceAt(p.u);
      continue;
    }
    p.vv -= PARTICLE_GRAVITY * dt;
    p.u += p.vu * dt;
    p.v += p.vv * dt;

    // The bowl wall: push back inside, mostly absorb the bounce, slide along.
    float r2 = p.u * p.u + p.v * p.v;
    if (r2 > limit * limit) {
      float r = sqrtf(r2);
      float nu = p.u / r, nv = p.v / r;  // outward normal
      p.u = nu * limit;
      p.v = nv * limit;
      float vn = p.vu * nu + p.vv * nv;
      if (vn > 0) {
        float slide = expf(-WALL_FRICTION * dt);
        float tvu = p.vu - vn * nu, tvv = p.vv - vn * nv;
        p.vu = tvu * slide - vn * WALL_BOUNCE * nu;
        p.vv = tvv * slide - vn * WALL_BOUNCE * nv;
      }
    }

    // Back into the water. Falling onto the surface: land as a fleck that
    // rides it. Swallowed by a rising wave from below: just gone.
    float surface = surfaceAt(p.u);
    if (p.vv < 0 && p.v < surface) {
      int cell = (int)((p.u + RADIUS) / DX + 0.5f);
      if (cell >= 0 && cell < N) foam_[cell] = fmaxf(foam_[cell], LANDING_FOAM);
      p.kind = 3;
      p.life = FLECK_LIFE;
      p.v = surface;
      p.vu = p.vv = 0;
    } else if (p.v < surface - 2) {
      p.kind = 0;
    }
  }
}

void SideView::prepareParticles() {
  // Water frame -> screen: screen offset = u * h + (-v) * d, with
  // h = (downY, -downX) and d = (downX, downY).
  dotCount_ = 0;
  for (int k = 0; k < MAX_PARTICLES; k++) {
    const Particle &p = particles_[k];
    if (!p.kind) continue;
    Dot &d = dots_[dotCount_++];
    d.x = (int16_t)(WIDTH / 2 + p.u * downY_ - p.v * downX_);
    d.y = (int16_t)(HEIGHT / 2 - p.u * downX_ - p.v * downY_);
    if (p.kind == 1) {
      // Fat blob at the base of the tongue, thinning to a sheet as it climbs.
      float young = 1 - (CURL_LIFE - p.life) / CURL_BLOB_TIME;
      d.r = (uint8_t)(CURL_SHEET_R + (young > 0 ? (int)(young * (CURL_BLOB_R - CURL_SHEET_R) + 0.5f) : 0));
      // Glassy water on the way up, paling with age; once it's falling it
      // has broken up into spray, so it turns white (otherwise a young piece
      // falling back is water-coloured and seems to land behind the surface).
      float age = (CURL_LIFE - p.life) / CURL_SHADE_TIME;
      int shade = age >= 1 ? CURL_SHADES - 1 : (int)(age * (CURL_SHADES - 1));
      if (p.vv < 0 && shade < CURL_FALLING_SHADE) shade = CURL_FALLING_SHADE;
      d.color = curlShade[shade];
    } else if (p.kind == 3) {
      d.r = p.life > FLECK_LIFE / 2 ? 2 : 1;  // shrinks as it fades
      d.color = DROPLET_COLOR;
    } else {
      d.r = 1 + (k & 1);
      d.color = DROPLET_COLOR;
    }
  }
}

void SideView::drawParticles(uint16_t *pixels, int16_t y0, int16_t rows) const {
  const int32_t r2max = (int32_t)(RADIUS - 1) * (RADIUS - 1);
  for (int k = 0; k < dotCount_; k++) {
    const Dot &d = dots_[k];
    if (d.y + d.r < y0 || d.y - d.r >= y0 + rows) continue;
    for (int dy = -d.r; dy <= d.r; dy++) {
      int y = d.y + dy;
      if (y < y0 || y >= y0 + rows) continue;
      int half = (int)sqrtf((float)(d.r * d.r - dy * dy) + 0.5f);
      uint16_t *row = pixels + (size_t)(y - y0) * WIDTH;
      int cy = y - HEIGHT / 2;
      for (int x = d.x - half; x <= d.x + half; x++) {
        if (x < 0 || x >= WIDTH) continue;
        int cx = x - WIDTH / 2;
        if (cx * cx + cy * cy > r2max) continue;  // outside the round screen
        row[x] = d.color;
      }
    }
  }
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

  float aSide = softClip(side + boost + swell_) * G;

  // Churning water loses energy faster: extra damping above CALM_RMS.
  float sumSq = 0;
  for (int i = 0; i < N; i++) sumSq += h_[i] * h_[i];
  float rms = sqrtf(sumSq / N);
  float churn = fminf(fmaxf((rms - CALM_RMS) / (CHURN_RMS - CALM_RMS), 0.0f), 1.0f);
  damping_ = DAMPING + TURBULENT_DAMPING * churn;

  // Refill the particle budget.
  particleBudget_ = fminf(particleBudget_ + PARTICLES_PER_SECOND * dt, PARTICLE_BURST);

  for (float t = 0; t < dt; t += SUBSTEP) {
    step(fminf(SUBSTEP, dt - t), aSide);
  }
  // How fast each cell's surface is rising, then spray and the wall curl.
  if (dt > 0) {
    for (int i = 0; i < N; i++) {
      rate_[i] = (h_[i] - hPrev_[i]) / dt;
      hPrev_[i] = h_[i];
    }
    emitWallCurl(-1, dt);
    emitWallCurl(+1, dt);
    emitCrestSpray(dt);
    updateParticles(dt);
  }

  // How long the water has been visibly still (all flow slow).
  float maxFlow = 0;
  for (int j = 1; j < N; j++) maxFlow = fmaxf(maxFlow, fabsf(q_[j]));
  stillTime_ = maxFlow < STILL_FLOW ? stillTime_ + dt : 0;

  updateFoam(dt);
  prepareRender();
  prepareParticles();
}

void SideView::step(float dt, float aSide) {
  // Flow speeds up downhill and with the sideways push.
  float keep = 1.0f - damping_ * dt;
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
    surfaceQ_[k] = (int32_t)((restLevel_ + h) * 65536.0f);
    foamPx_[k] = (uint8_t)(1 + foam * FOAM_MAX_PX + 0.5f);  // always at least a thin line

    // Distance to the wall at the surface's height (the wall is a circle,
    // so at height v it sits at |u| = sqrt(R^2 - v^2)), for the froth fade.
    float u = (float)(k - TABLE_OFFSET);
    float v = restLevel_ + h;
    float w2 = RADIUS * RADIUS - v * v;
    float wallDist = (w2 > 0 ? sqrtf(w2) : 0) - fabsf(u);
    float t = wallDist / WALL_FADE_PX;
    foamShade_[k] = (uint8_t)(t <= 0 ? 0 : (t >= 1 ? FOAM_SHADES - 1 : t * (FOAM_SHADES - 1) + 0.5f));
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
        row[x] = depthQ > -65536 ? edgeColor[foamShade_[k]][(depthQ + 65536) >> 12] : SKY_COLOR;
        continue;
      }
      int32_t depth = depthQ >> 16;
      if (depth < foamPx_[k]) {
        row[x] = foamShade[foamShade_[k]];
      } else {
        row[x] = dither[x & 1][depth < MAX_DEPTH ? depth : MAX_DEPTH - 1];
      }
    }
  }
  drawParticles(pixels, y0, rows);
}
