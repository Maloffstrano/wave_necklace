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

// ---- How the fluid's tuning (Material.h) fits together ----
// (The full version, with a table, is in things-I-learned.md.)
// waveSpeed is the hub: G = waveSpeed^2 / depth.
//   - Tilting: the sideways push is scaled by G, so how far the surface tilts
//     does NOT depend on waveSpeed. sensitivity and swellAmplitude (in g)
//     behave the same at any speed.
//   - Splashes and drops add flow (px/s). Wave height ~ flow * depth / speed,
//     so halving waveSpeed doubles splash heights: splashGain, maxSplash
//     and dropStrength must come down with it.
//   - How fast water climbs the wall (the rise rate) scales with waveSpeed,
//     and everything driven by it follows:
//       curl launch speed = rise * curlSpeedGain; flight height ~ speed^2,
//       so halving the waves quarters curl height unless the gain doubles
//       curl density = rise * curlEmitRate (also halves)
//       crest spray (sprayMinRise) and whipped-up foam (flow * foamFlowGain) fade
//   - STILL_FLOW (when drops may start) is a flow speed: slower waves count as
//     "still" sooner.
//   - SUBSTEP must keep waveSpeed * SUBSTEP / DX well below 1 (now 0.17).
// Independent of waveSpeed:
//   - damping (settling time in seconds; slower waves cross the bowl fewer
//     times before they settle, so they feel heavier)
//   - particleGravity (curl height for a given launch speed, fall speed,
//     and the loop-the-loop limit curlMaxSpeed ~ sqrt(gravity * radius))
//   - the lean level (REST_LEVEL..FULL_LEVEL), colours, foam fade.
// PoolView copies FULL_LEVEL and uses the same depth colours: keep FULL_LEVEL
// the same in both.

// Pixel loops: Arduino builds with -Os (small code), which is slow for tight
// inner loops. Optimise this file for speed instead.
#pragma GCC optimize("O3")

#include "SideView.h"

using namespace NecklaceBoard;

namespace {

// ---- The bowl ----
// The fluid's own tuning (wave speed, damping, splashes, the curl, foam and
// colours) is in Material.h / Material.cpp. What's here is about the bowl
// and is the same for any fluid.
const float REST_LEVEL = -14.0f;      // rest surface height (px above the bowl centre): about 45% full
// Leaning back shows more of the water (like looking into a glass from above):
// the level rises from REST_LEVEL at LEAN_RISE_START to FULL_LEVEL at
// LEAN_RISE_END, filling the bowl completely, and the top-down pool takes over.
const float LEAN_RISE_START = 40.0f;  // degrees: normal wear (+26..+34) stays unchanged
const float LEAN_RISE_END = 80.0f;    // degrees
const float FULL_LEVEL = 195.0f;      // px: above the rim (180), so the bowl is full before the pool takes over
const float GRAVITY_SMOOTHING = 0.35f;  // s: how long the water takes to follow a new "down"
const float SUBSTEP = 1.0f / 240.0f;  // s: keeps waveSpeed * dt / dx well below 1 (stable)
const float MAX_HEIGHT = 150.0f;      // clamp so extreme shakes can't blow up

// ---- The bowl model (setBowlModel) ----
// The circle is the container. Each cell is a column of water standing on
// the round floor (bed = -sqrt(R^2 - u^2)); near the walls the floor curves
// up, the water gets shallow, and can run out (dry). These are the full
// (non-linear) shallow-water equations with an uneven floor, the standard
// model for waves running up a beach:
//   depth:    d_t + (d w)_u = 0
//   momentum: (d w)_t + (d w^2 + g d^2 / 2)_u = -g d bed_u + d a_side
// solved with finite volumes: a Rusanov flux between cells, and "hydrostatic
// reconstruction" (Audusse et al. 2004) at each face, which keeps water at
// rest exactly at rest over the curved floor, and keeps depth >= 0 where the
// shoreline moves. Waves travel at sqrt(g d): fast in the deep middle, slower
// in the shallows near the walls, so they bunch up and steepen as they run
// up the side (shoaling, which is what makes real waves break).
const float CENTRE_DEPTH = REST_LEVEL + 180.0f;  // px of water at the centre at rest (166)
const float WET = 0.5f;               // px: shallower than this counts as dry
const float BOWL_CFL = 0.45f;         // fraction of a cell a wave may cross per substep (< 0.5)
const int MAX_BOWL_SUBSTEPS = 48;
const float MAX_FLOW_SPEED = 700.0f;  // px/s: cap for thin films near the shoreline
// The bowl's numerical method already smooths the water a little every step
// (that's what keeps the moving shoreline stable), so it needs less damping
// than classic for the same liveliness. Halving it roughly doubles how much
// water a slosh sends past the equator (measured in a simulation).
const float BOWL_DAMPING_SCALE = 0.5f;
// The curl in the bowl model: above the equator the wall curves back over the
// water, which a column model can't hold. Water that climbs past it leaves the
// body as curl pieces, each carrying some of that water. They leave at the
// speed the water's edge was climbing the wall when it got there (times
// Material::curlSpeedGain): near the steep wall a small sideways shift lifts
// the edge a long way, so the edge climbs fast even though the water itself
// barely moves sideways. The round wall steers the pieces inward, gravity
// brings them down, and their water rejoins the body where they land.
// Measured in a simulation: the edge reaches the equator climbing ~70 px/s
// in a firm swing, ~140 in a hard jolt, ~350 in violent shaking.
const float CURL_PIECE_WATER = 4.0f;  // px^2 of water per curl piece (the jet is a thin sheet)
const float BOWL_MIN_LAUNCH = 100.0f; // px/s: slow overflow still plops off the wall
const float BOWL_LAUNCH_BOOST = 1.5f; // extra on top of Material::curlSpeedGain (the bowl's edge climbs slower than classic's wall rise)
const float CLIMB_SMOOTHING = 0.03f;  // s
const int OVERHANG_CELLS = 4;         // outermost cells checked for water above the wall
// Numerical diffusion (Material::smoothing): each substep nudges every cell
// toward the average of its neighbours. It strongly damps the shortest wave
// the grid can hold (neighbouring cells alternating up/down), which otherwise
// can grow into spiky noise, while barely touching the big sloshing waves.

// Small motions: quick changes in sideways acceleration are boosted with a
// square-root curve (Material::sensitivity), so taps and footsteps make
// visible ripples while big swings stay physical. Below the dead zone it's
// sensor noise: ignored.
const float FAST_SMOOTHING = 0.15f;   // s: separates quick changes from steady tilt
const float DEAD_ZONE = 0.004f;       // g
const float MAX_BOOST = 0.25f;        // g

// Ambient life, so still water never looks frozen: a lazy swell
// (Material::swellAmplitude) and, once it has been still for a while, small
// drops (Material::dropStrength, dropInterval). In lava a drop is a gas
// bubble popping (Material::bloopSpeed): the surface lifts and a small blob
// jumps just off it and falls back.
const float SWELL_PERIOD = 2.5f;      // s: how often the swell picks a new target
const float QUIET_BEFORE_DROPS = 3.0f;  // s without splashes or real motion before drops start
const float ACTIVE_BOOST = 0.02f;     // g: a motion boost above this counts as real motion
const float STILL_FLOW = 6.0f;        // px/s: every cell's flow below this counts as still
const float STILL_BEFORE_DROPS = 2.0f;  // s the water must stay still before a drop

// Splashes: a jolt in any direction (a bang on the table, a bump while
// walking) kicks the bowl's natural sloshing shapes and whips up foam.
// Measured on the desk: rest ~0 g, typing nearby 0.07 g, three-finger tap
// 0.2 g, fist on the table > 2 g. 0.1 ignores typing but splashes on taps.
const float JOLT_THRESHOLD = 0.1f;    // g

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
//   - A slow climb just runs smoothly up the wall; only climbs that launch
//     faster than curlMinSpeed throw a tongue (220 px/s flies ~24 px).
//   - Above about sqrt(gravity * R) pieces ride the wall over the top of the
//     bowl ("loop-the-loop") instead of peeling off: curlMaxSpeed stays below.
//   - The tongue is thick at the base and stretches thin as it climbs: pieces
//     start spread over curlThickness px in from the wall; the ones nearest
//     the wall are fastest, the inner ones slower, so they fall back first.
//     Each starts as a fat blob and shrinks to a thin sheet.
//   - Wall friction is per second, not per contact: a piece sliding along the
//     wall touches it every frame. With gentle friction the sheet slows under
//     gravity as it climbs and detaches once too slow to stay pressed against
//     the curve (speed^2 / R < the part of gravity pulling it off). That
//     detachment is the curl.
//   - Landing pieces become flecks riding the surface, then fade.
const float CURL_MIN_HEIGHT = -25.0f;    // px: contact point must be above this (v)
const int MAX_SPRAY_PER_FRAME = 1;       // across the whole surface
// A forming curl sweeps up the foam near its wall (the rising water carries
// it into the curl). Otherwise old foam and landed flecks draw a white line
// across the base of the new tongue, and it looks cut off from the water.
const float CURL_CONSUMES_PX = 40.0f;    // px from the wall

// ---- Colours (panel byte order), built from the material ----
// The panel is RGB565: only 32 levels of red and blue. A smooth depth gradient
// would show visible bands, so there are two tables offset by half a step,
// alternated in a checkerboard (ordered dithering).
const int MAX_DEPTH = 400;
uint16_t depthColor[2][MAX_DEPTH];
const int EDGE_LEVELS = 16;
// Near the wall the froth fades into the colour of the water just under the
// surface, which is also the colour of the curl tongue at its base, so the
// tongue grows out of the water body instead of out of a white line.
// FOAM_SHADES steps from 0 (water, at the wall) to the last (full froth).
const int FOAM_SHADES = 8;
const float WALL_FADE_PX = 50.0f;  // width of the fade, measured from the wall
uint16_t foamShade[FOAM_SHADES];
uint16_t edgeColor[FOAM_SHADES][EDGE_LEVELS];  // each shade blended into the black sky, for anti-aliasing
uint16_t dropletColor;
// The curl tongue is the fluid itself: it starts in the colour just under the
// surface (so its overlapping blobs merge into one smooth shape with the
// body), turns to the sheet colour as it thins and climbs, then to the spray
// colour as it flies apart. Falling pieces are always spray coloured.
const int CURL_SHADES = 12;
uint16_t curlShade[CURL_SHADES];
const int CURL_FALLING_SHADE = CURL_SHADES - 1;
const uint16_t SKY_COLOR = 0;

uint16_t rgb565(float r, float g, float b) {
  auto c = [](float v, float maxv) { int i = (int)(v + 0.5f); return i < 0 ? 0 : (i > maxv ? (int)maxv : i); };
  return (uint16_t)((c(r, 31) << 11) | (c(g, 63) << 5) | c(b, 31));
}
uint16_t toPanel(const Rgb &c) {  // 0..255 components -> panel colour
  return panelColor(rgb565(c.r * 31 / 255, c.g * 63 / 255, c.b * 31 / 255));
}
Rgb blend(const Rgb &a, const Rgb &b, float t) {
  return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t};
}

void buildColors(const Material &m) {
  for (int d = 0; d < MAX_DEPTH; d++) {
    // Surface colour just under the surface, deepening to `deep`.
    float t = fminf(d / m.depthRange, 1.0f);
    float near = 1 - t;
    float r = m.deep.r + (m.surface.r - m.deep.r) * powf(near, m.curve.r);  // 0..255
    float g = m.deep.g + (m.surface.g - m.deep.g) * powf(near, m.curve.g);
    float b = m.deep.b + (m.surface.b - m.deep.b) * powf(near, m.curve.b);
    for (int k = 0; k < 2; k++) {
      float off = k ? 0.25f : -0.25f;  // +-1/4 of a step: the two dither phases
      depthColor[k][d] = panelColor(rgb565(r * 31 / 255 + off, g * 63 / 255 + off, b * 31 / 255 + off));
    }
  }
  // Curl shades: surface -> sheet -> spray.
  for (int c = 0; c < CURL_SHADES; c++) {
    float t = (float)c / (CURL_SHADES - 1) * 2;  // 0..1 surface->sheet, 1..2 sheet->spray
    curlShade[c] = toPanel(t <= 1 ? blend(m.surface, m.sheet, t) : blend(m.sheet, m.spray, t - 1));
  }
  // Froth shades: surface colour at the wall -> froth.
  for (int f = 0; f < FOAM_SHADES; f++) {
    Rgb c = blend(m.surface, m.froth, (float)f / (FOAM_SHADES - 1));
    foamShade[f] = toPanel(c);
    for (int k = 0; k < EDGE_LEVELS; k++) {
      float a = (k + 0.5f) / EDGE_LEVELS;  // share of the pixel covered by water
      edgeColor[f][k] = toPanel({c.r * a, c.g * a, c.b * a});
    }
  }
  dropletColor = toPanel(m.droplet);
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
  for (int i = 0; i < N; i++) {
    float u = -RADIUS + i * DX;
    float w2 = RADIUS * RADIUS - u * u;
    bed_[i] = -(w2 > 0 ? sqrtf(w2) : 0);
  }
  setMaterial(WATER);
  rngState ^= esp_random();
}

void SideView::setMaterial(const Material &m) {
  m_ = m;
  g_ = m.waveSpeed * m.waveSpeed / m.depth;  // so that waves travel at c = sqrt(G * depth)
  gBowl_ = m.waveSpeed * m.waveSpeed / CENTRE_DEPTH;  // same speed at the bowl's centre at rest
  buildColors(m);
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

void SideView::setBowlModel(bool on) {
  bowl_ = on;
  reset();
}

void SideView::kickFlow(int face, float dv) {
  if (face < 1 || face > N - 1) return;
  if (!bowl_) {
    q_[face] += dv;
    return;
  }
  // The face sits between cells face-1 and face: speed both up.
  for (int c = face - 1; c <= face; c++) {
    if (dep_[c] > WET) mom_[c] += dep_[c] * dv * 0.5f;
  }
}

void SideView::reset() {
  for (int i = 0; i < N; i++) h_[i] = foam_[i] = 0;
  for (int j = 0; j <= N; j++) q_[j] = 0;
  if (bowl_) {
    for (int i = 0; i < N; i++) {
      dep_[i] = fmaxf(restLevel_ - bed_[i], 0.0f);
      mom_[i] = 0;
    }
    curlWater_[0] = curlWater_[1] = 0;
    climb_[0] = climb_[1] = 0;
    edgeV_[0] = edgeV_[1] = restLevel_;
    syncBowl();
    for (int i = 0; i < N; i++) hPrev_[i] = h_[i];
  }
  slowSide_ = swell_ = swellTarget_ = 0;
  quietTime_ = stillTime_ = 0;
  damping_ = m_.damping;
  particleBudget_ = PARTICLE_BURST;
  for (auto &p : particles_) p.kind = 0;
  for (int i = 0; i < N; i++) {
    hPrev_[i] = h_[i];
    rate_[i] = 0;
  }
  dotCount_ = 0;
}

void SideView::splash(float jolt) {
  // The square root makes small taps visible without big bangs exploding.
  float strength = fminf(m_.splashGain * sqrtf(jolt - JOLT_THRESHOLD), m_.maxSplash);
  // Kick two of the bowl's natural sloshing shapes (standing waves that fit
  // between the walls), each with a random strength and direction. That
  // looks like water being thrown about, rather than random noise.
  for (int n = 0; n < 2; n++) {
    int mode = 1 + (int)(random01() * 5);  // 1..5 half-waves across the bowl
    float amp = strength * (0.4f + 0.6f * random01()) * (random01() < 0.5f ? -1 : 1);
    for (int j = 1; j < N; j++) {
      kickFlow(j, amp * sinf(PI * mode * j / N));
    }
  }
  float foam = fminf(strength / m_.maxSplash, 1.0f);
  for (int i = 0; i < N; i++) foam_[i] = fmaxf(foam_[i], foam * (0.5f + 0.5f * random01()));

  // A burst of droplets thrown up from random spots on the surface.
  int drops = (int)(m_.splashDroplets * foam);
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

void SideView::returnWater(float u, float vol) {
  if (!bowl_ || vol <= 0) return;
  int c = (int)((u + RADIUS) / DX + 0.5f);
  if (c < 0) c = 0;
  if (c > N - 1) c = N - 1;
  dep_[c] += vol / DX;
}

void SideView::emit(float u, float v, float vu, float vv, uint8_t kind, float life, float vol) {
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
  if (particles_[slot].kind) returnWater(particles_[slot].u, particles_[slot].vol);  // reused mid-flight
  particles_[slot] = {u, v, vu, vv, life, kind, vol};
}

void SideView::emitWallCurl(int side, float dt) {
  // Where does the surface meet the wall on this side? The wall at height v
  // is at |u| = sqrt(R^2 - v^2); a few iterations converge.
  if (bowl_) {
    emitBowlCurl(side, dt);
    return;
  }
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
  if (rise * m_.curlSpeedGain < m_.curlMinSpeed) return;

  // Tangent to the wall, pointing up: (-v, u)/R on the right, (v, -u)/R on the left.
  float tu = side > 0 ? -v / RADIUS : v / RADIUS;
  float tv = side > 0 ? u / RADIUS : -u / RADIUS;
  float speed = fminf(rise * m_.curlSpeedGain, m_.curlMaxSpeed);
  float count = rise * m_.curlEmitRate * (dt * 60);  // about the same per second at any FPS
  int n = (int)count + (random01() < count - (int)count ? 1 : 0);
  // Inward normal (towards the bowl centre) at the contact point.
  float nu = -u / RADIUS, nv = -v / RADIUS;
  for (int k = 0; k < n; k++) {
    float depth = random01();  // 0 = against the wall, 1 = innermost
    float s = speed * (1 - m_.curlInnerSlowdown * depth) * (0.95f + 0.1f * random01());
    float in = 1.5f + depth * m_.curlThickness;
    float along = random01() * 4;  // spread a little along the wall
    emit(u + nu * in + tu * along, v + nv * in + tv * along,
         tu * s, tv * s, 1, m_.curlLife);
  }
  if (n > 0) consumeWallFoam(side);
}

void SideView::consumeWallFoam(int side) {
  int cells = (int)(CURL_CONSUMES_PX / DX) + 1;
  for (int k = 0; k < cells; k++) foam_[side > 0 ? N - 1 - k : k] = 0;
  for (int k = 0; k < MAX_PARTICLES; k++) {
    Particle &p = particles_[k];
    if (p.kind == 3 && side * p.u > RADIUS - CURL_CONSUMES_PX) {
      p.kind = 0;  // flecks riding the surface there are swept up too
    }
  }
}

void SideView::emitBowlCurl(int side, float dt) {
  int s = side > 0 ? 1 : 0;
  // Where the water's edge is on this wall (before anything is skimmed off),
  // and how fast it's climbing. While it's past the equator, keep the speed
  // it arrived with (easing off), since that's the speed of the jet.
  int edge = side > 0 ? N - 1 : 0;
  while (edge != N / 2 && dep_[edge] <= WET) edge -= side;
  float e = bed_[edge] + dep_[edge];
  if (dt > 0) {
    if (e < -2) {
      float a = 1.0f - expf(-dt / CLIMB_SMOOTHING);
      climb_[s] += ((e - edgeV_[s]) / dt - climb_[s]) * a;
    } else {
      climb_[s] *= expf(-dt / 0.3f);
    }
  }
  edgeV_[s] = fminf(e, 0.0f);  // the skimming below caps it at the equator
  // Water above the wall in the outermost cells: the wall at u reaches up to
  // +sqrt(R^2 - u^2) (0 at the equator), so anything higher would be under
  // the overhang. Take it out of the body.
  float took = 0;
  for (int k = 0; k < OVERHANG_CELLS; k++) {
    int c = side > 0 ? N - 1 - k : k;
    float u = -RADIUS + c * DX;
    float w2 = RADIUS * RADIUS - u * u;
    float top = w2 > 0 ? sqrtf(w2) : 0;
    if (restLevel_ > top) continue;  // bowl filled past here by leaning back: not a curl
    float excess = bed_[c] + dep_[c] - top;
    if (excess <= 0) continue;
    mom_[c] *= (dep_[c] - excess) / dep_[c];
    dep_[c] -= excess;
    took += excess * DX;
  }
  if (took > 0) {
    // Remember the speed of the water that left, weighted by how much.
    float speed = fminf(fmaxf(climb_[s] * m_.curlSpeedGain * BOWL_LAUNCH_BOOST, BOWL_MIN_LAUNCH), m_.curlMaxSpeed);
    curlSpeed_[s] = (curlSpeed_[s] * curlWater_[s] + speed * took) / (curlWater_[s] + took);
    curlWater_[s] += took;
  }
  if (curlWater_[s] < CURL_PIECE_WATER) return;

  // Launch from the equator on this side, straight up the wall; the wall
  // bends the pieces inward as they climb.
  float u = side * (RADIUS - 2);
  float nu = -(float)side;  // inward
  int pieces = (int)(curlWater_[s] / CURL_PIECE_WATER);
  if (pieces > 0) consumeWallFoam(side);
  for (int k = 0; k < pieces; k++) {
    if (particleBudget_ < 1) break;
    float depth = random01();  // 0 = against the wall, 1 = innermost
    float sp = curlSpeed_[s] * (1 - m_.curlInnerSlowdown * depth) * (0.95f + 0.1f * random01());
    float in = 1.5f + depth * m_.curlThickness;
    emit(u + nu * in, random01() * 4, 0, sp, 1, m_.curlLife, CURL_PIECE_WATER);
    curlWater_[s] -= CURL_PIECE_WATER;
  }
  // Out of particle budget: don't let water pile up unseen; give it back.
  if (curlWater_[s] > 4 * CURL_PIECE_WATER) {
    returnWater(u, curlWater_[s]);
    curlWater_[s] = 0;
  }
}

void SideView::emitCrestSpray(float dt) {
  int emitted = 0;
  for (int i = 2; i < N - 2 && emitted < MAX_SPRAY_PER_FRAME; i++) {
    float rise = rate_[i];
    if (rise < m_.sprayMinRise) continue;
    if (h_[i] < h_[i - 1] || h_[i] < h_[i + 1]) continue;  // only from crests
    float chance = (rise - m_.sprayMinRise) * m_.sprayChance * (dt * 60);
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
      returnWater(p.u, p.vol);
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
    p.vv -= m_.particleGravity * dt;
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
        float slide = expf(-m_.wallFriction * dt);
        float tvu = p.vu - vn * nu, tvv = p.vv - vn * nv;
        p.vu = tvu * slide - vn * m_.wallBounce * nu;
        p.vv = tvv * slide - vn * m_.wallBounce * nv;
      }
    }

    // Back into the water. Falling onto the surface: land as a fleck that
    // rides it. Swallowed by a rising wave from below: just gone.
    float surface = surfaceAt(p.u);
    if (p.vv < 0 && p.v < surface) {
      int cell = (int)((p.u + RADIUS) / DX + 0.5f);
      if (cell >= 0 && cell < N) foam_[cell] = fmaxf(foam_[cell], m_.landingFoam);
      returnWater(p.u, p.vol);
      p.vol = 0;
      p.kind = 3;
      p.life = m_.fleckLife;
      p.v = surface;
      p.vu = p.vv = 0;
    } else if (p.v < surface - 2) {
      returnWater(p.u, p.vol);
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
      float young = 1 - (m_.curlLife - p.life) / m_.curlBlobTime;
      d.r = (uint8_t)(m_.curlSheetR + (young > 0 ? (int)(young * (m_.curlBlobR - m_.curlSheetR) + 0.5f) : 0));
      // Glassy water on the way up, paling with age; once it's falling it
      // has broken up into spray, so it turns white (otherwise a young piece
      // falling back is water-coloured and seems to land behind the surface).
      float age = (m_.curlLife - p.life) / m_.curlShadeTime;
      int shade = age >= 1 ? CURL_SHADES - 1 : (int)(age * (CURL_SHADES - 1));
      if (p.vv < 0 && shade < CURL_FALLING_SHADE) shade = CURL_FALLING_SHADE;
      d.color = curlShade[shade];
    } else if (p.kind == 3) {
      d.r = p.life > m_.fleckLife / 2 ? 2 : 1;  // shrinks as it fades
      d.color = dropletColor;
    } else {
      d.r = 1 + (k & 1);
      d.color = dropletColor;
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

  // Boost quick small changes (see FAST_SMOOTHING and Material::sensitivity).
  slowSide_ += (side - slowSide_) * (1.0f - expf(-dt / FAST_SMOOTHING));
  float change = side - slowSide_;
  float mag = fabsf(change) - DEAD_ZONE;
  float boost = mag > 0 ? fminf(m_.sensitivity * sqrtf(mag), MAX_BOOST) : 0;
  if (change < 0) boost = -boost;
  if (fabsf(boost) > ACTIVE_BOOST) quietTime_ = 0;  // the necklace is really moving

  // Ambient swell: drift toward a random target that changes now and then.
  swellTimer_ -= dt;
  if (swellTimer_ <= 0) {
    swellTimer_ = SWELL_PERIOD * (0.6f + 0.8f * random01());
    swellTarget_ = (random01() * 2 - 1) * m_.swellAmplitude;
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
    dropTimer_ = m_.dropInterval * (0.3f + 1.4f * random01());
    int j = 1 + (int)(random01() * (N - 2));
    float strength = m_.dropStrength * (0.5f + random01());
    if (m_.bloopSpeed > 0) {
      // A gas bubble: flow into cell j from both sides lifts the surface
      // there (spreading as a tiny ring of waves), and a small blob pops out.
      kickFlow(j, strength);
      kickFlow(j + 1, -strength);
      float u = -RADIUS + j * DX;
      int blobs = 1 + (random01() < 0.4f ? 1 : 0);
      for (int b = 0; b < blobs; b++) {
        // Start part-way through the blob's life, so it's already small.
        emit(u + (random01() * 2 - 1) * 3, surfaceAt(u) + 1, (random01() * 2 - 1) * 25,
             m_.bloopSpeed * (0.7f + 0.4f * random01()), 1, m_.curlLife - m_.curlBlobTime * 0.7f);
      }
    } else {
      kickFlow(j, (random01() < 0.5f ? -1 : 1) * strength);
    }
  }

  float aSide = softClip(side + boost + swell_) * (bowl_ ? gBowl_ : g_);

  // Churning water loses energy faster: extra damping above CALM_RMS.
  // (Bowl model: only where there's water; dry cells sit at the floor.)
  float sumSq = 0;
  int counted = 0;
  for (int i = 0; i < N; i++) {
    if (bowl_ && dep_[i] <= WET) continue;
    sumSq += h_[i] * h_[i];
    counted++;
  }
  float rms = counted ? sqrtf(sumSq / counted) : 0;
  float churn = fminf(fmaxf((rms - CALM_RMS) / (CHURN_RMS - CALM_RMS), 0.0f), 1.0f);
  damping_ = m_.damping * (bowl_ ? BOWL_DAMPING_SCALE : 1.0f) + TURBULENT_DAMPING * churn;

  // Refill the particle budget.
  particleBudget_ = fminf(particleBudget_ + PARTICLES_PER_SECOND * dt, PARTICLE_BURST);

  if (bowl_) {
    fillBowl();
    // Substeps sized so no wave crosses more than BOWL_CFL of a cell.
    float amax = 1;
    for (int i = 0; i < N; i++) {
      if (dep_[i] <= 0) continue;
      float a = fabsf(mom_[i] / dep_[i]) + sqrtf(gBowl_ * dep_[i]);
      if (a > amax) amax = a;
    }
    int n = (int)ceilf(dt * amax / (BOWL_CFL * DX));
    if (n < 1) n = 1;
    if (n > MAX_BOWL_SUBSTEPS) n = MAX_BOWL_SUBSTEPS;
    for (int k = 0; k < n; k++) stepBowl(dt / n, aSide);
    syncBowl();
  } else {
    for (float t = 0; t < dt; t += SUBSTEP) {
      step(fminf(SUBSTEP, dt - t), aSide);
    }
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
    if (bowl_) syncBowl();  // curl pieces took water out or gave it back
  }

  // How long the water has been visibly still (all flow slow).
  float maxFlow = 0;
  for (int j = 1; j < N; j++) maxFlow = fmaxf(maxFlow, fabsf(q_[j]));
  stillTime_ = maxFlow < STILL_FLOW ? stillTime_ + dt : 0;

  updateFoam(dt);
  prepareRender();
  prepareParticles();
}

void SideView::fillBowl() {
  // The level rises with lean (setLean): add or remove water to match the
  // volume a flat surface at restLevel_ would hold, spread over the wet cells.
  // Water in flight (curl pieces, and overflow about to leave) still counts.
  float target = 0, now = (curlWater_[0] + curlWater_[1]) / DX;
  for (int k = 0; k < MAX_PARTICLES; k++) {
    if (particles_[k].kind) now += particles_[k].vol / DX;
  }
  int wet = 0;
  for (int i = 0; i < N; i++) {
    target += fmaxf(restLevel_ - bed_[i], 0.0f);
    now += dep_[i];
    if (dep_[i] > WET) wet++;
  }
  float diff = target - now;
  if (fabsf(diff) < 0.01f || wet == 0) return;
  float each = diff / wet;
  for (int i = 0; i < N; i++) {
    if (dep_[i] > WET) dep_[i] = fmaxf(dep_[i] + each, 0.0f);
  }
}

void SideView::syncBowl() {
  // Everything else (rendering, curl, foam, stillness) reads the surface as
  // h_ (relative to the rest level) and flow speeds at the faces as q_.
  for (int i = 0; i < N; i++) h_[i] = bed_[i] + dep_[i] - restLevel_;
  q_[0] = q_[N] = 0;
  for (int j = 1; j < N; j++) {
    float wl = dep_[j - 1] > WET ? mom_[j - 1] / dep_[j - 1] : 0;
    float wr = dep_[j] > WET ? mom_[j] / dep_[j] : 0;
    q_[j] = 0.5f * (wl + wr);
  }
}

void SideView::stepBowl(float dt, float aSide) {
  const float g = gBowl_;
  // Fluxes at the N+1 faces. Face j sits between cells j-1 and j; beyond
  // each wall is a mirror-image cell (same depth, opposite flow), so the
  // walls reflect.
  float fluxD[N + 1], fluxMLeft[N + 1], fluxMRight[N + 1];
  for (int j = 0; j <= N; j++) {
    int l = j > 0 ? j - 1 : 0, r = j < N ? j : N - 1;
    float dl = dep_[l], dr = dep_[r];
    float wl = dl > 1e-3f ? mom_[l] / dl : 0;
    float wr = dr > 1e-3f ? mom_[r] / dr : 0;
    if (j == 0) wl = -wr;      // mirror at the left wall
    if (j == N) wr = -wl;      // mirror at the right wall
    // Hydrostatic reconstruction: depths seen at the face, over the higher floor.
    float bStar = fmaxf(bed_[l], bed_[r]);
    float hl = fmaxf(bed_[l] + dl - bStar, 0.0f);
    float hr = fmaxf(bed_[r] + dr - bStar, 0.0f);
    // Rusanov flux: the average of both sides' fluxes, plus diffusion at the
    // fastest local wave speed.
    float a = fmaxf(fabsf(wl) + sqrtf(g * hl), fabsf(wr) + sqrtf(g * hr));
    float fd = 0.5f * (hl * wl + hr * wr) - 0.5f * a * (hr - hl);
    float fm = 0.5f * (hl * wl * wl + 0.5f * g * hl * hl + hr * wr * wr + 0.5f * g * hr * hr) -
               0.5f * a * (hr * wr - hl * wl);
    fluxD[j] = fd;
    // The floor's push, folded into the momentum flux on each side.
    fluxMLeft[j] = fm + 0.5f * g * (dl * dl - hl * hl);   // as seen by cell j-1
    fluxMRight[j] = fm + 0.5f * g * (dr * dr - hr * hr);  // as seen by cell j
  }
  const float keep = 1.0f - damping_ * dt;
  const float k = dt / DX;
  for (int i = 0; i < N; i++) {
    float d = dep_[i] - k * (fluxD[i + 1] - fluxD[i]);
    float m = mom_[i] - k * (fluxMLeft[i + 1] - fluxMRight[i]);
    if (d < 1e-3f) {
      d = fmaxf(d, 0.0f);
      m = 0;
    } else {
      m = (m + dt * d * aSide) * keep;
      // Thin films at the shoreline can get absurd speeds: cap them.
      float lim = d * MAX_FLOW_SPEED;
      if (m > lim) m = lim;
      if (m < -lim) m = -lim;
    }
    dep_[i] = d;
    mom_[i] = m;
  }
}

void SideView::step(float dt, float aSide) {
  // Flow speeds up downhill and with the sideways push.
  float keep = 1.0f - damping_ * dt;
  for (int j = 1; j < N; j++) {
    float slope = (h_[j] - h_[j - 1]) / DX;
    q_[j] = (q_[j] + dt * (-g_ * slope + aSide)) * keep;
  }
  q_[0] = q_[N] = 0;  // walls

  // Water moves between cells.
  for (int i = 0; i < N; i++) {
    h_[i] -= dt * m_.depth * (q_[i + 1] - q_[i]) / DX;
  }

  // Numerical diffusion (Material::smoothing). The walls mirror, so no water is
  // lost or gained.
  float prev = h_[0];
  for (int i = 0; i < N; i++) {
    float left = prev;
    float right = h_[i < N - 1 ? i + 1 : i];
    prev = h_[i];
    float hi = h_[i] + m_.smoothing * (left + right - 2 * h_[i]);
    h_[i] = hi > MAX_HEIGHT ? MAX_HEIGHT : (hi < -MAX_HEIGHT ? -MAX_HEIGHT : hi);
  }
}

void SideView::updateFoam(float dt) {
  // Foam floats: carry it with the flow first. Each cell takes the foam from
  // where its water was a moment ago (upstream), so a swell running up the
  // wall carries foam with it, and foam gathers at the water's edge.
  float carried[N];
  for (int i = 0; i < N; i++) {
    float w = 0.5f * (q_[i] + q_[i + 1]);       // flow speed at the cell, px/s
    float from = i - w * dt / DX;               // where this water came from, in cells
    if (from <= 0) carried[i] = foam_[0];
    else if (from >= N - 1) carried[i] = foam_[N - 1];
    else {
      int j = (int)from;
      float f = from - j;
      carried[i] = foam_[j] + (foam_[j + 1] - foam_[j]) * f;
    }
  }
  float decay = expf(-dt / m_.foamFadeTime);
  for (int i = 0; i < N; i++) {
    foam_[i] = carried[i];
    // Fast flow and sharp crests whip up foam.
    float flow = fabsf(q_[i] + q_[i + 1]) * 0.5f;
    float left = h_[i > 0 ? i - 1 : i], right = h_[i < N - 1 ? i + 1 : i];
    float crest = h_[i] - 0.5f * (left + right);  // positive at a peak
    float target = flow * m_.foamFlowGain + (crest > 0 ? crest * m_.foamCrestGain : 0);
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
    float u = (float)(k - TABLE_OFFSET);
    float v = restLevel_ + h;
    foamPx_[k] = (uint8_t)(1 + foam * m_.foamMaxPx + 0.5f);  // always at least a thin line
    // Dry bowl floor (the surface down at the circle): no water, no foam
    // line and no anti-aliased edge tracing the empty wall. Push the surface
    // just outside the circle.
    float f2 = RADIUS * RADIUS - u * u;
    float floorV = -(f2 > 0 ? sqrtf(f2) : 0);
    if (v < floorV + WET) {
      v = floorV - 2;
      foamPx_[k] = 0;
    }
    surfaceQ_[k] = (int32_t)(v * 65536.0f);

    // Distance to the wall at the surface's height (the wall is a circle,
    // so at height v it sits at |u| = sqrt(R^2 - v^2)), for the froth fade.
    float w2 = RADIUS * RADIUS - v * v;
    float wallDist = (w2 > 0 ? sqrtf(w2) : 0) - fabsf(u);
    float t = wallDist / WALL_FADE_PX;
    // The fade is for the thin surface line (so the curl grows out of the
    // water); real foam carried to the wall stays white.
    t = fmaxf(t, foam * 2.0f);
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
