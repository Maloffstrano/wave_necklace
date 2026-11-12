// Side view of the "fish bowl": water sloshing in the lower part of the
// round display, with its surface kept level relative to gravity.
//
// Physics: linearized 1D shallow-water equations on a staggered grid.
//   - N cells across the bowl hold the surface height h (pixels, relative to the rest level)
//   - N+1 faces between them hold the horizontal flow velocity q (0 at both walls)
//   - each substep: q += dt * (-G * dh/du + a_side) - damping
//                   h -= dt * DEPTH * dq/du
// This conserves the water volume exactly, carries waves at c = sqrt(G * DEPTH)
// that reflect off the walls, and a steady sideways push settles into a tilted
// surface, which is what sloshing is.
//
// The water is drawn in a frame aligned with *smoothed* gravity. The raw
// accelerometer direction minus the smoothed one is the sideways push
// (a_side): rotate or jolt the necklace and the water sloshes, then settles.
#pragma once
#include <NecklaceBoard.h>
#include "Material.h"

class SideView {
 public:
  void begin();

  // The fluid: physics, splashes, curl, foam and colours (Material.h).
  // Rebuilds the colour tables, so it can be called every frame while
  // blending between materials.
  void setMaterial(const Material &m);

  // Calm water: clears all waves and foam.
  void reset();

  // gx, gy: gravity in screen coordinates (x right, y down), in g, raw from
  // the accelerometer. dt: seconds since the last update. jolt: the biggest
  // sudden change in acceleration since the last update, in g (any direction).
  void update(float gx, float gy, float dt, float jolt = 0);

  // Lean of the screen away from vertical, in degrees (smoothed): leaning
  // back raises the water level, as if looking into the bowl from above.
  void setLean(float leanDeg);

  // NecklaceBoard::StripRenderer: pass the SideView as ctx.
  static void renderStrip(uint16_t *pixels, int16_t y0, int16_t rows, void *ctx);

  // Start from this "down" direction (screen coordinates), e.g. the pool's
  // when switching back, so the colours carry on seamlessly.
  void setDown(float x, float y);

  // Smoothed "down" direction in screen coordinates (unit vector).
  float downX() const { return downX_; }
  float downY() const { return downY_; }

 private:
  static constexpr int N = 91;            // physics cells across the bowl
  static constexpr float RADIUS = 180.0f; // bowl radius in pixels
  static constexpr float DX = 2 * RADIUS / (N - 1);  // 4 px between cells

  void step(float dt, float aSide);
  void splash(float jolt);

  // ---- Spray and curl particles (see SideView.cpp) ----
  float surfaceAt(float u) const;  // surface height (v, px) at position u across the bowl
  void emit(float u, float v, float vu, float vv, uint8_t kind, float life);
  void emitWallCurl(int side, float dt);
  void emitCrestSpray(float dt);
  void updateParticles(float dt);
  void prepareParticles();
  void drawParticles(uint16_t *pixels, int16_t y0, int16_t rows) const;
  void updateFoam(float dt);
  void prepareRender();
  void render(uint16_t *pixels, int16_t y0, int16_t rows) const;

  Material m_ = WATER;  // the fluid
  float g_ = 0;         // wave model gravity, from m_ (waveSpeed^2 / depth)

  // Simulation state
  float h_[N] = {};        // surface height per cell, px above the rest level
  float q_[N + 1] = {};    // flow velocity at faces between cells, px/s
  float foam_[N] = {};     // foam amount per cell, 0..1
  float restLevel_ = -14.0f;     // current rest surface height, px (follows the lean)
  float downX_ = 0, downY_ = 1;  // smoothed gravity direction on screen
  float lastGx_ = 0, lastGy_ = 1;
  float slowSide_ = 0;          // slowly smoothed sideways push, g (for the sensitivity boost)
  float swell_ = 0, swellTarget_ = 0, swellTimer_ = 0;  // ambient swell, g
  float dropTimer_ = 1.0f;      // s until the next random disturbance
  float quietTime_ = 0;         // s since the last splash or real motion
  float stillTime_ = 0;         // s the water has been visibly still
  float damping_ = 0.9f;        // current damping, 1/s (rises when the water churns)
  float particleBudget_ = 120;  // particles that may be emitted right now
  float hPrev_[N] = {};         // surface heights last update, for the rise rate
  float rate_[N] = {};          // how fast each cell's surface is rising, px/s

  struct Particle {
    float u, v;    // position in the water frame (px): u across, v up
    float vu, vv;  // velocity, px/s
    float life;    // seconds left
    uint8_t kind;  // 0 = dead, 1 = curl sheet, 2 = droplet, 3 = fleck riding the surface
  };
  static constexpr int MAX_PARTICLES = 480;
  Particle particles_[MAX_PARTICLES] = {};
  int nextParticle_ = 0;
  // Screen-space copy for rendering
  struct Dot { int16_t x, y; uint8_t r; uint16_t color; };
  Dot dots_[MAX_PARTICLES];
  int dotCount_ = 0;

  // Per-frame render data, one entry per 1 px step across the bowl (u).
  // Wider than the bowl (+-256 px) so rotated corner pixels stay in range.
  static constexpr int TABLE_OFFSET = 256;
  static constexpr int TABLE_SIZE = 2 * TABLE_OFFSET;
  int32_t surfaceQ_[TABLE_SIZE] = {};  // surface height (v), 16.16 fixed point
  uint8_t foamPx_[TABLE_SIZE] = {};    // foam thickness, px
  uint8_t foamShade_[TABLE_SIZE] = {}; // froth colour: 0 = curl-sheet blue at the wall .. full froth
  int32_t hxQ_ = 0, hyQ_ = 0, dxQ_ = 0, dyQ_ = 0;  // water-frame axes
};
