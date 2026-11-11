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

class SideView {
 public:
  void begin();

  // Calm water: clears all waves and foam.
  void reset();

  // gx, gy: gravity in screen coordinates (x right, y down), in g, raw from
  // the accelerometer. dt: seconds since the last update. jolt: the biggest
  // sudden change in acceleration since the last update, in g (any direction).
  void update(float gx, float gy, float dt, float jolt = 0);

  // NecklaceBoard::StripRenderer: pass the SideView as ctx.
  static void renderStrip(uint16_t *pixels, int16_t y0, int16_t rows, void *ctx);

  // Smoothed "down" direction in screen coordinates (unit vector).
  float downX() const { return downX_; }
  float downY() const { return downY_; }

 private:
  static constexpr int N = 91;            // physics cells across the bowl
  static constexpr float RADIUS = 180.0f; // bowl radius in pixels
  static constexpr float DX = 2 * RADIUS / (N - 1);  // 4 px between cells

  void step(float dt, float aSide);
  void splash(float jolt);
  void updateFoam(float dt);
  void prepareRender();
  void render(uint16_t *pixels, int16_t y0, int16_t rows) const;

  // Simulation state
  float h_[N] = {};        // surface height per cell, px above the rest level
  float q_[N + 1] = {};    // flow velocity at faces between cells, px/s
  float foam_[N] = {};     // foam amount per cell, 0..1
  float downX_ = 0, downY_ = 1;  // smoothed gravity direction on screen
  float lastGx_ = 0, lastGy_ = 1;
  float slowSide_ = 0;          // slowly smoothed sideways push, g (for the sensitivity boost)
  float swell_ = 0, swellTarget_ = 0, swellTimer_ = 0;  // ambient swell, g
  float dropTimer_ = 1.0f;      // s until the next random disturbance
  float quietTime_ = 0;         // s since the last splash or real motion
  float stillTime_ = 0;         // s the water has been visibly still

  // Per-frame render data, one entry per 1 px step across the bowl (u).
  // Wider than the bowl (+-256 px) so rotated corner pixels stay in range.
  static constexpr int TABLE_OFFSET = 256;
  static constexpr int TABLE_SIZE = 2 * TABLE_OFFSET;
  int32_t surfaceQ_[TABLE_SIZE] = {};  // surface height (v), 16.16 fixed point
  uint8_t foamPx_[TABLE_SIZE] = {};    // foam thickness, px
  int32_t hxQ_ = 0, hyQ_ = 0, dxQ_ = 0, dyQ_ = 0;  // water-frame axes
};
