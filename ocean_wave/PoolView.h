// Top-down view of the "fish bowl": what you see when the necklace lies flat
// face up. The whole circle is water; ripples spread from touches and bumps, reflect off the round wall and fade.
//
// Physics: the 2D wave equation on a grid of 4 px cells.
// Rendering: each cell's shade comes from the slope of the surface (lit from
// the upper left), smoothly interpolated between cells for every pixel.
#pragma once
#include <NecklaceBoard.h>

class PoolView {
 public:
  void begin();

  // Calm water. Call when the pool appears; bumps are ignored for a moment
  // while the necklace is being set down.
  void reset();

  // dt: seconds since the last update. jolt: biggest sudden change in
  // acceleration since the last update, in g. gx, gy: gravity in screen
  // coordinates (small when flat): its direction sets which way is "down".
  void update(float dt, float jolt, float gx, float gy);

  // Start from this "down" direction (screen coordinates), e.g. the side
  // view's when switching over, so the colours carry on seamlessly.
  void setDown(float x, float y);

  // A finger on the screen at (x, y) in pixels. Call every frame while it's
  // down: a new touch drops one ripple.
  void touch(int16_t x, int16_t y);
  void release() { touching_ = false; }

  // NecklaceBoard::StripRenderer: pass the PoolView as ctx.
  static void renderStrip(uint16_t *pixels, int16_t y0, int16_t rows, void *ctx);

 private:
  static constexpr int CELL = 4;                       // px per cell
  static constexpr int GRID = NecklaceBoard::WIDTH / CELL + 1;  // 91 cells across, centres at 0, 4 .. 360 px
  static constexpr int CENTRE = GRID / 2;              // cell 45 = the bowl centre
  static constexpr int RADIUS_CELLS = CENTRE - 1;      // water stops one cell inside the rim

  void step();
  void drop(float cx, float cy, float radius, float strength);  // cell coordinates
  void ring(float strength);   // a ripple from the whole rim (a bang on the table)
  void prepareRender();
  void render(uint16_t *pixels, int16_t y0, int16_t rows) const;

  float h_[GRID][GRID] = {};      // surface height now
  float prev_[GRID][GRID] = {};   // surface height one step ago
  bool inside_[GRID][GRID] = {};  // cells holding water
  int16_t base_[GRID][GRID] = {}; // resting shade (depth gradient along "down"), 8.8 fixed
  int16_t shade_[GRID][GRID] = {};// this frame's shade, 8.8 fixed (palette index * 256)

  float stepTimer_ = 0;
  float settleTimer_ = 0;         // s left in which bumps are ignored
  float touchQuiet_ = 0;          // s left in which bumps are ignored after a tap
  float pendingJolt_ = 0;         // last frame's jolt, held back one frame
  bool touching_ = false;
  float downX_ = 0, downY_ = 1;   // which way is "down" on screen (unit vector)
  float tipX_ = 0, tipY_ = 0.01f; // smoothed in-plane gravity, g
};
