// Screen scale. All tuning (Material.cpp, the bowl constants in SideView.cpp
// and PoolView.cpp, gesture distances) is in "design pixels": pixels of the
// 360x360 screen it was tuned on (the 1.85" LCD). The physics runs in design
// pixels on every board; only drawing and touch input are converted to real
// screen pixels with SCALE. On the 1.85" SCALE is exactly 1.
//
// The boards' screens are nearly the same physical size (1.85" LCD: 47 mm,
// 1.75" AMOLED: 44.5 mm), so scaling by pixels keeps things the same size
// to the eye.
#pragma once
#include <NecklaceBoard.h>

constexpr float DESIGN_WIDTH = 360.0f;
constexpr float SCALE = NecklaceBoard::WIDTH / DESIGN_WIDTH;
