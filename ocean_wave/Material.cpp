#include <stddef.h>
#include <math.h>
#include "Material.h"

// Every field is a float (Rgb is three floats), so a Material is an array of
// floats as far as blending is concerned.
static_assert(sizeof(Material) % sizeof(float) == 0, "Material must contain only floats");

Material Material::mix(const Material &a, const Material &b, float t) {
  Material out;
  const float *pa = reinterpret_cast<const float *>(&a);
  const float *pb = reinterpret_cast<const float *>(&b);
  float *po = reinterpret_cast<float *>(&out);
  for (size_t i = 0; i < sizeof(Material) / sizeof(float); i++) po[i] = pa[i] + (pb[i] - pa[i]) * t;
  return out;
}

// Water: slow and stately (the tuning history is in things-I-learned.md).
const Material WATER = {
    // wave model
    /*waveSpeed*/ 165, /*depth*/ 100, /*damping*/ 0.9f, /*smoothing*/ 0.06f,
    /*sensitivity*/ 0.35f, /*swellAmplitude*/ 0.012f, /*dropStrength*/ 30,
    // splashes
    /*splashGain*/ 90, /*maxSplash*/ 110, /*splashDroplets*/ 25,
    // curl and spray
    /*particleGravity*/ 1000, /*curlSpeedGain*/ 2.8f, /*curlMinSpeed*/ 220, /*curlMaxSpeed*/ 410,
    /*curlEmitRate*/ 0.04f, /*curlThickness*/ 12, /*curlInnerSlowdown*/ 0.35f,
    /*curlLife*/ 2.0f, /*curlBlobTime*/ 0.45f, /*curlBlobR*/ 6, /*curlSheetR*/ 2, /*curlShadeTime*/ 0.9f,
    /*sprayMinRise*/ 220, /*sprayChance*/ 0.0003f, /*wallFriction*/ 1.2f, /*wallBounce*/ 0.25f,
    // foam
    /*foamMaxPx*/ 3, /*foamFlowGain*/ 0.012f, /*foamCrestGain*/ 0.25f, /*foamFadeTime*/ 0.6f,
    /*landingFoam*/ 0.6f, /*fleckLife*/ 0.8f,
    // colours
    /*deep*/ {10, 30, 70}, /*surface*/ {50, 180, 240}, /*curve*/ {2, 1, 1}, /*depthRange*/ 190,
    /*froth*/ {205, 232, 252}, /*sheet*/ {140, 205, 245}, /*spray*/ {200, 235, 255},
    /*droplet*/ {240, 248, 255}, /*poolShadow*/ {2, 8, 25}, /*poolGlint*/ {245, 250, 255},
    // pool
    /*rippleSpeed*/ 160, /*rippleDamping*/ 0.8f, /*lightGain*/ 9, /*touchDrop*/ 6, /*ringGain*/ 2,
};
