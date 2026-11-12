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
    .waveSpeed = 165, .depth = 100, .damping = 0.9f, .smoothing = 0.06f,
    .sensitivity = 0.35f, .swellAmplitude = 0.012f, .dropStrength = 30,
    .dropInterval = 1.8f, .bloopSpeed = 0,
    // splashes
    .splashGain = 90, .maxSplash = 110, .splashDroplets = 25,
    // curl and spray
    .particleGravity = 1000, .curlSpeedGain = 2.8f, .curlMinSpeed = 220, .curlMaxSpeed = 410,
    .curlEmitRate = 0.04f, .curlThickness = 12, .curlInnerSlowdown = 0.35f,
    .curlLife = 2.0f, .curlBlobTime = 0.45f, .curlBlobR = 6, .curlSheetR = 2, .curlShadeTime = 0.9f,
    .sprayMinRise = 220, .sprayChance = 0.0003f, .wallFriction = 1.2f, .wallBounce = 0.25f,
    // foam
    .foamMaxPx = 3, .foamFlowGain = 0.012f, .foamCrestGain = 0.25f, .foamFadeTime = 0.6f,
    .landingFoam = 0.6f, .fleckLife = 0.8f,
    // colours
    .deep = {10, 30, 70}, .surface = {50, 180, 240}, .curve = {2, 1, 1}, .depthRange = 190,
    .froth = {205, 232, 252}, .sheet = {140, 205, 245}, .spray = {200, 235, 255},
    .droplet = {240, 248, 255}, .poolShadow = {2, 8, 25}, .poolGlint = {245, 250, 255},
    // pool
    .rippleSpeed = 160, .rippleDamping = 0.8f, .lightGain = 9, .touchDrop = 6, .ringGain = 2, .poolBloop = 0,
};

// Lava: thick, slow and glowing. Waves at about 0.4x water's speed, so (see
// the rules of thumb) splash strength scales down with it; heavy damping and
// smoothing make it gloopy. No spray: the curl is a few fat, slow blobs that
// stick to the wall and plop back. Hot (bright) just under the surface,
// deepening to dark red.
const Material LAVA = {
    // wave model
    .waveSpeed = 70, .depth = 100, .damping = 2.5f, .smoothing = 0.2f,
    .sensitivity = 0.25f, .swellAmplitude = 0.006f, .dropStrength = 10,
    .dropInterval = 4.0f, .bloopSpeed = 130,
    // splashes
    .splashGain = 40, .maxSplash = 50, .splashDroplets = 4,
    // curl and spray
    .particleGravity = 700, .curlSpeedGain = 5, .curlMinSpeed = 180, .curlMaxSpeed = 345,
    .curlEmitRate = 0.03f, .curlThickness = 14, .curlInnerSlowdown = 0.2f,
    .curlLife = 2.5f, .curlBlobTime = 1.2f, .curlBlobR = 7, .curlSheetR = 4, .curlShadeTime = 1.5f,
    .sprayMinRise = 100000, .sprayChance = 0, .wallFriction = 4, .wallBounce = 0.05f,
    // foam (the glowing edge)
    .foamMaxPx = 2, .foamFlowGain = 0.02f, .foamCrestGain = 0.3f, .foamFadeTime = 1.5f,
    .landingFoam = 0.5f, .fleckLife = 1.5f,
    // colours
    .deep = {90, 5, 0}, .surface = {255, 150, 20}, .curve = {1, 1.6f, 3}, .depthRange = 220,
    .froth = {255, 225, 120}, .sheet = {255, 110, 10}, .spray = {255, 190, 70},
    .droplet = {255, 235, 170}, .poolShadow = {30, 0, 0}, .poolGlint = {255, 245, 190},
    // pool
    .rippleSpeed = 70, .rippleDamping = 2.5f, .lightGain = 14, .touchDrop = 8, .ringGain = 2.5f, .poolBloop = 1.5f,
};

// Mercury: a heavy liquid metal, between water and lava. Waves a little
// faster than water's (so, by the rules of thumb, splash gain a bit lower and
// curl gain a bit lower), settling quickly; surface tension smooths small
// ripples. It doesn't wet the glass: pieces slide easily and bounce off the
// wall, and the curl is a few round beads that stay round. No foam: the
// surface line is a thin bright highlight. Gunmetal deep down to bright
// silver near the surface; in the pool, ripples are lit strongly like a mirror.
const Material MERCURY = {
    // wave model
    .waveSpeed = 200, .depth = 100, .damping = 1.6f, .smoothing = 0.1f,
    .sensitivity = 0.3f, .swellAmplitude = 0.008f, .dropStrength = 25,
    .dropInterval = 2.5f, .bloopSpeed = 0,
    // splashes
    .splashGain = 70, .maxSplash = 85, .splashDroplets = 10,
    // curl and spray (beads)
    .particleGravity = 1300, .curlSpeedGain = 2.3f, .curlMinSpeed = 240, .curlMaxSpeed = 470,
    .curlEmitRate = 0.015f, .curlThickness = 8, .curlInnerSlowdown = 0.1f,
    .curlLife = 2.0f, .curlBlobTime = 0.3f, .curlBlobR = 4, .curlSheetR = 3, .curlShadeTime = 0.6f,
    .sprayMinRise = 260, .sprayChance = 0.0002f, .wallFriction = 0.5f, .wallBounce = 0.5f,
    // foam: just a thin highlight
    .foamMaxPx = 1, .foamFlowGain = 0.005f, .foamCrestGain = 0.1f, .foamFadeTime = 0.4f,
    .landingFoam = 0.2f, .fleckLife = 0.4f,
    // colours
    .deep = {38, 40, 46}, .surface = {190, 195, 205}, .curve = {1.5f, 1.5f, 1.3f}, .depthRange = 160,
    .froth = {240, 243, 248}, .sheet = {210, 214, 222}, .spray = {235, 238, 245},
    .droplet = {245, 247, 250}, .poolShadow = {8, 8, 12}, .poolGlint = {255, 255, 255},
    // pool
    .rippleSpeed = 200, .rippleDamping = 1.6f, .lightGain = 16, .touchDrop = 5, .ringGain = 2, .poolBloop = 0,
};
