// Material: everything that makes the fluid in the bowl look and move like
// water (or lava): wave physics, splashes, the curl, foam and colours, for
// both the side view and the pool.
//
// Only the fluid lives here. The bowl itself (grid sizes, step sizes, the
// lean levels, safety limits) stays as constants in SideView/PoolView.
//
// Every field is a float, so two materials can be blended field by field
// (Material::mix), e.g. for a gradual switch from water to lava.
// See things-I-learned.md ("How the tuning variables relate") before tuning:
// wave speed in particular changes splash height and the curl.
#pragma once

struct Rgb {
  float r, g, b;  // 0..255
};

struct Material {
  // ---- Side view: the wave model ----
  float waveSpeed;        // px/s: how fast waves cross the bowl (the hub, see above)
  float depth;            // px: effective depth in the wave equation (with waveSpeed sets G)
  float damping;          // 1/s: how quickly sloshing dies down
  float smoothing;        // numerical diffusion per substep: damps grid-sized ripples
  float sensitivity;      // boost for small quick motions = this * sqrt(|change| in g)
  float swellAmplitude;   // g: lazy drifting push, so still water isn't frozen
  float dropStrength;     // px/s of flow from an ambient drop in still water
  float dropInterval;     // s: average time between ambient drops (once it's still)
  float bloopSpeed;       // px/s: a gas bubble pops, throwing a blob this fast (0 = plain drops)

  // ---- Side view: splashes from jolts ----
  float splashGain;       // px/s of flow per sqrt(g) of jolt
  float maxSplash;        // px/s
  float splashDroplets;   // droplets thrown up by the biggest splash

  // ---- Side view: the wall curl and spray ----
  float particleGravity;  // px/s^2: how fast pieces fall (and how high they fly)
  float curlSpeedGain;    // launch speed = wall rise rate * this
  float curlMinSpeed;     // px/s: slower climbs make no tongue
  float curlMaxSpeed;     // px/s: just under the loop-the-loop limit sqrt(gravity * radius)
  float curlEmitRate;     // pieces per frame per px/s of rise
  float curlThickness;    // px: how far in from the wall the tongue starts
  float curlInnerSlowdown;// innermost pieces launch this much slower (the tongue stretches)
  float curlLife;         // s
  float curlBlobTime;     // s: pieces shrink from fat to thin over this
  float curlBlobR;        // px: radius at launch
  float curlSheetR;       // px: radius once stretched out
  float curlShadeTime;    // s: from body colour to spray colour
  float sprayMinRise;     // px/s: crests rising faster throw droplets
  float sprayChance;      // per frame, per px/s above the minimum
  float wallFriction;     // 1/s while pieces slide along the wall
  float wallBounce;       // share of speed kept bouncing off the wall

  // ---- Side view: foam ----
  float foamMaxPx;        // foam line: 1 px + up to this much more
  float foamFlowGain;     // foam from fast flow, per px/s
  float foamCrestGain;    // foam from sharp crests, per px of crest
  float foamFadeTime;     // s
  float landingFoam;      // foam a landing piece leaves (0..1)
  float fleckLife;        // s a landed piece rides the surface

  // ---- Colours ----
  // Body colour by depth: near = 1 just under the surface .. 0 at depthRange
  // and deeper. Each channel blends deep -> surface by near^curve.
  Rgb deep, surface, curve;
  float depthRange;       // px
  Rgb froth;              // the foam line (fades into `surface` near the walls)
  Rgb sheet;              // the curl as it thins
  Rgb spray;              // falling pieces
  Rgb droplet;            // droplets and flecks
  Rgb poolShadow;         // pool: slopes facing away from the light (darker than deep)
  Rgb poolGlint;          // pool: steep slopes facing the light

  // ---- Pool (top-down view) ----
  float rippleSpeed;      // px/s
  float rippleDamping;    // 1/s
  float lightGain;        // palette steps per px of height difference
  float touchDrop;        // px: ripple from a tap
  float ringGain;         // px per sqrt(g) of a sharp jab
  float poolBloop;        // px: a gas bubble now and then in the still pool (0 = none)

  // Blend: t = 0 gives a, t = 1 gives b.
  static Material mix(const Material &a, const Material &b, float t);
};

extern const Material WATER;
extern const Material LAVA;
