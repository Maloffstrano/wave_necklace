# Things I Learned

Notes on surprising or interesting things discovered while building the wave necklace.

## How a "digital" accelerometer gets stuck (MEMS stiction)

*2026-11-11: the board's QMI8658 accelerometer read its Y axis pinned at full scale, even on Waveshare's factory firmware. Later it came unstuck.*

### What's inside

Only the output of a MEMS accelerometer is digital. Inside the chip is a tiny mechanical machine etched from silicon:

1. **A proof mass on springs.** A block of silicon, a fraction of a millimetre across, hangs on very thin silicon springs. When the board accelerates, the mass lags behind and shifts slightly, by nanometres.
2. **Comb-shaped capacitor fingers.** Interleaved fingers on the mass and on the fixed frame form capacitors. When the mass shifts, the gaps change, and so does the capacitance.
3. **The digital part only comes last.** An analog circuit measures the capacitance change, an ADC converts it to a number, and that number is what appears in the registers read over I2C.

### Stiction

Stiction (static friction) happens because, at that scale, surface forces win over spring forces. Van der Waals attraction, a film of moisture, or a little static charge can all hold two silicon surfaces together, and the springs are far too weak to pull them apart. A sharp knock, like a drop, a hard tap or shipping, drives the mass against one of its end stops, and it simply stays there.

### How it matched what we saw

- **Pinned at the maximum on every range.** The stuck mass is displaced further than any range can measure, so the output always reads full scale: +4.00 g at ±4 g, +8.00 g at ±8 g, +16.00 g at ±16 g. Real acceleration would have come back into bounds on a wider range. A board sitting still only feels 1 g.
- **The other axes were wrong too.** Multi-axis designs often share masses or have cross-coupled structures, so a stuck mass can drag down neighbouring axes.
- **The data was still live.** The readings had normal noise from sample to sample, and the gyro in the same I2C transfer tracked rotation correctly. So it wasn't frozen registers, a FIFO buffer or a bus problem.
- **The gyro was fine.** A MEMS gyro has its own structure, which is constantly driven to vibrate thousands of times a second, and that constant motion makes it much harder to stick.
- **It came free.** Some vibration or shock overcame the surface force: tapping, or handling while moving the board around.

### Takeaways

- A sensor value pinned exactly at full scale that *follows the range setting* is a strong sign of a stuck mechanical element, not a software bug.
- Code on a wearable should sanity-check the accelerometer: at rest, the total of the three axes should be about 1 g. If it isn't, fall back to the gyro.
- The chip has a **self-test** feature that pushes the proof mass electrostatically. It could be used to try to free a stuck mass without anyone tapping the board.

## Frame budget on a microcontroller display

*2026-11-11: first measurements of the render loop, and the plan to keep a realistic render at 27+ FPS.*

### Where the time goes

The baseline loop drew almost nothing (text and two dots), yet ran at only about 25-27 FPS:

| Step | Time |
|---|---|
| Clear the 360×360 framebuffer in PSRAM and draw | 12.7 ms |
| Send the frame to the display ("flush") | 27.5 ms |
| **Total per frame** | **40.2 ms (~25 FPS)** |

The theoretical limit for sending a frame: 360 × 360 pixels × 16 bits = about 2 Mbit, over a 4-lane QSPI bus at 80 MHz (320 Mbit/s), which takes about **6.5 ms**. The library's flush is about 4× slower than the wire, so most of the frame time is overhead, not unavoidable work.

### Techniques for keeping a realistic render fast

- **Use both cores.** The ESP32-S3 has two CPU cores: one renders the next frame while the other sends the current one to the display.
- **Only redraw what changes.** The black sky needs no work each frame; only the water and the region around the waterline get redrawn.
- **Strips in fast memory.** Render in strips in the chip's fast internal SRAM instead of the slower external PSRAM, and pass them straight to DMA (hardware transfers to the display with no CPU involvement).
- **Cheap maths.** Use fixed-point arithmetic and lookup tables where floating point isn't needed.
- **Measure every step.** Keep frame timing on screen, so each change shows what it costs.

### Why the library's flush was slow

The library sent the frame in 1,024-pixel chunks (127 per frame). For each chunk the CPU read the pixels from PSRAM, byte-swapped them into a small buffer, started the transfer, and **waited for it to finish** before preparing the next chunk. The CPU and the bus took turns instead of working in parallel.

### After the DMA strip pipeline

The fix: render in strips of 20 rows straight into two 14 KB buffers in fast internal RAM, already in the panel's byte order. While DMA sends one strip, the CPU renders the next. An animated water surface (a waterline per column plus a colour per depth) rendered this way:

| Wave screen | Time |
|---|---|
| Prepare the frame (FPS label + surface curve) | 3.2 ms |
| Render + send, overlapped | 9.0 ms |
| ...of which CPU rendering pixels | 7.9 ms |
| **Total per frame** | **12.2 ms (82 FPS)** |

The send is now nearly free: the wire needs about 6.5 ms per frame, and it runs in parallel with rendering. Frame time is now set by how fast the CPU fills pixels, not by the display. At a 27 FPS target (37 ms per frame), that leaves about 25 ms for realism.

| Screen | Before | After |
|---|---|---|
| Diagnostics (text in a PSRAM framebuffer, copied into strips) | 40.2 ms (25 FPS) | 24.2 ms (41 FPS) |
| Wave (rendered straight into strips) | n/a | 12.2 ms (82 FPS) |

### Gotcha: a second SPI device needs a shared bus

Adding our own DMA device to the bus the display library already used made the board hang on the first transfer. By default the library calls `spi_device_acquire_bus()` and holds the bus forever, so any other device waits indefinitely. Its constructor has an `is_shared_interface` flag; with it set, the library only holds the bus while it's drawing.

### Gotcha: a USB serial print can stall the whole loop

With native USB serial (the ESP32-S3's USB-Serial/JTAG), `Serial.print` blocks when the USB buffer is full and nothing on the computer is reading the port. A once-per-500 ms log line was stalling the loop to about two iterations per second, which showed up as a laggy button. The fix is `Serial.setTxTimeoutMs(0)`: drop the output instead of waiting.

The same stall also made a bug in the FPS counter visible. The first frame measured a meaningless tiny interval (FPS in the tens of thousands), and with the loop barely running, the smoothed value took ages to decay, looking "settled" at numbers like 522 FPS.

### Gotcha: polling buttons once per frame

Reading a button once per frame (about every 40 ms) can miss quick taps entirely. Interrupt-driven presses fix that, but contact bounce happens on *release* too, and a release bounce can look like a second press. The fix: count a press only on a falling edge after the line has been quiet for 50 ms.

### Gotcha: `sinf()` gets slower the longer the board runs

The animated test surface ran at 82 FPS right after an upload, but had dropped to 50 FPS a few minutes later, with no code change in between. The rendering and sending times hadn't moved; the per-frame preparation step had grown from 3.2 ms to 10.9 ms.

The cause: the surface curve called `sinf()` 720 times per frame with arguments like `0.03 * x + 1.7 * t`, where `t` is seconds since boot. As `t` grows into the hundreds, `sinf()` has to do expensive *range reduction* to map a large angle back into 0..2π, so the same call gets slower the longer the board runs.

The fix is to wrap the time-based phase once per frame, `phase = fmodf(1.7f * t, 2π)`, so `sinf()` only ever sees small arguments. After the fix, the timing held steady at 82 FPS and 3.2 ms after four minutes.

Takeaways:
- Keep arguments to trig functions small: wrap phases, or use a lookup table.
- Time-dependent slowdowns are easy to miss if you only measure right after flashing. Check timing again after the board has been running for a while.

## Design decision: what the water does when the necklace isn't upright

*2026-11-11: settling how orientation drives the simulation, before writing it.*

### Two views of the same bowl

"Flat water" isn't a special case in wave simulation. What changes is the viewing angle relative to gravity:

- **Upright, as worn:** you look at the bowl from the side. The water fills the lower part of the circle, its surface is a line across the screen, and waves are that line moving up and down. That's a **1D height field**: one surface height per screen column, updated with a wave or shallow-water equation, with sloshing driven by sideways acceleration.
- **Flat, face up:** you look down into the bowl. The water covers the whole circle, and waves become ripples spreading across the face and bouncing off the round wall. That's the classic **2D ripple tank** (often called "Hugo Elias water"): a grid of heights, each cell pulled toward the average of its neighbours, with damping. The rendering uses the surface slope for shading and highlights.
- **In between:** a tilted bowl seen at an angle. Simulating that properly is expensive, so the plan is to crossfade between the two models.

### Decisions

| Question | Decision |
|---|---|
| Flat, face up | Switch to the top-down ripple pool |
| What makes ripples when flat | Bumps to the table (accelerometer spikes), touching the screen (a ripple where the finger lands), occasional random drops |
| Transition | Short crossfade, with hysteresis so it doesn't flicker around the threshold |
| Flat, face down | Blank the screen to save battery; wake when picked up |
| Still and flat for a while | Low-power rest mode after about 30 seconds of stillness |

### Measuring "lean", and why the first guess was wrong

"Lean" is the screen's tilt from vertical: 0° hanging straight, +90° flat face up, -90° flat face down. It comes from the accelerometer: the angle whose sine is the share of gravity pointing through the screen. Face down reads +1 g on the sensor's Z axis, so face up is -1 g.

The first proposal assumed a worn necklace hangs nearly vertical: side view up to 50°, crossfade to 70°. Measuring it on the body showed otherwise:

| Position | Lean |
|---|---|
| Standing upright | +34° |
| Sitting bolt upright | +26° |
| Leaning back in a chair | +50° |
| Leaning back as far as the chair allows | +66° |
| Flat on a table, face up | +90° |

The pendant leans back even when you're upright, because it rests on the chest and the chest slopes: the sternum angles outward from the neckline, and the chain holds the top of the pendant in at the neck. With the first thresholds, simply leaning back in a chair would have started switching to the ripple pool.

### Final thresholds

| Lean | Shown |
|---|---|
| Up to 65° | Side view (sloshing): all normal wearing, including leaning back |
| 65-80° | Crossfade |
| Above 80° | Ripple pool: lying flat on a table, or on the wearer lying on their back |
| Back below 72° | Return to the side view (hysteresis) |

The tilt is averaged over about half a second, so swings and jolts don't count as "lying flat". Sloshing still uses the raw readings, so splashes stay instant.

Because the side view is usually seen at a lean of 25-50°, a viewer is looking slightly down onto the water. A later refinement: show a sliver of the water's top surface behind the waterline, widening as the lean increases.

**Lesson:** measure on the body. A reasonable guess about how a wearable hangs was off by about 30°.

## The sloshing simulation: speed and stability

*2026-11-11: the first version of the side-view water (`ocean_wave/SideView.cpp`).*

### Making the pixel loop fast

The first version rendered at 36 FPS, spending 24.5 ms of CPU per frame on pixels (about 42 CPU cycles per pixel at 240 MHz).

| Change | Render CPU per frame | FPS |
|---|---|---|
| First version (`-Os`, the Arduino default) | 24.5 ms | 36 |
| Compile this one file with `-O3` (`#pragma GCC optimize("O3")`) | 23.1 ms | 38 |
| Precompute the surface for every 1 px step once per frame, and render only the visible circle | 9.4 ms | 81 |

The big win was moving work out of the per-pixel loop. Instead of interpolating between physics cells for every pixel (with clamps, a multiply and branches), the surface height is computed once per frame for each of the ~360 pixel steps across the bowl. The pixel loop then just looks it up. And the round display can't show the corners of its 360×360 square, about 21% of the pixels, so each row only renders the span inside the circle.

Lesson: on a microcontroller, compiler flags give a little; restructuring the inner loop gives a lot.

### A numerical instability

One build showed tiny triangular waves on the surface that rapidly grew into steep spikes, turning the water into noise. That's a classic **numerical instability**: the shortest wave the grid can represent, with neighbouring cells alternating up and down (the "zig-zag" or "2Δx" mode), gains energy each step instead of losing it, until it saturates.

The physics code was the same in builds that worked fine, so the scheme was likely sitting close enough to its limit for that shortest wave that small differences (floating-point rounding, frame timing) decided whether it grew or died out.

The fix is standard insurance: a little **numerical diffusion**. Each substep nudges every cell slightly toward the average of its neighbours. That strongly damps the zig-zag mode while barely touching the large, visible sloshing waves, and it made no visible difference to the water. Real-world grid simulations nearly always include some smoothing or viscosity for this reason.

Debugging lesson: when a simulation goes wrong, reset it before changing anything. If the problem comes straight back, it's in the code; if it doesn't, the state blew up from a particular sequence of inputs.

### Why still water showed bands: only 32 levels of blue

The display uses RGB565: each pixel is 16 bits, with 5 bits of red, 6 of green and 5 of blue. So blue has only 32 levels. The water's depth gradient takes blue from about 240 down to 70 (8-bit values) over 190 px, which in 5-bit terms is only about 21 distinct steps: a new band roughly every 9 px. Green's 64 levels step at different depths, which makes the band edges even easier to see. In still water the bands become sharp, stationary lines following the waterline, and the eye is very good at spotting straight edges in a smooth gradient.

The fix is **ordered dithering**: two colour tables, offset by a quarter of a step either way, alternated in a checkerboard. Each pixel is still one of the 32 levels, but from a normal viewing distance the eye averages neighbouring pixels into in-between shades. After the fix, only a very faint band remained, visible only by staring. A 2×2 Bayer pattern (four offsets instead of two) would give finer in-between shades if ever needed.

The same release added **anti-aliasing** at the waterline: the sky pixel straddling the surface gets a partial foam brightness, according to how much of it is water. That smooths the jagged edge when the surface is tilted, and makes tiny surface movements visible as a shimmer instead of vanishing.

Together, dithering and anti-aliasing cost about 5 ms per frame (81 → 57 FPS).

### Why a fist on the table didn't make a splash

Two problems hid the shocks from the simulation:

1. **Sampling.** The sensor produces 250 samples a second, but the render loop read it only once per frame (~57 times a second), taking whatever the latest sample was. A bang's shock lasts a few tens of milliseconds, so most of it fell between reads. Fix: a FreeRTOS task on the otherwise idle second core reads the sensor at its full 250 Hz and records the biggest jolt since the render loop last asked. Moving the I2C reads off the render loop also raised the frame rate slightly.
2. **The sensor's own low-pass filter.** Even with every sample read, a fist bang measured only 0.01 g. The QMI8658's accelerometer filter had been switched on during setup to smooth the readings, and its narrowest mode cuts off at about 2.7% of the data rate: **about 7 Hz** at 250 Hz. A bang is a sharp shock almost entirely above 7 Hz, so the filter flattened it to nearly nothing. With the filter off (smoothing for tilt is done in software anyway), the same bang measured over 2 g.

"Jolt" is how far the acceleration jumps away from its average over the last ~0.1 s, across all three axes, so a bang counts whichever way it shakes the board. Measured on the desk:

| Event | Peak jolt |
|---|---|
| At rest | ~0 g |
| Typing nearby | 0.07 g |
| Three-finger tap next to the display | 0.2 g |
| Fist on the table | > 2 g |

The splash threshold went to 0.1 g: typing is ignored, taps and anything stronger splash, with a square-root curve so a tap gives a small splash and a bang a full one. A splash kicks two of the bowl's natural sloshing shapes (standing waves that fit between the walls) with random strength and direction, which looks like water being thrown about rather than noise.

Lesson: a filter chosen for one purpose (smooth tilt) can silently destroy the signal another purpose needs (shocks). Filter as late as possible, in software, where each consumer can choose its own.

## Making a wave curl

*2026-11-11: the question was whether a wave could rise up the round wall so far that it curls back over itself, and whether that needs a different algorithm.*

### Why the sloshing model can't curl

The sloshing simulation is a **height field**: exactly one surface height per column across the bowl. A curling wave is an overhang, where a single column has water, then air, then water again above it. A height field cannot represent that, by design. That restriction is also what makes it fast and stable.

### The options, from cheapest to most real

1. **Height field plus spray particles** (the classic game technique). Keep the sloshing model. When a crest gets steep or water races up the wall, emit particles that fly ballistically under gravity and fall back in, leaving foam where they land. It reads as breaking, spraying water, and costs little next to the per-pixel rendering.
2. **A curl that follows the wall.** The round bowl helps here: above its middle, the wall curves back *inward*, so water racing up the wall gets steered back over the top of itself. That's the real reason waves curl in a round container.
3. **A true 2D fluid simulation:** particle methods (SPH, position-based fluids) or a grid tracking a free surface (FLIP, volume of fluid). Real overhangs, sheets and droplets with no tricks, but a different algorithm altogether, much more expensive, and less stable. On this chip, perhaps 1,000-2,000 particles at 30 FPS, rendered at low resolution and scaled up. That could suit a chunky 8-bit quality mode, or the lava idea, where a particle fluid's gloopy behaviour would show.

### The plan built: options 1 and 2 together

The curl comes out of the same particle system as the spray:

- **Particles:** a pool of up to 320, in the water's frame, flying ballistically under gravity. Hitting the bowl wall, they slide along it with most of their bounce absorbed. Falling back into the water, they disappear and leave a fleck of foam.
- **The wall curl:** each frame, find where the surface meets the wall on each side. If the water there is rising fast and is above the bowl's middle, where the wall curves inward, launch particles along the wall's tangent. They shoot up the curved wall already heading inward, and gravity bends them over into an arc that lands back on the water away from the wall. A continuous stream of them forms the curling lip, from the physics of the round wall rather than a faked shape.
- **Crest spray:** crests rising fast throw off droplets.
- **Splashes:** a jolt adds a burst of droplets on top of the sloshing kick.
- **Rendering:** small discs of 1-3 px, light blue for the curling sheet (whitening as it ages) and white for droplets, drawn over each strip.

### Making the curl look like it belongs to the water

The first curl worked physically but looked "disjoint" from the water. Two visual fixes, both the user's diagnosis:

- **The froth line was too light where the curl left it.** The white froth ran right up to the wall, and the curl sheet coming off it was a darker blue, so the eye saw two separate things. Fix: within 50 px of the wall (first tried 28 px, then widened), the froth shades from its blue-white into the curl-sheet blue. Now the curl grows out of the surface. Cost: nothing measurable, because the shade is worked out once per column per frame, not per pixel.
- **The water rose as a thin stream.** Real water running up a wall bunches into a thick tongue, then stretches thin as it climbs. Fix: curl particles start spread over 12 px in from the wall as fat blobs (6 px radius) that shrink to a 2 px sheet over about half a second. The ones against the wall launch fastest; the inner ones launch up to 35% slower, fall back first, and draw the tip out.

## Slow and stately: tuning the feel

Halving the wave speed (330 to 165 px/s, a wave crossing the bowl in 2.2 s instead of 1.1 s) made the water feel "heavier and more stately", which suits a 1.85-inch display better. The 330 felt too fast for its width.

A side effect worth knowing: **halving the wave speed doubled the splash heights.** A splash adds a burst of flow, and a wave's height is roughly flow × depth ÷ wave speed. A steady tilt was unaffected, because the sideways push is scaled by the same gravity constant that sets the wave speed.

| Setting | Before | After | Why |
|---|---|---|---|
| Wave speed | 330 px/s | 165 px/s | heavier, slower water |
| Splash gain | 220 | 90 | bring splash heights back down, then a little lower |
| Max splash | 260 | 110 | cap on the biggest bang |
| Drop strength | 35 | 30 | still water should still show the drops clearly |

The user's verdict, with the board swaying gently on a neck: "slow and stately and draws attention without being shock and awe".

## Design decision: the water rises as you lean back (replaces the crossfade)

The orientation plan had a crossfade between the side view and a top-down pool from 65° to 80° of lean. The user suggested something better: the further the necklace leans back, the higher the water rises in the bowl, until it turns into the top-down view.

It's physically right. Look at a glass of water from the side, then gradually from above: you see more and more of the water's surface until, from directly above, it fills the circle. The top-down pool is the end of that continuous motion, so no blend is needed.

| Lean | Water level | Situation |
|---|---|---|
| up to 40° | normal | worn standing or sitting (+26 to +34) |
| 40° to 80° | rises smoothly (smoothstep) until 30 px of sky is left | leaning back in a chair (+50), big lean (+66), being set down |
| above 80° | full circle: the top-down pool (to be built) | flat on a table |

Starting at 40° keeps normal wear unchanged. A bonus the user noticed: tilting back turns more of the display a deep blue, which looks good in its own right.

## Face down: the screen goes dark

Smoothed lean (averaged over 0.5 s) below -70° turns off the backlight and stops drawing; above -50° it wakes and the water carries on where it left off. The 20° gap stops it flickering at the threshold. The user's note: it could wake a little sooner, to tune later.

## The top-down pool

When the necklace lies flat (smoothed lean above 80°), the side view hands over to a view from above. The whole circle is water and ripples spread from taps, bumps and random drops.

### How it works

- **Physics:** the 2D wave equation on a 91 × 91 grid of 4 px cells, stepped 120 times a second. Ripples travel at 160 px/s, about the side view's wave speed, reflect off the round rim and fade (damping 0.8/s).
- **The look:** seen from above, water shows its shape through light, not height. Each cell's shade comes from its slope towards the upper left: lit slopes brighter, shadowed slopes darker, the steepest lit slopes glinting white. The shade is interpolated between cells for every pixel and dithered like the side view.
- **Speed:** 45 FPS (side view 56-60). Comfortably above the 27 FPS goal.

### Why you can't "fill it up" with taps

The pool doesn't keep a list of ripples. It's one surface of 91 × 91 heights, and a tap just pushes down a small patch of it. The wave equation then moves the whole surface, so ripples pass through each other and add up where they meet, like real water. A hundred ripples cost exactly the same as one. They fade fast for two reasons: each ring spreads its energy over a growing circle, and damping removes about half of what's left every 0.9 s.

### Making the hand-over seamless

| Snap the user saw | Cause | Fix |
|---|---|---|
| The last strip of sky vanished as the pool appeared | The side view stopped rising with 30 px of sky left | The side view now fills the bowl completely by 80° |
| Even blue pool after the side view's light-to-navy gradient | The pool had its own flat colour | The pool's resting colour is the full side view's depth gradient, oriented by whichever way the flat necklace tips (the user's idea) |
| Ripples everywhere the moment the pool appeared | Old ripples frozen from the last visit, plus the bump of setting it down (a hard one sends a ring in from the whole rim) | Calm water on every entry, and bumps ignored for 1.5 s while it's set down |
| One tap made many ripples when hand-held, but not on a table | Tapping a hand-held board shakes it: the pool read the jolt as a bump and rang the whole rim | Bumps ignored while touching and for 0.3 s after; each jolt is held back one frame (17 ms) so a jolt arriving just before the touch registers is caught too |

The user's observation that cracked the last one: the many ripples only happened when the board was held in mid-air "as if being worn", never on a flat surface.

### Touch: one ripple per tap

At first a dragged finger left a wake of ripples. The user preferred a single ripple per tap: it "invites more tapping to see how they interact".

### Brightness: touch and hold

Tapping to cycle brightness clashed with tapping for ripples. The user's idea: touch and hold to adjust, so a quick tap stays free.

| Holding for | Brightness |
|---|---|
| under 0.6 s | no change (a tap) |
| 0.6 to 1.4 s | dimmest: release here to keep it |
| 1.4 to 3.9 s | ramps up to full |
| longer | stays at full |

The pause at dimmest was the user's addition, because dimmest is how it will most likely be worn in dim light. The ramp is exponential (each step multiplies the brightness), because that's how the eye judges brightness. A linear ramp would look "already bright" after the first moment.

### No random drops in the pool

The pool first had a random drop every couple of seconds, copied from the side view's "still water never looks frozen" idea. The user removed them: a flat necklace on a table is a still bowl, and taps plus sharp jabs (which send a ring in from the rim) are entertaining enough.

### The "random" drops were the user's hand

Even with random drops removed, drops kept appearing as the user turned the flat board. Turning it by hand jiggles it enough to register small bumps (over 0.1 g), and the pool answered each one by scattering ripples at random spots. Now only a sharp jab (over 0.6 g) disturbs the pool, with a ring from the rim.

### The light area jumped across when tipped the other way

On a flat necklace, "down" comes from a tiny, heavily smoothed tilt. Tipping it the other way passes through "exactly flat", where the direction is lost: the light area froze, then jumped to the opposite edge once the new tilt was readable. Fix: the light area now swings round towards the new direction at up to about 90° a second, so it travels round the rim instead of jumping.

### Coming out of flat: the side view started somewhere else

Lifting the necklace out of the pool, the side view's sky sometimes appeared in a completely different place from the pool's light area. The side view had kept its "down" from before the pool took over, so it started where it left off, not where the light area had drifted to. Fix: the hand-over now goes both ways. Entering the pool, it starts from the side view's direction; leaving, the side view starts from the pool's. The switch itself also no longer counts as a sideways push, so lifting it doesn't trigger a slosh.

## Rest mode

Lying flat with nothing happening for 30 s, the necklace rests: the backlight fades out over 3 s and drawing stops. It checks the sensor and touch about 20 times a second. A touch, a bump over 0.08 g (just above typing nearby, 0.07 g) or picking it up wakes it with a 0.3 s fade-in, back to whatever brightness was chosen, and the pool carries on where it was.

The backlight is the big power saving. Whether to go further (lower CPU clock, light sleep with the IMU's wake-on-motion interrupt) waits until the current draw can be measured on battery.

A small design point: every backlight change (the hold-to-dim ramp, face down, the rest fade, the wake fade) now goes through one function, so they can't fight over the brightness.

## Making the curl look like water, frame by frame

The user filmed at 120 FPS (about two video frames per screen frame) and stepped through frame by frame. Each round of photos pointed at the next fix.

| What the photos showed | Cause | Fix |
|---|---|---|
| The tongue looked like blobs sitting on the water | It was drawn in a pale sky-blue, lighter than the water, so the eye saw a separate object. Its overlapping round discs made a "string of beads" outline | The tongue starts in exactly the colour of the water just under the surface, so its blobs merge with the water body and each other. It pales to sheet blue as it climbs and thins. The froth line at the wall fades into the same water colour |
| Spray seemed to land *behind* the water line | Colour followed age only: a young piece falling back was still water-coloured and vanished into the water before reaching the line | Glassy on the way up, white once falling: real water breaks up into spray as it falls |
| Little tongues that turned white but never left the water | Halving the wave speed halved how fast water climbs the wall, so launch speed halved and flight height (v²/2g) dropped to a quarter | Launch gain doubled (1.4 → 2.8), and no launch below 220 px/s (about 24 px of flight): slow climbs just run up the wall smoothly |
| Tongues a bit sparse | The emit rate also follows the climb rate, so it had halved too | Emit rate doubled (0.02 → 0.04) |
| White pieces hovered above the line, then vanished | They landed on a rising crest; when the wave turned, the surface dropped away faster than they could fall. On landing they simply disappeared, leaving an under-1-px foam bump | Landing spray becomes a **fleck riding the surface**: it moves up and down with the wave and drifts with the flow, shrinks, and fades after 0.8 s |

The user's verdict: "the best yet. Even the curls that barely leave the water at the edges land back right under where the curl was. Those that fly off the edge of the screen leave a pattern of foam on the wave."

A lesson from this: front-to-back jolts (towards and away from the viewer) slosh differently from sideways ones, and the user had only been testing sideways. Test motions in every direction.

## How the tuning variables relate (why one change affects everything)

The user changed one setting, the wave speed, to make the water feel thicker, and it changed *everything*: splash heights, curl height, curl density, spray, foam. This section maps why.

### The hub: wave speed

The water model has a "gravity" G, set from the wave speed: G = WAVE_SPEED² / DEPTH. Inputs reach the water in two different ways, and that's the key:

- **As a force (tilting, swinging, swell):** the sideways push is measured in g and multiplied by G. Tilt settles where the push balances the surface slope, and G cancels out. **Tilt doesn't depend on wave speed.**
- **As a burst of flow (splashes, drops):** these add a speed in px/s directly. A wave's height is roughly flow × depth ÷ wave speed. **Halve the wave speed, and every splash is twice as tall.**

Everything about the curl is driven by how fast the water climbs the wall (the rise rate), and for a wave of a given height that scales with wave speed. So slower waves mean:

- **Lower curls:** launch speed = rise × CURL_SPEED_GAIN, and flight height = speed² / (2 × particle gravity). Half the rise rate gives a quarter of the height.
- **Thinner curls:** pieces emitted = rise × CURL_EMIT_RATE.
- **Less spray and foam:** crest spray needs a rise above SPRAY_MIN_RISE, and foam grows with flow speed.
- **Ambient drops start sooner:** "still" means every flow speed is under STILL_FLOW.

### What each setting changes

| Setting | Directly controls | Also changes | Doesn't change |
|---|---|---|---|
| WAVE_SPEED | how fast waves cross the bowl (165 px/s: 2.2 s) | splash height (inverse), curl height (squared) and density, spray, foam, when drops start | how far the surface tilts |
| DAMPING | how fast sloshing dies, in seconds | at a slower wave speed, fewer crossings before it settles: feels heavier | everything else |
| SMOOTHING | wipes out the smallest ripples (keeps the grid stable) | high values make the water look thick | big sloshing |
| SPLASH_GAIN, MAX_SPLASH | wave height from a jolt | curls and spray after a jolt (bigger waves climb faster) | tilting |
| DROP_STRENGTH | ripples from ambient drops in still water | | |
| SENSITIVITY | boost for small quick motions (in g) | | independent of wave speed |
| CURL_SPEED_GAIN | curl launch speed | curl height (squared) | |
| CURL_MIN_SPEED | which climbs throw a tongue at all | | |
| CURL_EMIT_RATE | tongue density | particle budget use | |
| PARTICLE_GRAVITY | how fast pieces fall | curl height for a given launch, the loop-the-loop limit (CURL_MAX_SPEED ≈ √(gravity × radius)) | the water itself |
| PARTICLES_PER_SECOND, PARTICLE_BURST | safety cap on pieces | can cap tongue density in violent shaking | |
| TURBULENT_DAMPING, CALM_RMS, CHURN_RMS | extra calming when the water churns | | gentle motion |
| REST_LEVEL, FULL_LEVEL, LEAN_RISE_* | water level and its rise with lean | | physics |
| SUBSTEP | physics step size | must keep WAVE_SPEED × SUBSTEP / 4 px well below 1 (now 0.17) | |

### Rules of thumb

- **Change the wave speed by a factor k, and to keep the same look:** splash and drop strength × k, curl gain ÷ k, curl emit rate ÷ k. That's what we ended up doing by hand: speed halved; splash 220 → 90, which went a bit further than half; curl gain 1.4 → 2.8; emit rate 0.02 → 0.04.
- **To make it feel thicker without changing anything else:** raise DAMPING (settles faster) or SMOOTHING (small ripples vanish).
- **The pool** (top-down view) has its own ripple speed and damping and is independent, except that it copies FULL_LEVEL and the depth colours from the side view to keep the hand-over seamless. Change those together.

## Getting ready for lava: the Material struct

Before adding lava, everything that makes the fluid *water* (wave speed, damping, splashes, the curl, foam, colours, pool ripples) moved into one `Material` struct, with `WATER` holding the tuned values. The bowl's own settings (lean levels, grid sizes, step sizes, safety limits) stayed where they were. The test for this step was simply that nothing looked different, and the user confirmed it looked identical with jolts, taps and the pool.

Why this design:

- **Lava can't break water:** they're two separate sets of numbers, so tuning one never touches the other.
- **All floats on purpose:** two materials can be blended field by field (`Material::mix`), so switching can be a gradual thickening, water turning into lava, rather than a cut.
- **Colours are part of the material:** the colour tables are rebuilt from them, fast enough to do every frame during a blend.

A side discovery: the program shrank by about 100 KB. The view objects had been stored in flash as a full starting image, including about 108 KB of zeros for the pool's grids, and copied into RAM at boot. Now they start zeroed and are filled in at startup. RAM use is unchanged.

## Lava, and the swipe between

Built in three contained steps, each tested before the next:

1. **The Material struct** (no visible change): water's tuning moved into one set of numbers.
2. **The lava material:** a second set of numbers, chosen with the rules of thumb from the relationships section.

   | | Water | Lava | Why |
   |---|---|---|---|
   | Wave speed | 165 | 70 | thick and slow |
   | Damping | 0.9 | 2.5 | settles quickly |
   | Smoothing | 0.06 | 0.2 | small ripples vanish |
   | Splash gain / max | 90 / 110 | 40 / 50 | scaled with wave speed |
   | Curl | thin tongue into spray | a few fat, slow, sticky blobs | lava doesn't spray |
   | Colours | navy to light blue, white froth | dark red to orange-yellow, glowing yellow edge | |

   The user's verdict: "Everything looks amazing. It's thick and gloopy."
3. **Swipe to switch:** a finger travelling more than 80 px sideways within 0.6 s toggles water and lava. Hold-to-dim only counts if the finger stays still, so the gestures never clash.

**The user's idea: gas bubbles.** Unlike water, where a drop falls in, still lava now and then lets out a gas bubble. A small blob of lava barely jumps off the surface and makes a very small, fast-fading wave. In the pool it's a tiny ring. It "adds some life to a still view".

**The fade between them.** Because every material setting is a float, the switch blends all of them over 1.5 s: physics and colours together. The fluid visibly thickens into lava (waves slowing and being absorbed) or thins back into water. The user's reaction: "OMG! You added a fade between the two! Exquisite!"

A safety improvement along the way: the materials were first written as plain ordered lists of numbers, and two lava values went in the wrong order (caught before upload). They now use named fields (`.waveSpeed = 70`), and the compiler rejects any field out of order.

## Remembering settings across power-off

Brightness and the water/lava choice are saved in the ESP32's key-value store in flash (NVS, via the Preferences library). Flash wears out with writes (roughly 100,000 erases per block, spread over several blocks by NVS), so settings are saved only once they're settled: brightness when the finger lifts after adjusting, the material when a swipe happens. That's a handful of writes a day, so it would take centuries to wear out. Saving every frame of the brightness ramp would have been about 60 writes a second, which could have worn it out in weeks.

At boot, a saved lava choice goes straight to lava with no fade.

## Mercury: the third material

Choosing a third material, the user found water and lava "sort of the extremes". Of three ideas (honey, mercury, a bioluminescent night sea), mercury was picked because it "lands right in the middle of water and lava".

| | Water | Mercury | Lava |
|---|---|---|---|
| Wave speed | 165 | 200: heavy but lively | 70 |
| Damping | 0.9 | 1.6: settles quickly | 2.5 |
| Smoothing | 0.06 | 0.1: surface tension | 0.2 |
| Curl | thin tongue into spray | a few round beads that stay round | fat, sticky blobs |
| On the wall | slides | slides easily and bounces back (doesn't wet the glass) | sticks |
| Falling | normal (1000) | fast (1300): heavy | slow (700) |
| Surface line | white froth | a thin bright highlight | glowing edge |
| Colours | blues | gunmetal to bright silver | dark red to orange-yellow |

It took only a new set of numbers; no code changes for the fluid itself. The user's verdict: "Mercury has distinct beads and less of them in the curl. And the pool ripples sit closer to water than lava, but feel distinct."

The swipe now steps right through water → mercury → lava (left goes back, wrapping round), blending from whatever is on screen, even mid-fade. Swapping between any two of them is a smooth transformation.

## Future ideas

### More materials

Water and lava are the extremes, and mercury sits between them. Each new material is mostly a new set of numbers in `Material.cpp`, plus maybe one small effect of its own.

- **Honey:** between water and lava. Amber and golden, very slow but less damped than lava, so it sags and folds instead of stopping dead. No spray, maybe long stretchy strands on the curl.
- **Night sea:** water, but dark, with bioluminescence. Flecks and froth glow cyan where the water is disturbed and fade slowly in the dark. Mostly colours plus a glow fade.

### A desk toy, not just a pendant

Pick it up to play; set it down to watch. It needs a stand that lets it move between the side view and the pool.

Stand ideas:

- **A pivoting cradle:** the disc sits in a fork, like a desk globe, with detents at about 30° (side view) and 90° (pool). In between, the water rises as it tips. Flicking it on the axle sloshes it.
- **A rocking base (the user's favourite):** a two-position rocker. A flat face at the slant angle holds it as a stand; push it, and it rolls over a curved section and rocks a few times before settling flat in pool mode.
  - **Rest positions:** the slant face is stable while the centre of mass sits inside it. Flat on the bottom is the lowest point, so that's where it settles.
  - **How it rocks:** with a circular curve and the centre of mass below the curve's centre, it rocks before stopping. Weight low makes it settle faster, weight high makes it livelier. The battery is the heaviest part, so where it sits tunes the rocking.
  - **Slant angle:** 30-40° shows the normal water level; 50-60° shows a fuller bowl.
  - **Software:** while it rolls, the side view sloshes and the water rises. The averaged lean and the 8° gap between the switch points (into the pool at 80°, back out at 72°) should stop it flickering between the views while it rocks. The pool starts calm and ignores bumps for 1.5 s, so the last wobbles won't splash it. Try the motion by hand first, and film the rocker in slow motion once it's printed.
- **A magnetic dock:** snaps onto an angled block upright, lifts off to lie flat. It could double as the charger.
- **A ball joint:** pose it at any angle, including the in-between lean where the bowl is nearly full.

### Power for the desk toy

A USB cable would drag on the rocker, so the desk toy wants a battery too: unplug it to play, plug it back in. Charging options:

- **A magnetic USB-C connector:** cheapest; the cable snaps off if pushed.
- **Charging contacts on the rocker's slant face:** pogo pins or magnetic pads. It charges whenever it sits as a stand, and pushing it off disconnects it.
- **Wireless (Qi):** a receiver coil in the base, charging whenever it lies flat on a pad.

Software for a desk toy: rest mode currently only starts when it's lying flat (in the pool), because worn it's never still for long. On a stand it could sit upright and untouched for hours, so it would want rest after a few minutes upright and still too, waking on a bump or touch.

### For the battery session

- Measure current draw while awake (full brightness and dimmest) and while resting. Rest currently turns off the backlight but leaves the chip running, so a resting toy still drains the battery.
- If resting draw is high, add deeper sleep, woken by the motion sensor's interrupt on a bump, so it can sit for days between plays.
- Tune the splash threshold and rest settings with real footsteps while worn.

### Case design: drawings and a STEP model

Waveshare publishes the mechanical files on the board's [Resources and Documents](https://docs.waveshare.com/ESP32-S3-Touch-LCD-1.85/Resources-And-Documents) page, which is linked from the [product docs](https://docs.waveshare.com/ESP32-S3-Touch-LCD-1.85).

- **3D model (STEP), DWG and PDF drawing:** [ESP32-S3-Touch-LCD-1.85.rar](https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.85/ESP32-S3-Touch-LCD-1.85.rar), about 3.8 MB. It contains `ESP32-S3-Touch-LCD-1_85-20260703.stp` (19 MB), a `.dwg` and a `.pdf`.
  - The page labels this link "1.85C Structure Drawing", but the files inside are named for the 1.85.
  - The PDF shows this round board with three M2 holes.
  - Open the RAR with The Unarchiver or Keka on a Mac.
- **2D drawing only, an older version:** [ESP32-S3-Touch-LCD-1.85_2D_structure.zip](https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.85/ESP32-S3-Touch-LCD-1.85_2D_structure.zip) (DWG + PDF).
- **Schematics:** [original](https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.85/ESP32-S3-Touch-LCD-1.85.pdf) and [V2.1](https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.85/ESP32-S3-Touch-LCD-1.85_V2.1.pdf).

Key numbers from the PDF drawing in the RAR:

| What | Size |
|---|---|
| Cover glass (outer diameter) | 55.00 ± 0.1 mm |
| Visible display area | 46.08 mm diameter |
| Total thickness, glass to the tallest part on the back | 9.70 mm |
| Glass thickness | 0.70 mm |
| Mounting holes | 3 × M2, on the back of the board |

**Check first:** the two drawings show different board layouts (the USB-C port, SD slot and holes sit in different places). Waveshare has revised this board more than once, and we already know of two panel revisions. Compare the drawing with your own board before modelling the holes and cut-outs.

**Battery:** a 602535 cell is about 6.0 × 25 × 35 mm. Allow a few extra millimetres at one end for the protection board and wires, and measure the actual cell.

## Understanding the code

A reading order for the physics, from simplest to most involved:

1. **`SideView::step()`** in `ocean_wave/SideView.cpp`: the heart of the water, about 20 lines. Each tiny time step, flow speeds up downhill (`q += dt * (-G * slope + aSide)`), then water moves between cells (`h -= dt * depth * dq/du`). Everything else is built on those two lines.
2. **`SideView::update()`:** where the motion sensor becomes sloshing. It smooths gravity, takes the difference from the raw reading as the sideways push, boosts small motions, adds the swell, and triggers splashes.
3. **`SideView::emitWallCurl()` and `updateParticles()`:** the curl. Find where the surface meets the round wall, launch pieces along the wall, let gravity bend them over, slide them along the glass, and turn them into flecks when they land.
4. **`PoolView::step()`** in `ocean_wave/PoolView.cpp`: the 2D ripple equation, one line per cell. A cell's next height comes from its momentum plus how far it sits above or below its four neighbours. That's how ripples spread, reflect and pass through each other.
5. **`SideView::render()`:** the speed trick. Each pixel costs two additions to know where it sits in the tilted water, then one lookup for its colour.
6. **`ocean_wave/Material.cpp`:** the three fluids side by side as plain numbers. It's the easiest file to play with.

The long comments at the top of `SideView.cpp` and `PoolView.cpp` explain the ideas in plain words. So does the "How the tuning variables relate" section above.

## Future board considerations

### Moving to a smaller board

The 1.85" board is fast mainly because of its **4-line QSPI** display bus: frames go out about four times faster than over a single data line. That's the first thing to check on any other board: the display controller and its bus.

| Board | Display | Motion sensor | Notes |
|---|---|---|---|
| ESP32-S3-Touch-LCD-1.28 | 240×240 LCD, single-line SPI (GC9A01) | QMI8658 | battery charger; only 2 MB PSRAM; smallest and cheapest |
| ESP32-S3-Touch-LCD-1.46 | 412×412 LCD | yes | speaker and mic; more pixels than the 1.85" in a smaller circle |
| **ESP32-S3-Touch-AMOLED-1.43** | 466×466 AMOLED | QMI8658 | RTC, battery header, optional metal case (-B); likely QSPI |
| ESP32-S3-Touch-AMOLED-1.32 | 466×466 AMOLED | possibly none | check before considering |

(From search results: confirm on the official pages.)

**What a port involves**, using the 1.28" as the example:

- **Board library (the biggest job):** the fast strip pipeline sends 4-line QSPI transactions. A single-line SPI display with a separate data/command pin needs that part rewritten. The idea stays the same: strips rendered into two buffers while the previous one sends.
- **Pins and quirks:** different pins. The 1.85" puts the screen and touch resets on an expander chip; check how the new board wires them, and its power latch and buttons, against its schematic.
- **Pixel-based tuning:** wave speed, splash strength, curl speeds, particle gravity, blob sizes, foam, water levels and the pool cell size are all in pixels. They scale with the bowl radius (⅔ for 240 px). The clean way is one scale factor applied to the materials and bowl constants.
- **Speed:** 240×240 is 44% of the pixels, so rendering takes about 6-7 ms. Sending a frame over single-line SPI at 80 MHz takes about 11.5 ms, against about 6.5 ms for QSPI now. Overlapped, that should still give 60+ FPS.

### The 1.43" AMOLED: the favourite

- **True black:** AMOLED pixels are off when black, so the black sky costs no power and glows in a dark room.
- **Likely QSPI:** the fast frame path should carry over with few changes.
- **More pixels:** 466×466 is about 1.7× today's pixel count.

| Display | Pixels across | Density |
|---|---|---|
| 1.85" (current) | 360 | about 195 ppi |
| 1.43" at full resolution | 466 | about 326 ppi, the same as the iPhone 4's "Retina" screen |
| 1.43" at half resolution, pixels doubled | 233 | about 163 ppi: blocks about 16% larger than today's pixels |

### The plan: try three ways and compare

1. **Full resolution:** render all 466×466 pixels. Expect about 25 ms of rendering, roughly 35-40 FPS: still over the 27 FPS goal, with less headroom. Measure the real FPS.
2. **Half resolution with smooth scaling:** render 233×233 and blend between pixels while scaling up. Gradients stay perfect, and edges (waterline, beads, flecks) come out soft instead of stepped. Rendering is cheaper than today.
3. **Plain half resolution:** render 233×233 and copy each pixel into a 2×2 block. Cheapest, but with slightly stepped edges and squarer beads.

What to expect: the water body (gradients plus dithering) hides pixels at any of these. Edges are where the differences show. At about 30 cm, when the wearer looks down, the eye can just pick out pixels at 163 ppi. Someone looking from a metre away couldn't.

### A photo locket

Swipe vertically to step through round photos, like a locket. There's plenty of room. Memory as of the three-material version:

| Memory | Used | Free |
|---|---|---|
| Program flash (3 MB app partition) | 427 KB (13%) | about 2.7 MB |
| Internal RAM (320 KB) | 157 KB (47%) | about 170 KB, shared by the stack, the DMA strip buffers and anything added later |
| PSRAM (8 MB) | about 260 KB (diagnostics canvas) | about 7.7 MB |
| Built-in flash storage (9 MB FAT partition) | unused | 9 MB |

How it would work:

- **Showing a photo:** a 360×360 photo is about 260 KB decoded, or 30-60 KB as a JPEG. Decode it once into PSRAM (well under 0.1 s), then send it through the same strip pipeline the diagnostics screen uses. The JPEG decoder plus file libraries add roughly 100-150 KB of program flash.
- **The circle:** square photos are fine as they are. The round screen hides the corners, so the locket shows the centre circle.
- **The gesture:** vertical swipe is free. Today's swipe requires mostly-horizontal movement, so the vertical one is its mirror image.
- **A battery bonus:** a still photo can be drawn once and left on screen without redrawing. Locket mode would use far less power than the water.
- **Storage:** the built-in 9 MB FAT partition, already set up and empty, holds 150-plus JPEG photos, so the SD card may not be needed. The SD slot's control pin goes through the expander chip, so check how Waveshare's demo drives it before relying on it.

Getting photos onto it:

- **Bluetooth (BLE):** about 50-100 KB/s in practice, so about a second per photo. It needs about 50-70 KB of internal RAM, a big chunk of what's free. The real problem is the phone side: iPhones can't send files over BLE from a web page (Safari has no Web Bluetooth), so it would need a custom app or a browser like Bluefy.
- **Wi-Fi (recommended):** the necklace hosts a small upload web page, either as its own hotspot or on the home network.
  - Works from any phone, including iPhones in plain Safari, with no app to install.
  - Around 1 MB/s.
  - The page can crop and shrink each photo to 360×360 on the phone before sending.
  - Wi-Fi needs internal RAM too, but only while uploading: enter an upload mode from a menu or button combination.

**Plan:** a vertical-swipe locket reading JPEGs from the built-in storage, photos added through a Wi-Fi upload page, an SD card later only if 150 photos isn't enough.

**Photos survive code uploads.** A normal sketch upload rewrites only the boot loader, the partition table and the app partition. The FAT partition holding the photos, and the NVS settings (brightness, material), sit in their own areas of flash and aren't touched. Things that *would* wipe them:

- changing the partition scheme (the storage moves or changes size)
- the IDE's "Erase All Flash Before Sketch Upload" option
- `esptool erase-flash`, or restoring the full factory firmware backup (`backup/factory_firmware_16MB.bin` overwrites all 16 MB)

Photos on an SD card are safe from all of these.

## The bowl model: the circle becomes the container

### The insight (the user's, after a steak and shrimp dinner)

"The calculation looks to go off the edge of the screen. So the edge of the screen means nothing. It's still a porthole." The original model was a rectangular tank, 360 px wide with a flat floor and vertical walls at the far left and right; the circle was only drawn over it. Water climbing those invisible walls was cut off by the screen edge, and the curl was layered on separately, so it never quite matched the water it seemed to come from.

### The fix, within the same family of models

No need for a full 2D fluid (particles or grids that track clumps of water). The shallow-water method handles an uneven floor; it's how waves on beaches are modelled.

- **The round floor:** each cell is a column of water standing on the circle (floor = -√(R² - u²)). The view rotates with gravity, so the floor never changes shape.
- **Wet and dry cells:** near the walls the floor curves up and the water gets shallow, or runs out. When the water tilts, the columns past the waterline are dry floor. In the classic model those columns still held water, drawn outside the circle.
- **The full equations:** the non-linear shallow-water equations, solved with a Rusanov flux and "hydrostatic reconstruction" (Audusse et al., 2004). That keeps still water exactly still over the curved floor and depth never negative where the waterline moves.
- **Shoaling for free:** waves travel at √(g·depth), fast in the deep middle and slow in the shallows, so they bunch up and steepen as they run up the wall. That's what makes real waves break.

Checked in a Mac simulation before uploading: still water stayed perfectly still (zero drift), a strong push sent the water up one side with dry cells on the other, three seconds of violent shaking stayed stable, and volume was conserved exactly.

### The curl, rebuilt on real water

Above the equator the wall curves back over the water, which a column model can't hold. That's where the curl starts:

- Water that climbs past the equator is taken out of the body and launched as curl pieces, each carrying 4 px² of water. Landing pieces give it back. The bowl keeps its exact volume, counting water in flight.
- **Launch speed was the hard part.** The first try used how fast the water flows sideways at the wall: nearly zero on the steep part, so pieces barely left (the user saw "one particle"). The simulation showed why: near a steep wall, a small sideways shift lifts the edge a long way. The edge reaches the equator climbing about 70 px/s in a firm swing, 140 in a hard jolt, 350 in violent shaking. That climbing speed, times the material's curl gain (×1.5 for the bowl), is the launch speed.
- **Too little curl at first:** the scheme smooths the water a little every step, on top of the material's damping. Halving the damping for the bowl doubled the water reaching the equator, and smaller pieces (4 px² instead of 10) made denser tongues: about 150 pieces for a firm swing, against about 30 before.

### Foam that floats

- **Foam moves with the water** (it was attached to fixed spots). A swell carries foam up the wall and leaves it there as it drains. It gathers at the water's edge, which produced an unplanned **meniscus** that slowly drains back.
- **A forming curl consumes the foam.** Old foam and landed flecks at the wall drew a white line across the base of each new tongue, so it looked "severed". Now a curl clears the foam and flecks within 40 px of its wall as it forms.

### Lessons

- **Make the mode visible.** Twice the user filmed the classic model thinking it was the bowl, because nothing on screen said which was running and every upload reset it to classic. The model is now saved like the other settings, and the water screen can show "bowl" or "classic" while comparing.
- **Simulate before uploading.** Quick simulations on the Mac caught a launch speed that would have been near zero, and quantified the damping trade-off, before any flashing.

### The verdict

"There is a definite difference between bowl and classic. The water seems to move better. It might not curl as much, or as high, even during violent motions, but it seems to feel better. With classic I could throw water from one end of the circle to the other." Part of classic's drama was the porthole: water built height on invisible walls outside the circle. Lava stays thick with barely any curl; mercury throws fewer, smaller marbles, since only water that really overshoots the wall leaves. Bowl is now the default; classic stays switchable on the diagnostics screen (short PWR).
