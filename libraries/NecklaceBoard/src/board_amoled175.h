// Board constants: Waveshare ESP32-S3-Touch-AMOLED-1.75 (round 466x466 AMOLED, CO5300).
// The ESP32 core has no entry for this board; select "Waveshare ESP32-S3-Touch-AMOLED-1.43"
// (same chip, flash and PSRAM), whose define picks this file.
//
// Not written yet: needs the pins from Waveshare's schematic / demo code, the
// CO5300 init sequence, the touch chip (CST9217 or FT3168) and the AXP2101
// power chip. See things-I-learned.md ("The 1.75" AMOLED").
#pragma once

#error "NecklaceBoard: support for the ESP32-S3-Touch-AMOLED-1.75 isn't written yet"
