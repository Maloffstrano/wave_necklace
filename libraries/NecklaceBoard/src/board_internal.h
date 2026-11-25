// Shared pieces (NecklaceBoard.cpp) that each board's begin() calls.
// Not part of the public API.
#pragma once
#include "NecklaceBoard.h"

namespace NecklaceBoard {
namespace detail {

// Serial over native USB: never block on a full buffer (drop output instead).
void serialNoWait();

// True if a device answers at this address on Wire.
bool i2cPresent(uint8_t addr);

// Starts the QMI8658 on Wire (Wire.begin() must have been called).
void imuInit();

// Adds the fast DMA device to the display's SPI bus and allocates the strip
// buffers. Call after the Arduino_GFX display has started (shared bus).
bool fastInit();

}  // namespace detail
}  // namespace NecklaceBoard
