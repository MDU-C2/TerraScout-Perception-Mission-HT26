#pragma once

#include "config.h"

// Wire packet: 0xA5 | signed int32 left ERPM (little-endian) |
// signed int32 right ERPM (little-endian) | XOR of the preceding nine bytes.
// These are physical motor directions: forward = left negative, right positive.
// The output layer clamps to MAX_MOTOR_ERPM; it does not add another inversion.

uint8_t nuc_checksum(const uint8_t * packet);
