#pragma once

#include <stdint.h>

// How often loop() runs, in seconds.
static constexpr uint32_t LOOP_INTERVAL_S = 1;

// readFrame() is non-blocking; the loop cadence above paces CAN draining.
static constexpr uint32_t CAN_READ_TIMEOUT_MS = 0;
