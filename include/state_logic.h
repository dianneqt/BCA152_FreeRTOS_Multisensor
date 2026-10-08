#pragma once

#include <stdint.h>

// Pure ACTIVE/INACTIVE decision logic: no ESP32, no FreeRTOS, unit-testable.

// Laboratory testing value (step 31)
constexpr int64_t INACTIVITY_TIMEOUT_MS = 15000;

// Returns true if the system should be ACTIVE after this evaluation.
//
//   motion detected                      -> ACTIVE   (wakes from INACTIVE too)
//   ACTIVE,   no motion, timeout reached -> INACTIVE
//   ACTIVE,   no motion, timeout not yet -> ACTIVE
//   INACTIVE, no motion                  -> INACTIVE
bool evaluateSystemState(
    bool currentlyActive,
    bool motionDetected,
    int64_t msSinceLastMotion,
    int64_t timeoutMs
);