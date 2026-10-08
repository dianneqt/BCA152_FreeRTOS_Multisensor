#pragma once

// Page navigation logic: no ESP32, no FreeRTOS, unit-testable.

// Pages shown on the OLED (step 28)
enum class DisplayMode
{
    TEMPERATURE,
    HUMIDITY,
    LIGHT,
    MOTION
};

// TEMPERATURE -> HUMIDITY -> LIGHT -> MOTION -> TEMPERATURE (wraps)
DisplayMode nextMode(DisplayMode mode);

// Reverse order, with wraparound (TEMPERATURE -> MOTION)
DisplayMode previousMode(DisplayMode mode);

const char *modeToString(DisplayMode mode);