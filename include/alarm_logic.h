#pragma once

// Pure temperature decision logic: no ESP32, no FreeRTOS, unit-testable.

enum class AlarmState
{
    NORMAL,
    LOW_TEMPERATURE,
    HIGH_TEMPERATURE
};

constexpr float LOW_TEMPERATURE_THRESHOLD_C  = 18.0f;
constexpr float HIGH_TEMPERATURE_THRESHOLD_C = 30.0f;

// < 18 C -> LOW_TEMPERATURE, > 30 C -> HIGH_TEMPERATURE, else NORMAL
// (exactly 18 C and exactly 30 C are NORMAL)
AlarmState evaluateTemperature(float temperature);

const char *alarmStateToString(AlarmState state);