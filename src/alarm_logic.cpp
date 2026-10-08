#include "alarm_logic.h"

AlarmState evaluateTemperature(float temperature)
{
    if (temperature < LOW_TEMPERATURE_THRESHOLD_C)
        return AlarmState::LOW_TEMPERATURE;

    if (temperature > HIGH_TEMPERATURE_THRESHOLD_C)
        return AlarmState::HIGH_TEMPERATURE;

    return AlarmState::NORMAL;
}

const char *alarmStateToString(AlarmState state)
{
    switch (state)
    {
        case AlarmState::NORMAL:           return "NORMAL";
        case AlarmState::LOW_TEMPERATURE:  return "LOW_TEMPERATURE";
        case AlarmState::HIGH_TEMPERATURE: return "HIGH_TEMPERATURE";
    }

    return "UNKNOWN";
}