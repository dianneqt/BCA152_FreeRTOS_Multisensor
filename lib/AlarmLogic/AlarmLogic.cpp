#include "AlarmLogic.h"

AlarmState evaluateTemperature(float temperature)
{
    if (temperature < 20.0f)
    {
        return AlarmState::LOW_TEMPERATURE;
    }
    else if (temperature > 30.0f)
    {
        return AlarmState::HIGH_TEMPERATURE;
    }
    else
    {
        return AlarmState::NORMAL;
    }
}