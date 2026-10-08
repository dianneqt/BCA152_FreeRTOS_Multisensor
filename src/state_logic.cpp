#include "state_logic.h"

bool evaluateSystemState(
    bool currentlyActive,
    bool motionDetected,
    int64_t msSinceLastMotion,
    int64_t timeoutMs
)
{
    if (motionDetected)
        return true;

    if (currentlyActive)
        return msSinceLastMotion < timeoutMs;

    return false;
}