#pragma once

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// DisplayMode, nextMode(), previousMode(), modeToString()
// (pure logic, unit-tested: display_mode.h / display_mode.cpp)
#include "display_mode.h"

// ====================================================
// SHARED TYPES
// ====================================================

// Step 32: system state machine
enum class SystemState
{
    ACTIVE,
    INACTIVE
};

// Step 24: data sent from SensorTask to its consumers
struct SensorData
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};

// ====================================================
// SYSTEM EVENTS (step 35): one event group
// ====================================================
//
// Each bit has exactly ONE producer (the only task that sets or
// clears it), which avoids write conflicts and removes the need
// for unsynchronized global variables.
//
// EVENT_ACTIVE  (BIT0)  system is in the ACTIVE state
//   Producer : MotionTask
//   Consumers: InputTask, DisplayTask, AlarmTask
//   Set      : at boot, and when PIR activity restores ACTIVE
//   Cleared  : when 15 s pass without PIR activity (INACTIVE)
//
// EVENT_MOTION  (BIT1)  motion detected (stays set while ACTIVE)
//   Producer : MotionTask
//   Consumers: SensorTask (copies it into SensorData),
//              DisplayTask (MOTION page)
//   Set      : on the PIR rising edge
//   Cleared  : when the 15 s inactivity timeout expires
//              (at the same moment EVENT_ACTIVE is cleared)
//
// EVENT_ALARM   (BIT2)  temperature alarm is active
//   Producer : AlarmTask
//   Consumers: DisplayTask (shows "ALARM" on the OLED)
//   Set      : evaluateTemperature() returns LOW or HIGH_TEMPERATURE
//   Cleared  : temperature returns to NORMAL, or system goes INACTIVE
//
// The event group holds STATE. To wake DisplayTask when something
// changes, task notifications are used (notifyDisplay()):
//   Producers: SensorTask, MotionTask, InputTask, AlarmTask
//   Consumer : DisplayTask   (wakes and redraws)
// and the PIR interrupt wakes MotionTask (vTaskNotifyGiveFromISR).
//
// ====================================================

#define EVENT_ACTIVE (1U << 0)
#define EVENT_MOTION (1U << 1)
#define EVENT_ALARM  (1U << 2)

// Defined in system_state.cpp, created by rtosObjectsInit()
extern EventGroupHandle_t systemEvents;

static inline bool eventIsSet(EventBits_t bit)
{
    return (xEventGroupGetBits(systemEvents) & bit) != 0;
}

// Step 32 state machine, derived from EVENT_ACTIVE
static inline SystemState getSystemState()
{
    return eventIsSet(EVENT_ACTIVE)
        ? SystemState::ACTIVE
        : SystemState::INACTIVE;
}

static inline bool isMotionDetected()
{
    return eventIsSet(EVENT_MOTION);
}

static inline bool isAlarmActive()
{
    return eventIsSet(EVENT_ALARM);
}