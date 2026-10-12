#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"

#include "config.h"
#include "rtos_objects.h"
#include "system_state.h"
#include "state_logic.h"
#include "motion.h"

// ====================================================
// MOTION TASK  (steps 31-34)
//
//   ACTIVE   --(no PIR activity for 15 s)-->  INACTIVE
//   INACTIVE --(PIR activity)------------->  ACTIVE
//
// The decision itself is the pure function evaluateSystemState()
// (state_logic.cpp). This task only gathers inputs and applies
// the result to the event group.
// ====================================================

// Handle of this task, set when it starts (used by the ISR).
static TaskHandle_t motionTaskHandle = NULL;

// PIR interrupt: fires on BOTH edges of the PIR output and wakes
// MotionTask immediately, so it does not have to poll.
static void IRAM_ATTR pirIsr(void *arg)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (motionTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(motionTaskHandle, &higherPriorityTaskWoken);
    }

    if (higherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

void motionTask(void *parameter)
{
    motionTaskHandle = xTaskGetCurrentTaskHandle();

    gpio_set_direction(PIR_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(PIR_PIN);

    // Interrupt on both edges of the PIR output
    gpio_set_intr_type(PIR_PIN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(PIR_PIN, pirIsr, NULL);

    int64_t lastMotionTime = esp_timer_get_time();
    bool lastPirLevel = false;

    logPrint("MotionTask: Started on CPU %d\n", xPortGetCoreID());

    while (1)
    {
        // Sleep until the PIR output changes (interrupt) or 100 ms
        // pass (needed to check the inactivity timeout).
        ulTaskNotifyTake(pdTRUE, ms_to_ticks(100));

        bool pirLevel = gpio_get_level(PIR_PIN) != 0;
        int64_t now = esp_timer_get_time();

        // PIR output changed (either direction)
        if (pirLevel != lastPirLevel)
        {
            lastPirLevel = pirLevel;

            if (pirLevel)
            {
                // Motion stays flagged until the inactivity timeout,
                // so it is NOT cleared on the falling edge.
                xEventGroupSetBits(systemEvents, EVENT_MOTION);
                logPrint("MotionTask: MOTION DETECTED\n");
            }
            else
            {
                logPrint("MotionTask: motion ended\n");
            }

            notifyDisplay();
        }

        if (pirLevel)
            lastMotionTime = now;

        bool wasActive = (getSystemState() == SystemState::ACTIVE);

        bool nowActive = evaluateSystemState(
            wasActive,
            pirLevel,
            (now - lastMotionTime) / 1000,
            INACTIVITY_TIMEOUT_MS
        );

        if (nowActive && !wasActive)
        {
            xEventGroupSetBits(systemEvents, EVENT_ACTIVE);
            logPrint("System entering ACTIVE mode\n");
            notifyDisplay();
        }
        else if (!nowActive && wasActive)
        {
            xEventGroupClearBits(systemEvents, EVENT_ACTIVE | EVENT_MOTION);

            logPrint("MotionTask: NO MOTION - "
                     "15 SECOND TIMEOUT\n"
                     "System entering INACTIVE mode\n");

            notifyDisplay();
        }
    }
}