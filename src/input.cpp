#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"

#include "config.h"
#include "rtos_objects.h"
#include "system_state.h"
#include "input.h"

// ====================================================
// ENCODER INTERRUPT
//
// Wokwi's KY-040 only pulls CLK and DT low for a few ms per
// click, so polling can miss the whole click. An interrupt on
// the falling edge of CLK reads DT right away:
//   DT HIGH -> clockwise      DT LOW -> counter-clockwise
// ====================================================

static void IRAM_ATTR encoderIsr(void *arg)
{
    int8_t direction = (gpio_get_level(ENCODER_DT) != 0) ? 1 : -1;

    if (ENCODER_REVERSE)
        direction = -direction;

    BaseType_t higherPriorityTaskWoken = pdFALSE;

    xQueueSendFromISR(encoderQueue, &direction, &higherPriorityTaskWoken);

    if (higherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

// ====================================================
// INPUT TASK
//
// Clockwise:         TEMPERATURE -> HUMIDITY -> LIGHT -> MOTION -> TEMPERATURE
// Counter-clockwise: reverse, with wraparound
// Button press:      next page (same order as clockwise)
//
// Blocks on encoderQueue (max 20 ms): never busy-loops.
// ====================================================

void inputTask(void *parameter)
{
    gpio_set_direction(ENCODER_CLK, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_DT,  GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_SW,  GPIO_MODE_INPUT);

    gpio_pullup_en(ENCODER_CLK);
    gpio_pullup_en(ENCODER_DT);
    gpio_pullup_en(ENCODER_SW);

    // Interrupt on CLK falling edge
    gpio_set_intr_type(ENCODER_CLK, GPIO_INTR_NEGEDGE);

    esp_err_t err = gpio_install_isr_service(0);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        logPrint("gpio_install_isr_service failed: %s\n",
                 esp_err_to_name(err));
    }

    gpio_isr_handler_add(ENCODER_CLK, encoderIsr, NULL);

    DisplayMode currentMode = DisplayMode::TEMPERATURE;

    bool lastButton = gpio_get_level(ENCODER_SW);

    logPrint("InputTask: Started on CPU %d\n", xPortGetCoreID());

    while (1)
    {
        // ---------------- ROTATION ----------------

        int8_t direction = 0;

        if (xQueueReceive(encoderQueue, &direction, ms_to_ticks(20)) == pdPASS)
        {
            // Encoder is only active in the ACTIVE state (step 33/34)
            if (getSystemState() == SystemState::ACTIVE)
            {
                if (direction > 0)
                {
                    currentMode = nextMode(currentMode);
                    logPrint("InputTask: clockwise -> %s\n",
                             modeToString(currentMode));
                }
                else
                {
                    currentMode = previousMode(currentMode);
                    logPrint("InputTask: counter-clockwise -> %s\n",
                             modeToString(currentMode));
                }

                xQueueOverwrite(modeQueue, &currentMode);
                notifyDisplay();
            }
        }

        // ---------------- BUTTON: next page ----------------

        bool currentButton = gpio_get_level(ENCODER_SW);

        if (lastButton == 1 &&
            currentButton == 0 &&
            getSystemState() == SystemState::ACTIVE)
        {
            currentMode = nextMode(currentMode);

            xQueueOverwrite(modeQueue, &currentMode);
            notifyDisplay();

            logPrint("InputTask: button pressed -> %s\n",
                     modeToString(currentMode));

            // Simple debounce
            vTaskDelay(ms_to_ticks(50));
        }

        lastButton = currentButton;
    }
}