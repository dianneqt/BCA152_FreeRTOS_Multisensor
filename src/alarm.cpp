#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/ledc.h"
#include "esp_err.h"

#include "config.h"
#include "rtos_objects.h"
#include "system_state.h"
#include "alarm_logic.h"
#include "alarm.h"

// ====================================================
// BUZZER (hardware driver)
//
// Wokwi's buzzer only makes sound with a square wave, so the pin
// is driven by LEDC PWM (50 % duty). A different tone is used for
// each alarm: HIGH = 2000 Hz, LOW = 1000 Hz.
// ====================================================

#define BUZZER_SPEED_MODE   LEDC_LOW_SPEED_MODE
#define BUZZER_TIMER        LEDC_TIMER_0
#define BUZZER_CHANNEL      LEDC_CHANNEL_0
#define BUZZER_DUTY_ON      512          // 50 % of 10-bit resolution

static void buzzerInit()
{
    ledc_timer_config_t timer = {};
    timer.speed_mode      = BUZZER_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num       = BUZZER_TIMER;
    timer.freq_hz         = 2000;
    timer.clk_cfg         = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer);

    if (err != ESP_OK)
        logPrint("Buzzer timer config failed: %s\n", esp_err_to_name(err));

    ledc_channel_config_t channel = {};
    channel.gpio_num   = BUZZER_PIN;
    channel.speed_mode = BUZZER_SPEED_MODE;
    channel.channel    = BUZZER_CHANNEL;
    channel.timer_sel  = BUZZER_TIMER;
    channel.duty       = 0;                 // silent at start
    channel.hpoint     = 0;

    err = ledc_channel_config(&channel);

    if (err != ESP_OK)
        logPrint("Buzzer channel config failed: %s\n", esp_err_to_name(err));
}

static void buzzerSet(AlarmState state)
{
    if (state == AlarmState::NORMAL)
    {
        ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, 0);
        ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
        return;
    }

    uint32_t frequency =
        (state == AlarmState::HIGH_TEMPERATURE) ? 2000 : 1000;

    ledc_set_freq(BUZZER_SPEED_MODE, BUZZER_TIMER, frequency);
    ledc_set_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL, BUZZER_DUTY_ON);
    ledc_update_duty(BUZZER_SPEED_MODE, BUZZER_CHANNEL);
}

// ====================================================
// ALARM TASK
// ====================================================

void alarmTask(void *parameter)
{
    SensorData sensorData = {};

    AlarmState lastState = AlarmState::NORMAL;
    bool firstEvaluation = true;

    buzzerInit();

    logPrint("AlarmTask: Started on CPU %d\n", xPortGetCoreID());

    while (1)
    {
        if (xQueueReceive(alarmQueue, &sensorData, portMAX_DELAY) == pdPASS)
        {
            // Alarm is only active in ACTIVE state (step 33/34)
            if (getSystemState() != SystemState::ACTIVE)
            {
                buzzerSet(AlarmState::NORMAL);

                if (isAlarmActive())
                {
                    xEventGroupClearBits(systemEvents, EVENT_ALARM);
                    logPrint("Alarm cleared - system INACTIVE\n");
                }

                logPrint("Status: INACTIVE | Alarm: OFF\n");

                firstEvaluation = true;
                continue;
            }

            AlarmState state = evaluateTemperature(sensorData.temperature);

            if (firstEvaluation || state != lastState)
            {
                logPrint("Alarm State: %s (%.1f C)\n",
                         alarmStateToString(state),
                         sensorData.temperature);

                // Publish the result as an event
                if (state == AlarmState::NORMAL)
                    xEventGroupClearBits(systemEvents, EVENT_ALARM);
                else
                    xEventGroupSetBits(systemEvents, EVENT_ALARM);

                buzzerSet(state);

                notifyDisplay();

                lastState = state;
                firstEvaluation = false;
            }

            // One status line per sample so the state is always visible
            logPrint("Status: ACTIVE | Alarm: %s\n",
                     alarmStateToString(state));
        }
    }
}