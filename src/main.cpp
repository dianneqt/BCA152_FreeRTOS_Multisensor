// main.cpp: initialization only.
//   FreeRTOS object creation -> task creation -> scheduler-driven operation
//
// Module map:
//   sensors.cpp       SensorTask  (DHT22 + LDR, vTaskDelayUntil)
//   display.cpp       DisplayTask (owns the OLED)
//   input.cpp         InputTask   (rotary encoder)
//   motion.cpp        MotionTask  (PIR + inactivity timeout)
//   alarm.cpp         AlarmTask   (alarm event + buzzer)
//   alarm_logic.cpp   pure temperature decision (unit tested)
//   state_logic.cpp   pure ACTIVE/INACTIVE decision (unit tested)
//   system_state.cpp  event-group helpers, DisplayMode helpers
//   rtos_objects.cpp  queues, event group, serial mutex, logPrint()

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "esp_err.h"

#include "config.h"
#include "rtos_objects.h"
#include "system_state.h"
#include "sensors.h"
#include "display.h"
#include "input.h"
#include "motion.h"
#include "alarm.h"

// ====================================================
// SERIAL MUTEX TEST (step 36 / fault experiment 3)
//
//   SERIAL_MUTEX_TEST 1            -> start two test tasks
//   SERIAL_MUTEX_TEST_USE_MUTEX 0  -> no protection (output interleaves)
//   SERIAL_MUTEX_TEST_USE_MUTEX 1  -> mutex held for the whole message
//
// Set SERIAL_MUTEX_TEST back to 0 when finished.
// ====================================================

#define SERIAL_MUTEX_TEST 0
#define SERIAL_MUTEX_TEST_USE_MUTEX 0

#if SERIAL_MUTEX_TEST

static void serialTestTask(void *parameter)
{
    const char *name = (const char *)parameter;

    while (1)
    {
#if SERIAL_MUTEX_TEST_USE_MUTEX
        xSemaphoreTake(serialMutex, portMAX_DELAY);
#endif

        printf("[%s] one, ", name);
        printf("two, ");
        printf("three, ");
        printf("four\n");

#if SERIAL_MUTEX_TEST_USE_MUTEX
        xSemaphoreGive(serialMutex);
#endif

        vTaskDelay(ms_to_ticks(10));
    }
}

#endif

extern "C" void app_main(void)
{
    // Creates the serial mutex first, then queues and event group
    bool objectsOk = rtosObjectsInit();

    logPrint("\n");
    logPrint("====================================\n");
    logPrint("BCA152 FreeRTOS Multisensor\n");
    logPrint("System starting...\n");
    logPrint("====================================\n");

    // DHT22 idle state
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DHT_PIN);

    // PIR
    gpio_set_direction(PIR_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(PIR_PIN);

    // GPIO interrupt service (needed by encoder and PIR)
    esp_err_t isrResult = gpio_install_isr_service(0);

    if (isrResult != ESP_OK && isrResult != ESP_ERR_INVALID_STATE)
    {
        logPrint("gpio_install_isr_service failed: %s\n",
                 esp_err_to_name(isrResult));
    }

    if (!objectsOk)
    {
        logPrint("ERROR: Failed to create RTOS objects\n");
        return;
    }

    logPrint("Queues and event group created successfully.\n");

    // The system starts in the ACTIVE state
    xEventGroupSetBits(systemEvents, EVENT_ACTIVE);

    // CPU0: SensorTask, MotionTask
    // CPU1: InputTask, DisplayTask, AlarmTask

    BaseType_t result;

    result = xTaskCreatePinnedToCore(
        sensorTask, "SensorTask", 6144, NULL, PRIORITY_SENSOR, NULL, 0);

    if (result != pdPASS)
    {
        logPrint("ERROR: SensorTask creation failed\n");
        return;
    }

    result = xTaskCreatePinnedToCore(
        motionTask, "MotionTask", 4096, NULL, PRIORITY_MOTION, NULL, 0);

    if (result != pdPASS)
    {
        logPrint("ERROR: MotionTask creation failed\n");
        return;
    }

    result = xTaskCreatePinnedToCore(
        inputTask, "InputTask", 4096, NULL, PRIORITY_INPUT, NULL, 1);

    if (result != pdPASS)
    {
        logPrint("ERROR: InputTask creation failed\n");
        return;
    }

    result = xTaskCreatePinnedToCore(
        displayTask, "DisplayTask", 4096, NULL, PRIORITY_DISPLAY,
        &displayTaskHandle, 1);

    if (result != pdPASS)
    {
        logPrint("ERROR: DisplayTask creation failed\n");
        return;
    }

    result = xTaskCreatePinnedToCore(
        alarmTask, "AlarmTask", 4096, NULL, PRIORITY_ALARM, NULL, 1);

    if (result != pdPASS)
    {
        logPrint("ERROR: AlarmTask creation failed\n");
        return;
    }

#if SERIAL_MUTEX_TEST
    xTaskCreatePinnedToCore(serialTestTask, "SerialTestA", 3072, (void *)"A", 3, NULL, 0);
    xTaskCreatePinnedToCore(serialTestTask, "SerialTestB", 3072, (void *)"B", 3, NULL, 1);
#endif

    logPrint("All tasks started successfully.\n");
}