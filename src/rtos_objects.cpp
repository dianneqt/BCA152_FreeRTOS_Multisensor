#include <stdio.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "rtos_objects.h"
#include "system_state.h"

QueueHandle_t displayQueue = NULL;
QueueHandle_t alarmQueue = NULL;
QueueHandle_t modeQueue = NULL;
QueueHandle_t encoderQueue = NULL;

SemaphoreHandle_t serialMutex = NULL;

TaskHandle_t displayTaskHandle = NULL;

void logPrint(const char *format, ...)
{
    va_list args;
    va_start(args, format);

    // Before the mutex exists (very early boot) just print directly
    if (serialMutex != NULL)
        xSemaphoreTake(serialMutex, portMAX_DELAY);

    vprintf(format, args);

    if (serialMutex != NULL)
        xSemaphoreGive(serialMutex);

    va_end(args);
}

void notifyDisplay()
{
    if (displayTaskHandle != NULL)
        xTaskNotifyGive(displayTaskHandle);
}

bool rtosObjectsInit()
{
    // The mutex must exist before anything prints
    serialMutex =
        xSemaphoreCreateMutex();

    displayQueue =
        xQueueCreate(
            1,
            sizeof(SensorData)
        );

    alarmQueue =
        xQueueCreate(
            1,
            sizeof(SensorData)
        );

    modeQueue =
        xQueueCreate(
            1,
            sizeof(DisplayMode)
        );

    encoderQueue =
        xQueueCreate(
            8,
            sizeof(int8_t)
        );

    systemEvents =
        xEventGroupCreate();

    return
        serialMutex != NULL &&
        displayQueue != NULL &&
        alarmQueue != NULL &&
        modeQueue != NULL &&
        encoderQueue != NULL &&
        systemEvents != NULL;
}