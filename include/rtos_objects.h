#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ====================================================
// QUEUES (step 25)
// ====================================================
//
// Created in rtosObjectsInit().
//   displayQueue : SensorData      SensorTask -> DisplayTask
//   alarmQueue   : SensorData      SensorTask -> AlarmTask
//   modeQueue    : DisplayMode     InputTask  -> DisplayTask
//   encoderQueue : int8_t          encoder ISR -> InputTask
//                  (+1 = clockwise, -1 = counter-clockwise)

extern QueueHandle_t displayQueue;
extern QueueHandle_t alarmQueue;
extern QueueHandle_t modeQueue;
extern QueueHandle_t encoderQueue;

// ====================================================
// TICK HELPER
// ====================================================
//
// pdMS_TO_TICKS() rounds DOWN. With a 100 Hz tick (10 ms per tick),
// pdMS_TO_TICKS(5) == 0, and vTaskDelay(0) does NOT block, which
// starves lower-priority tasks and triggers the task watchdog.
// ms_to_ticks() guarantees at least one tick so a task always blocks.

static inline TickType_t ms_to_ticks(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);

    return (ticks == 0) ? 1 : ticks;
}

// ====================================================
// SHARED RESOURCE PROTECTION (step 36): serial output
// ====================================================
//
// Resource : the serial console (UART0, used by printf)
// Users    : every task (Sensor, Motion, Input, Display, Alarm)
// Problem  : two tasks printing at the same time can interleave
//            their output, splitting or mixing multi-part messages
// Solution : serialMutex. A task must hold it while printing.
//
// Use logPrint() instead of printf() everywhere.
//
// Rules:
//  - never call logPrint() from an ISR (a mutex cannot be taken
//    from an interrupt)
//  - never call logPrint() inside portENTER_CRITICAL()/
//    portEXIT_CRITICAL() (taking a mutex can block)
//  - related lines that must stay together are printed with ONE
//    logPrint() call (one Take/Give)
//
// A mutex (not a plain binary semaphore) is used because it has
// priority inheritance.

extern SemaphoreHandle_t serialMutex;

void logPrint(const char *format, ...)
    __attribute__((format(printf, 1, 2)));

// ====================================================
// DISPLAY WAKE-UP (task notification)
// ====================================================

// Handle of DisplayTask (filled in by xTaskCreatePinnedToCore)
extern TaskHandle_t displayTaskHandle;

// Wakes DisplayTask immediately. Safe to call from any task.
void notifyDisplay();

// ====================================================
// INITIALISATION
// ====================================================

// Creates the serial mutex FIRST, then the queues and the event
// group. Returns false if anything could not be created.
bool rtosObjectsInit();