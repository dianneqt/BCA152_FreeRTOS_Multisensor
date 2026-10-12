#pragma once

// Hardware pins, hardware settings and task priorities.
// (Pure decision constants live in alarm_logic.h and state_logic.h so
//  they can be unit tested on the PC without any ESP-IDF headers.)

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"

// ---------------- PIN DEFINITIONS ----------------
#define DHT_PIN          GPIO_NUM_4
#define LDR_ADC_CHANNEL  ADC_CHANNEL_6      // GPIO34 (ADC1)

#define OLED_SDA         GPIO_NUM_21
#define OLED_SCL         GPIO_NUM_22
#define OLED_ADDR        0x3C
#define I2C_PORT         I2C_NUM_0

#define ENCODER_CLK      GPIO_NUM_32
#define ENCODER_DT       GPIO_NUM_33
#define ENCODER_SW       GPIO_NUM_25

#define PIR_PIN          GPIO_NUM_27
#define BUZZER_PIN       GPIO_NUM_26

// ---------------- DHT22 TIMING ----------------
#define DHT_TIMEOUT_US          150
#define DHT_BIT_TIMEOUT_US      120
#define DHT_ONE_THRESHOLD_US    50

// If clockwise ever moves BACKWARDS through the pages, set to true.
constexpr bool ENCODER_REVERSE = false;

// ---------------- TASK PRIORITIES (steps 38-39) ----------------
// Priority = scheduling urgency (how fast the task must react),
// not "importance". The FreeRTOS idle task runs at priority 0.
//
//  Task         Prio  Core  Why
//  MotionTask     3     0   Must react promptly to PIR edges; short work, then blocks.
//  InputTask      3     1   Encoder clicks must feel instant; blocks on its queue.
//  SensorTask     2     0   Periodic 2 s; small jitter OK, keeps its period.
//  AlarmTask      2     1   Decision on a new sample; hundreds of ms tolerable.
//  DisplayTask    1     1   Slow I2C drawing; tolerates latency, must not delay others.
constexpr UBaseType_t PRIORITY_MOTION  = 3;
constexpr UBaseType_t PRIORITY_INPUT   = 3;
constexpr UBaseType_t PRIORITY_SENSOR  = 2;
constexpr UBaseType_t PRIORITY_ALARM   = 2;
constexpr UBaseType_t PRIORITY_DISPLAY = 1;