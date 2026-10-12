#pragma once

// AlarmTask: blocks on alarmQueue, evaluates the temperature with
// evaluateTemperature() (alarm_logic.h), sets/clears EVENT_ALARM and
// drives the buzzer (LEDC PWM on BUZZER_PIN).
void alarmTask(void *parameter);