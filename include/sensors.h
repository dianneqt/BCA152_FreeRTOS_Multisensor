#pragma once

// SensorTask: reads the DHT22 and LDR every 2 s (vTaskDelayUntil)
// and publishes SensorData to displayQueue and alarmQueue.
void sensorTask(void *parameter);