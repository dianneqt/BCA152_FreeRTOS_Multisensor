#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_err.h"

#include "config.h"
#include "rtos_objects.h"
#include "system_state.h"
#include "sensors.h"

// ====================================================
// DHT22
// ====================================================

static portMUX_TYPE dhtMux = portMUX_INITIALIZER_UNLOCKED;

static bool dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};
    bool success = false;

    // Error text is stored here and printed AFTER the critical
    // section ends (a mutex must not be taken inside one).
    char errorMessage[64];
    errorMessage[0] = '\0';

    portENTER_CRITICAL(&dhtMux);

    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_PIN, 0);
    esp_rom_delay_us(1200);
    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DHT_PIN);

    // Sensor response LOW
    int64_t start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 1)
    {
        if ((esp_timer_get_time() - start) > DHT_TIMEOUT_US)
            break;
    }

    bool responseLow = (gpio_get_level(DHT_PIN) == 0);

    // Sensor response HIGH
    if (responseLow)
    {
        start = esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 0)
        {
            if ((esp_timer_get_time() - start) > DHT_TIMEOUT_US)
                break;
        }
    }

    bool responseHigh = (gpio_get_level(DHT_PIN) == 1);

    if (responseLow && responseHigh)
    {
        start = esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 1)
        {
            if ((esp_timer_get_time() - start) > DHT_TIMEOUT_US)
                break;
        }

        bool firstBitStarted = (gpio_get_level(DHT_PIN) == 0);

        if (firstBitStarted)
        {
            bool readOK = true;

            for (int i = 0; i < 40; i++)
            {
                start = esp_timer_get_time();

                while (gpio_get_level(DHT_PIN) == 0)
                {
                    if ((esp_timer_get_time() - start) > DHT_BIT_TIMEOUT_US)
                    {
                        readOK = false;
                        break;
                    }
                }

                if (!readOK)
                    break;

                int64_t highStart = esp_timer_get_time();

                while (gpio_get_level(DHT_PIN) == 1)
                {
                    if ((esp_timer_get_time() - highStart) > DHT_BIT_TIMEOUT_US)
                    {
                        readOK = false;
                        break;
                    }
                }

                if (!readOK)
                    break;

                int64_t pulseLength = esp_timer_get_time() - highStart;

                int byteIndex = i / 8;
                int bitIndex  = 7 - (i % 8);

                if (pulseLength > DHT_ONE_THRESHOLD_US)
                    data[byteIndex] |= (uint8_t)(1U << bitIndex);
            }

            if (readOK)
            {
                uint8_t checksum =
                    (uint8_t)(data[0] + data[1] + data[2] + data[3]);

                if (checksum == data[4])
                {
                    uint16_t rawHumidity =
                        ((uint16_t)data[0] << 8) | data[1];

                    uint16_t rawTemperature =
                        ((uint16_t)data[2] << 8) | data[3];

                    float newHumidity = rawHumidity / 10.0f;

                    bool negative = (rawTemperature & 0x8000U) != 0;
                    rawTemperature &= 0x7FFFU;

                    float newTemperature = rawTemperature / 10.0f;

                    if (negative)
                        newTemperature = -newTemperature;

                    if (newHumidity >= 0.0f && newHumidity <= 100.0f &&
                        newTemperature >= -40.0f && newTemperature <= 80.0f)
                    {
                        *humidity = newHumidity;
                        *temperature = newTemperature;
                        success = true;
                    }
                    else
                    {
                        snprintf(errorMessage, sizeof(errorMessage),
                                 "DHT22 invalid values: T=%.2f C H=%.2f %%",
                                 newTemperature, newHumidity);
                    }
                }
                else
                {
                    snprintf(errorMessage, sizeof(errorMessage),
                             "DHT22 checksum failed (calc=%u received=%u)",
                             (unsigned)checksum, (unsigned)data[4]);
                }
            }
            else
            {
                snprintf(errorMessage, sizeof(errorMessage),
                         "DHT22 timing/read failed");
            }
        }
    }

    if (!success && !(responseLow && responseHigh))
    {
        snprintf(errorMessage, sizeof(errorMessage),
                 "DHT22 sensor response timeout");
    }

    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DHT_PIN);

    portEXIT_CRITICAL(&dhtMux);

    if (errorMessage[0] != '\0')
        logPrint("%s\n", errorMessage);

    return success;
}

// ====================================================
// SENSOR TASK
// ====================================================

void sensorTask(void *parameter)
{
    SensorData sensorData = {};

    TickType_t lastWakeTime = xTaskGetTickCount();

    adc_oneshot_unit_handle_t adc_handle = NULL;

    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id  = ADC_UNIT_1;
    init_config.clk_src  = ADC_RTC_CLK_SRC_DEFAULT;
    init_config.ulp_mode = ADC_ULP_MODE_DISABLE;

    esp_err_t result = adc_oneshot_new_unit(&init_config, &adc_handle);

    if (result != ESP_OK)
    {
        logPrint("LDR ADC initialization failed: %s\n",
                 esp_err_to_name(result));
        vTaskDelete(NULL);
        return;
    }

    adc_oneshot_chan_cfg_t channel_config = {};
    channel_config.atten    = ADC_ATTEN_DB_12;
    channel_config.bitwidth = ADC_BITWIDTH_12;

    result = adc_oneshot_config_channel(
        adc_handle, LDR_ADC_CHANNEL, &channel_config);

    if (result != ESP_OK)
    {
        logPrint("LDR ADC channel configuration failed: %s\n",
                 esp_err_to_name(result));
        vTaskDelete(NULL);
        return;
    }

    sensorData.temperature    = 0.0f;
    sensorData.humidity       = 0.0f;
    sensorData.lightLevel     = 0;
    sensorData.motionDetected = false;

    bool haveValidDhtReading = false;

    logPrint("SensorTask: Started on CPU %d\n", xPortGetCoreID());

    while (1)
    {
        // --------------------------------------------
        // DHT22
        // --------------------------------------------

        float newTemperature = 0.0f;
        float newHumidity    = 0.0f;

        if (dht22_read(&newTemperature, &newHumidity))
        {
            sensorData.temperature = newTemperature;
            sensorData.humidity    = newHumidity;
            haveValidDhtReading    = true;
        }
        else
        {
            if (haveValidDhtReading)
                logPrint("DHT22 reading failed - "
                         "keeping previous valid reading\n");
            else
                logPrint("DHT22 reading failed - "
                         "no valid reading yet\n");
        }

        // --------------------------------------------
        // LDR (raw 0-4095 converted to relative 0-100 %)
        // --------------------------------------------

        int raw_value = 0;

        result = adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &raw_value);

        if (result == ESP_OK)
        {
            // The Wokwi LDR module's AO output falls as light increases,
            // so invert the reading: 100 % = bright, 0 % = dark.
            sensorData.lightLevel =
                100 - (int)((raw_value / 4095.0f) * 100.0f);

            if (sensorData.lightLevel < 0)
                sensorData.lightLevel = 0;

            if (sensorData.lightLevel > 100)
                sensorData.lightLevel = 100;
        }
        else
        {
            sensorData.lightLevel = 0;
        }

        // --------------------------------------------
        // MOTION STATE (owned by MotionTask)
        // --------------------------------------------

        sensorData.motionDetected = isMotionDetected();

        // --------------------------------------------
        // SERIAL OUTPUT (one protected write)
        // --------------------------------------------

        logPrint(
            "Temperature: %.2f C\n"
            "Humidity: %.2f %%\n"
            "Light Level: %d %%\n"
            "Motion: %s\n",
            sensorData.temperature,
            sensorData.humidity,
            sensorData.lightLevel,
            sensorData.motionDetected ? "DETECTED" : "NONE"
        );

        // Only publish once a real DHT22 reading exists, so the
        // alarm never evaluates the placeholder 0.0 C value.
        if (haveValidDhtReading)
        {
            xQueueOverwrite(displayQueue, &sensorData);
            xQueueOverwrite(alarmQueue,   &sensorData);

            logPrint("Sensor data sent to queues\n");

            notifyDisplay();
        }

        // Periodic sampling without drift (step 22).
        // DHT22 requires about 2 s between measurements.
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2000));
    }
}