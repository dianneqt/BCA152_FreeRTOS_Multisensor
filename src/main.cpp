#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define DHT_PIN GPIO_NUM_4

// GPIO 34 = ADC1 Channel 6
#define LDR_ADC_CHANNEL ADC_CHANNEL_6

// ----------------------------------------------------
// Sensor Data Structure
// ----------------------------------------------------
struct SensorData
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};

// ----------------------------------------------------
// Sensor Queues
// ----------------------------------------------------
QueueHandle_t displayQueue;
QueueHandle_t alarmQueue;

// ----------------------------------------------------
// DHT22 reading function
// ----------------------------------------------------
static bool dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};

    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_PIN, 0);

    esp_rom_delay_us(1200);

    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);

    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DHT_PIN);

    int64_t start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 1)
    {
        if (esp_timer_get_time() - start > 100)
            return false;
    }

    start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 0)
    {
        if (esp_timer_get_time() - start > 100)
            return false;
    }

    start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 1)
    {
        if (esp_timer_get_time() - start > 100)
            return false;
    }

    for (int i = 0; i < 40; i++)
    {
        start = esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 0)
        {
            if (esp_timer_get_time() - start > 100)
                return false;
        }

        int64_t high_start = esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 1)
        {
            if (esp_timer_get_time() - high_start > 100)
                return false;
        }

        int64_t pulse_length =
            esp_timer_get_time() - high_start;

        int byte_index = i / 8;
        int bit_index = 7 - (i % 8);

        if (pulse_length > 40)
        {
            data[byte_index] |= (1 << bit_index);
        }
    }

    uint8_t checksum =
        data[0] +
        data[1] +
        data[2] +
        data[3];

    if (checksum != data[4])
    {
        return false;
    }

    *humidity =
        ((data[0] << 8) | data[1]) / 10.0f;

    int16_t raw_temperature =
        (data[2] << 8) | data[3];

    if (raw_temperature & 0x8000)
    {
        raw_temperature &= 0x7FFF;
        *temperature =
            -(raw_temperature / 10.0f);
    }
    else
    {
        *temperature =
            raw_temperature / 10.0f;
    }

    return true;
}

// ----------------------------------------------------
// SensorTask
// ----------------------------------------------------
void sensorTask(void *parameter)
{
    SensorData sensorData;

    TickType_t lastWakeTime = xTaskGetTickCount();

    // ADC setup for LDR
    adc_oneshot_unit_handle_t adc_handle = NULL;

    adc_oneshot_unit_init_cfg_t init_config = {};

    init_config.unit_id = ADC_UNIT_1;
    init_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
    init_config.ulp_mode = ADC_ULP_MODE_DISABLE;

    esp_err_t result =
        adc_oneshot_new_unit(
            &init_config,
            &adc_handle
        );

    if (result != ESP_OK)
    {
        printf("LDR ADC initialization failed\n");
        vTaskDelete(NULL);
        return;
    }

    adc_oneshot_chan_cfg_t channel_config = {};

    channel_config.atten = ADC_ATTEN_DB_12;
    channel_config.bitwidth = ADC_BITWIDTH_12;

    result =
        adc_oneshot_config_channel(
            adc_handle,
            LDR_ADC_CHANNEL,
            &channel_config
        );

    if (result != ESP_OK)
    {
        printf("LDR ADC channel configuration failed\n");
        vTaskDelete(NULL);
        return;
    }

    for (;;)
    {
        // --------------------------------------------
        // Read DHT22
        // --------------------------------------------
        if (!dht22_read(
                &sensorData.temperature,
                &sensorData.humidity))
        {
            printf("DHT22 reading failed\n");

            sensorData.temperature = 0.0f;
            sensorData.humidity = 0.0f;
        }

        // --------------------------------------------
        // Read LDR
        // --------------------------------------------
        int raw_value = 0;

        result =
            adc_oneshot_read(
                adc_handle,
                LDR_ADC_CHANNEL,
                &raw_value
            );

        if (result == ESP_OK)
        {
            sensorData.lightLevel =
                (int)((raw_value / 4095.0f) * 100.0f);
        }
        else
        {
            printf("LDR reading failed\n");
            sensorData.lightLevel = 0;
        }

        // Motion sensor has not been added yet.
        sensorData.motionDetected = false;

        // --------------------------------------------
        // Display sensor values
        // --------------------------------------------
        printf(
            "Temperature: %.2f C\n",
            sensorData.temperature
        );

        printf(
            "Humidity: %.2f %%\n",
            sensorData.humidity
        );

        printf(
            "Light Level: %d %%\n",
            sensorData.lightLevel
        );

        // --------------------------------------------
        // Send the same sensor data to both queues
        // --------------------------------------------
        if (xQueueSend(
                displayQueue,
                &sensorData,
                pdMS_TO_TICKS(100)) != pdPASS)
        {
            printf("Display queue full\n");
        }

        if (xQueueSend(
                alarmQueue,
                &sensorData,
                pdMS_TO_TICKS(100)) != pdPASS)
        {
            printf("Alarm queue full\n");
        }

        printf("Sensor data sent to queues\n");

        // Maintain a 2-second periodic schedule
        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}

// ----------------------------------------------------
// DisplayTask
// ----------------------------------------------------
void displayTask(void *parameter)
{
    SensorData sensorData;

    while (1)
    {
        if (xQueueReceive(
                displayQueue,
                &sensorData,
                portMAX_DELAY) == pdPASS)
        {
            // OLED will be added in Part 26.
            // For now, verify that DisplayTask
            // receives the sensor data.
            printf(
                "DisplayTask received: "
                "T=%.2f C, H=%.2f %%, Light=%d %%\n",
                sensorData.temperature,
                sensorData.humidity,
                sensorData.lightLevel
            );
        }
    }
}

// ----------------------------------------------------
// AlarmTask
// ----------------------------------------------------
void alarmTask(void *parameter)
{
    SensorData sensorData;

    while (1)
    {
        if (xQueueReceive(
                alarmQueue,
                &sensorData,
                portMAX_DELAY) == pdPASS)
        {
            // Alarm functionality will be added later.
            // For now, verify that AlarmTask
            // receives the sensor data.
            printf(
                "AlarmTask received sensor data\n"
            );
        }
    }
}

// ----------------------------------------------------
// Main
// ----------------------------------------------------
extern "C" void app_main(void)
{
    printf("\n");
    printf("BCA152 FreeRTOS Multisensor\n");
    printf("System starting...\n");

    // Configure DHT22 pin
    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(DHT_PIN);

    // --------------------------------------------
    // Create queues
    // --------------------------------------------
    displayQueue =
        xQueueCreate(
            5,
            sizeof(SensorData)
        );

    alarmQueue =
        xQueueCreate(
            5,
            sizeof(SensorData)
        );

    if (displayQueue == NULL ||
        alarmQueue == NULL)
    {
        printf("Failed to create sensor queues\n");
        return;
    }

    // --------------------------------------------
    // Create SensorTask
    // --------------------------------------------
    xTaskCreate(
        sensorTask,
        "SensorTask",
        6144,
        NULL,
        2,
        NULL
    );

    // --------------------------------------------
    // Create DisplayTask
    // --------------------------------------------
    xTaskCreate(
        displayTask,
        "DisplayTask",
        4096,
        NULL,
        1,
        NULL
    );

    // --------------------------------------------
    // Create AlarmTask
    // --------------------------------------------
    xTaskCreate(
        alarmTask,
        "AlarmTask",
        4096,
        NULL,
        1,
        NULL
    );
}