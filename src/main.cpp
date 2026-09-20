#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define DHT_PIN GPIO_NUM_4

// ----------------------------------------------------
// DHT22 reading function
// ----------------------------------------------------
static bool dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};

    // Start signal
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_PIN, 0);

    // DHT22 requires at least 1 ms LOW
    esp_rom_delay_us(1200);

    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);

    // Listen for DHT22 response
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DHT_PIN);

    // Wait for response LOW
    int64_t start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 1)
    {
        if (esp_timer_get_time() - start > 100)
            return false;
    }

    // Wait for response HIGH
    start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 0)
    {
        if (esp_timer_get_time() - start > 100)
            return false;
    }

    // Wait for response LOW
    start = esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 1)
    {
        if (esp_timer_get_time() - start > 100)
            return false;
    }

    // Read 40 bits
    for (int i = 0; i < 40; i++)
    {
        // Wait for the beginning of the bit
        start = esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 0)
        {
            if (esp_timer_get_time() - start > 100)
                return false;
        }

        // Measure HIGH pulse
        int64_t high_start = esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 1)
        {
            if (esp_timer_get_time() - high_start > 100)
                return false;
        }

        int64_t pulse_length = esp_timer_get_time() - high_start;

        // Around 26-28 us = 0
        // Around 70 us = 1
        int byte_index = i / 8;
        int bit_index = 7 - (i % 8);

        if (pulse_length > 40)
        {
            data[byte_index] |= (1 << bit_index);
        }
    }

    // Verify checksum
    uint8_t checksum =
        data[0] +
        data[1] +
        data[2] +
        data[3];

    if (checksum != data[4])
    {
        return false;
    }

    // Humidity
    *humidity = ((data[0] << 8) | data[1]) / 10.0f;

    // Temperature
    int16_t raw_temperature = (data[2] << 8) | data[3];

    if (raw_temperature & 0x8000)
    {
        raw_temperature &= 0x7FFF;
        *temperature = -(raw_temperature / 10.0f);
    }
    else
    {
        *temperature = raw_temperature / 10.0f;
    }

    return true;
}

// ----------------------------------------------------
// Task A
// ----------------------------------------------------
void taskA(void *parameter)
{
    while (1)
    {
        printf("Task A running\n");

        // Block for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ----------------------------------------------------
// Task B
// ----------------------------------------------------
void taskB(void *parameter)
{
    while (1)
    {
        printf("Task B running\n");

        // Block for 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ----------------------------------------------------
// DHT22 Sensor Task
// ----------------------------------------------------
void sensorTask(void *parameter)
{
    float temperature;
    float humidity;

    while (1)
    {
        if (dht22_read(&temperature, &humidity))
        {
            printf("Temperature: %.2f C\n", temperature);
            printf("Humidity: %.2f %%\n", humidity);
        }
        else
        {
            printf("DHT22 reading failed\n");
        }

        // DHT22 should not be read too frequently.
        // Wait 2 seconds before the next reading.
        printf("SensorTask waiting 2 sec\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
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
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(DHT_PIN);

    // Create Task A
    xTaskCreate(
        taskA,
        "Task A",
        2048,
        NULL,
        1,
        NULL
    );

    // Create Task B
    xTaskCreate(
        taskB,
        "Task B",
        2048,
        NULL,
        1,
        NULL
    );

    // Create DHT22 Sensor Task
    xTaskCreate(
        sensorTask,
        "SensorTask",
        4096,
        NULL,
        2,
        NULL
    );
}