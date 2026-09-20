#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define DHT_PIN GPIO_NUM_4

// LDR
#define LDR_ADC_CHANNEL ADC_CHANNEL_6   // GPIO34

// OLED I2C
#define OLED_SDA GPIO_NUM_21
#define OLED_SCL GPIO_NUM_22
#define OLED_ADDR 0x3C
#define I2C_PORT I2C_NUM_0

// ----------------------------------------------------
// Sensor Data
// ----------------------------------------------------
struct SensorData
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};

// ----------------------------------------------------
// Queues
// ----------------------------------------------------
QueueHandle_t displayQueue;
QueueHandle_t alarmQueue;

// ----------------------------------------------------
// OLED functions
// ----------------------------------------------------
static void oled_command(uint8_t command)
{
    uint8_t data[2];

    data[0] = 0x00;
    data[1] = command;

    i2c_master_write_to_device(
        I2C_PORT,
        OLED_ADDR,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    );
}

static void oled_data(const uint8_t *data, size_t length)
{
    uint8_t buffer[129];

    if (length > 128)
        return;

    buffer[0] = 0x40;

    for (size_t i = 0; i < length; i++)
    {
        buffer[i + 1] = data[i];
    }

    i2c_master_write_to_device(
        I2C_PORT,
        OLED_ADDR,
        buffer,
        length + 1,
        pdMS_TO_TICKS(100)
    );
}

// ----------------------------------------------------
// OLED initialization
// ----------------------------------------------------
static void oled_init()
{
    i2c_config_t config = {};

    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = OLED_SDA;
    config.scl_io_num = OLED_SCL;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = 400000;

    i2c_param_config(I2C_PORT, &config);

    i2c_driver_install(
        I2C_PORT,
        config.mode,
        0,
        0,
        0
    );

    vTaskDelay(pdMS_TO_TICKS(100));

    oled_command(0xAE);
    oled_command(0xD5);
    oled_command(0x80);
    oled_command(0xA8);
    oled_command(0x3F);
    oled_command(0xD3);
    oled_command(0x00);
    oled_command(0x40);
    oled_command(0x8D);
    oled_command(0x14);
    oled_command(0x20);
    oled_command(0x00);
    oled_command(0xA1);
    oled_command(0xC8);
    oled_command(0xDA);
    oled_command(0x12);
    oled_command(0x81);
    oled_command(0xCF);
    oled_command(0xD9);
    oled_command(0xF1);
    oled_command(0xDB);
    oled_command(0x40);
    oled_command(0xA4);
    oled_command(0xA6);
    oled_command(0xAF);
}

// ----------------------------------------------------
// Clear OLED
// ----------------------------------------------------
static void oled_clear()
{
    uint8_t blank[128] = {0};

    for (int page = 0; page < 8; page++)
    {
        oled_command(0xB0 + page);
        oled_command(0x00);
        oled_command(0x10);

        oled_data(blank, 128);
    }
}

// ----------------------------------------------------
// Set OLED cursor
// ----------------------------------------------------
static void oled_set_cursor(int x, int page)
{
    oled_command(0xB0 + page);
    oled_command(0x00 + (x & 0x0F));
    oled_command(0x10 + ((x >> 4) & 0x0F));
}

// ----------------------------------------------------
// Draw one character
// ----------------------------------------------------
static void oled_char(int x, int page, char c)
{
    uint8_t pixels[6] = {0, 0, 0, 0, 0, 0};

    switch (c)
    {
        // ----------------------------
        // Numbers
        // ----------------------------
        case '0':
            pixels[0] = 0x3E;
            pixels[1] = 0x51;
            pixels[2] = 0x49;
            pixels[3] = 0x45;
            pixels[4] = 0x3E;
            break;

        case '1':
            pixels[0] = 0x00;
            pixels[1] = 0x42;
            pixels[2] = 0x7F;
            pixels[3] = 0x40;
            pixels[4] = 0x00;
            break;

        case '2':
            pixels[0] = 0x42;
            pixels[1] = 0x61;
            pixels[2] = 0x51;
            pixels[3] = 0x49;
            pixels[4] = 0x46;
            break;

        case '3':
            pixels[0] = 0x21;
            pixels[1] = 0x41;
            pixels[2] = 0x45;
            pixels[3] = 0x4B;
            pixels[4] = 0x31;
            break;

        case '4':
            pixels[0] = 0x18;
            pixels[1] = 0x14;
            pixels[2] = 0x12;
            pixels[3] = 0x7F;
            pixels[4] = 0x10;
            break;

        case '5':
            pixels[0] = 0x27;
            pixels[1] = 0x45;
            pixels[2] = 0x45;
            pixels[3] = 0x45;
            pixels[4] = 0x39;
            break;

        case '6':
            pixels[0] = 0x3C;
            pixels[1] = 0x4A;
            pixels[2] = 0x49;
            pixels[3] = 0x49;
            pixels[4] = 0x30;
            break;

        case '7':
            pixels[0] = 0x01;
            pixels[1] = 0x71;
            pixels[2] = 0x09;
            pixels[3] = 0x05;
            pixels[4] = 0x03;
            break;

        case '8':
            pixels[0] = 0x36;
            pixels[1] = 0x49;
            pixels[2] = 0x49;
            pixels[3] = 0x49;
            pixels[4] = 0x36;
            break;

        case '9':
            pixels[0] = 0x06;
            pixels[1] = 0x49;
            pixels[2] = 0x49;
            pixels[3] = 0x29;
            pixels[4] = 0x1E;
            break;

        // ----------------------------
        // Letters
        // ----------------------------
        case 'A':
            pixels[0] = 0x7E;
            pixels[1] = 0x11;
            pixels[2] = 0x11;
            pixels[3] = 0x11;
            pixels[4] = 0x7E;
            break;

        case 'C':
            pixels[0] = 0x3E;
            pixels[1] = 0x41;
            pixels[2] = 0x41;
            pixels[3] = 0x41;
            pixels[4] = 0x22;
            break;

        case 'E':
            pixels[0] = 0x7F;
            pixels[1] = 0x49;
            pixels[2] = 0x49;
            pixels[3] = 0x49;
            pixels[4] = 0x41;
            break;

        case 'G':
            pixels[0] = 0x3E;
            pixels[1] = 0x41;
            pixels[2] = 0x49;
            pixels[3] = 0x49;
            pixels[4] = 0x7A;
            break;

        case 'H':
            pixels[0] = 0x7F;
            pixels[1] = 0x08;
            pixels[2] = 0x08;
            pixels[3] = 0x08;
            pixels[4] = 0x7F;
            break;

        case 'I':
            pixels[0] = 0x00;
            pixels[1] = 0x41;
            pixels[2] = 0x7F;
            pixels[3] = 0x41;
            pixels[4] = 0x00;
            break;

        case 'L':
            pixels[0] = 0x7F;
            pixels[1] = 0x40;
            pixels[2] = 0x40;
            pixels[3] = 0x40;
            pixels[4] = 0x40;
            break;

        case 'M':
            pixels[0] = 0x7F;
            pixels[1] = 0x02;
            pixels[2] = 0x0C;
            pixels[3] = 0x02;
            pixels[4] = 0x7F;
            break;

        case 'P':
            pixels[0] = 0x7F;
            pixels[1] = 0x09;
            pixels[2] = 0x09;
            pixels[3] = 0x09;
            pixels[4] = 0x06;
            break;

        case 'T':
            pixels[0] = 0x01;
            pixels[1] = 0x01;
            pixels[2] = 0x7F;
            pixels[3] = 0x01;
            pixels[4] = 0x01;
            break;

        case 'U':
            pixels[0] = 0x3F;
            pixels[1] = 0x40;
            pixels[2] = 0x40;
            pixels[3] = 0x40;
            pixels[4] = 0x3F;
            break;

        // ----------------------------
        // Symbols
        // ----------------------------
        case '%':
            pixels[0] = 0x63;
            pixels[1] = 0x13;
            pixels[2] = 0x08;
            pixels[3] = 0x64;
            pixels[4] = 0x63;
            break;

        case '-':
            pixels[0] = 0x08;
            pixels[1] = 0x08;
            pixels[2] = 0x08;
            pixels[3] = 0x08;
            pixels[4] = 0x08;
            break;

        case ' ':
        default:
            break;
    }

    oled_set_cursor(x, page);
    oled_data(pixels, 6);
}

// ----------------------------------------------------
// Draw integer number
// ----------------------------------------------------
static void oled_number(int x, int page, int number)
{
    if (number < 0)
    {
        oled_char(x, page, '-');
        number = -number;
        x += 6;
    }

    if (number >= 100)
    {
        oled_char(x, page, '0' + (number / 100));
        number %= 100;
        x += 6;
    }

    if (number >= 10)
    {
        oled_char(x, page, '0' + (number / 10));
        number %= 10;
        x += 6;
    }

    oled_char(x, page, '0' + number);
}

// ----------------------------------------------------
// DHT22 reading function
// ----------------------------------------------------
static bool dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_OUTPUT
    );

    gpio_set_level(DHT_PIN, 0);

    esp_rom_delay_us(1200);

    gpio_set_level(DHT_PIN, 1);

    esp_rom_delay_us(30);

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

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

    // Read 40 bits
    for (int i = 0; i < 40; i++)
    {
        start = esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 0)
        {
            if (esp_timer_get_time() - start > 100)
                return false;
        }

        int64_t high_start =
            esp_timer_get_time();

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
            data[byte_index] |=
                (1 << bit_index);
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

    TickType_t lastWakeTime =
        xTaskGetTickCount();

    // ----------------------------
    // Initialize ADC for LDR
    // ----------------------------
    adc_oneshot_unit_handle_t adc_handle =
        NULL;

    adc_oneshot_unit_init_cfg_t init_config = {};

    init_config.unit_id = ADC_UNIT_1;
    init_config.clk_src =
        ADC_RTC_CLK_SRC_DEFAULT;
    init_config.ulp_mode =
        ADC_ULP_MODE_DISABLE;

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

    channel_config.atten =
        ADC_ATTEN_DB_12;

    channel_config.bitwidth =
        ADC_BITWIDTH_12;

    result =
        adc_oneshot_config_channel(
            adc_handle,
            LDR_ADC_CHANNEL,
            &channel_config
        );

    if (result != ESP_OK)
    {
        printf(
            "LDR ADC channel configuration failed\n"
        );

        vTaskDelete(NULL);
        return;
    }

    // ----------------------------
    // Periodic sensor loop
    // ----------------------------
    while (1)
    {
        // Read DHT22
        if (!dht22_read(
                &sensorData.temperature,
                &sensorData.humidity))
        {
            printf("DHT22 reading failed\n");

            sensorData.temperature = 0.0f;
            sensorData.humidity = 0.0f;
        }

        // Read LDR
        int raw_value = 0;

        result =
            adc_oneshot_read(
                adc_handle,
                LDR_ADC_CHANNEL,
                &raw_value
            );

        if (result == ESP_OK)
        {
            // Documented representation:
            // raw ADC value 0-4095
            // converted to percentage 0-100%
            sensorData.lightLevel =
                (int)(
                    (raw_value / 4095.0f)
                    * 100.0f
                );
        }
        else
        {
            sensorData.lightLevel = 0;
        }

        // Motion sensor not added yet
        sensorData.motionDetected = false;

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

        // Send to DisplayTask
        xQueueSend(
            displayQueue,
            &sensorData,
            pdMS_TO_TICKS(100)
        );

        // Send to AlarmTask
        xQueueSend(
            alarmQueue,
            &sensorData,
            pdMS_TO_TICKS(100)
        );

        printf(
            "Sensor data sent to queues\n"
        );

        // Periodic execution
        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}

// ----------------------------------------------------
// DisplayTask
// OLED is owned ONLY by DisplayTask
// ----------------------------------------------------
void displayTask(void *parameter)
{
    SensorData sensorData;

    oled_init();

    oled_clear();

    while (1)
    {
        if (xQueueReceive(
                displayQueue,
                &sensorData,
                portMAX_DELAY) == pdPASS)
        {
            oled_clear();

            // ----------------------------
            // TEMP 24 C
            // ----------------------------
            oled_char(0, 0, 'T');
            oled_char(6, 0, 'E');
            oled_char(12, 0, 'M');
            oled_char(18, 0, 'P');

            oled_number(
                36,
                0,
                (int)sensorData.temperature
            );

            oled_char(54, 0, 'C');

            // ----------------------------
            // HUM 40 %
            // ----------------------------
            oled_char(0, 2, 'H');
            oled_char(6, 2, 'U');
            oled_char(12, 2, 'M');

            oled_number(
                30,
                2,
                (int)sensorData.humidity
            );

            oled_char(48, 2, '%');

            // ----------------------------
            // LIGHT 24 %
            // ----------------------------
            oled_char(0, 4, 'L');
            oled_char(6, 4, 'I');
            oled_char(12, 4, 'G');
            oled_char(18, 4, 'H');
            oled_char(24, 4, 'T');

            oled_number(
                42,
                4,
                sensorData.lightLevel
            );

            oled_char(60, 4, '%');

            printf(
                "DisplayTask updated OLED: "
                "T=%.2f C, H=%.2f %%, "
                "Light=%d %%\n",
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

    // Configure DHT22
    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(DHT_PIN);

    // Create queues
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
        printf("Failed to create queues\n");
        return;
    }

    // Create SensorTask
    xTaskCreate(
        sensorTask,
        "SensorTask",
        6144,
        NULL,
        2,
        NULL
    );

    // Create DisplayTask
    xTaskCreate(
        displayTask,
        "DisplayTask",
        4096,
        NULL,
        1,
        NULL
    );

    // Create AlarmTask
    xTaskCreate(
        alarmTask,
        "AlarmTask",
        4096,
        NULL,
        1,
        NULL
    );
}