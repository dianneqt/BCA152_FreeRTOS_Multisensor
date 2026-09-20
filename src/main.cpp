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

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define DHT_PIN GPIO_NUM_4

// LDR AO -> GPIO34
#define LDR_ADC_CHANNEL ADC_CHANNEL_6

// OLED
#define OLED_SDA GPIO_NUM_21
#define OLED_SCL GPIO_NUM_22
#define OLED_ADDR 0x3C
#define I2C_PORT I2C_NUM_0

// Rotary Encoder
#define ENCODER_CLK GPIO_NUM_32
#define ENCODER_DT  GPIO_NUM_33


// ============================================================
// DISPLAY MODE
// ============================================================

enum class DisplayMode
{
    TEMPERATURE,
    HUMIDITY,
    LIGHT,
    MOTION
};


// ============================================================
// SENSOR DATA
// ============================================================

struct SensorData
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};


// ============================================================
// QUEUES
// ============================================================

QueueHandle_t displayQueue;
QueueHandle_t modeQueue;


// ============================================================
// OLED LOW-LEVEL FUNCTIONS
// ============================================================

static void oled_command(uint8_t command)
{
    uint8_t data[2];

    data[0] = 0x00;
    data[1] = command;

    i2c_master_write_to_device(
        I2C_PORT,
        OLED_ADDR,
        data,
        2,
        pdMS_TO_TICKS(100)
    );
}


static void oled_data(
    const uint8_t *data,
    size_t length)
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


// ============================================================
// OLED INITIALIZATION
// ============================================================

static void oled_init()
{
    i2c_config_t config = {};

    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = OLED_SDA;
    config.scl_io_num = OLED_SCL;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = 400000;

    i2c_param_config(
        I2C_PORT,
        &config
    );

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


// ============================================================
// OLED CLEAR
// ============================================================

static void oled_clear()
{
    uint8_t blank[128] = {0};

    for (int page = 0; page < 8; page++)
    {
        oled_command(0xB0 + page);
        oled_command(0x00);
        oled_command(0x10);

        oled_data(
            blank,
            128
        );
    }
}


// ============================================================
// OLED CURSOR
// ============================================================

static void oled_set_cursor(
    int x,
    int page)
{
    oled_command(
        0xB0 + page
    );

    oled_command(
        0x00 + (x & 0x0F)
    );

    oled_command(
        0x10 + ((x >> 4) & 0x0F)
    );
}


// ============================================================
// FONT
// ============================================================

static void oled_char(
    int x,
    int page,
    char c)
{
    uint8_t p[6] =
    {
        0, 0, 0, 0, 0, 0
    };

    switch (c)
    {
        case 'A':
            p[0]=0x7E; p[1]=0x11; p[2]=0x11;
            p[3]=0x11; p[4]=0x7E;
            break;

        case 'B':
            p[0]=0x7F; p[1]=0x49; p[2]=0x49;
            p[3]=0x49; p[4]=0x36;
            break;

        case 'C':
            p[0]=0x3E; p[1]=0x41; p[2]=0x41;
            p[3]=0x41; p[4]=0x22;
            break;

        case 'D':
            p[0]=0x7F; p[1]=0x41; p[2]=0x41;
            p[3]=0x22; p[4]=0x1C;
            break;

        case 'E':
            p[0]=0x7F; p[1]=0x49; p[2]=0x49;
            p[3]=0x49; p[4]=0x41;
            break;

        case 'F':
            p[0]=0x7F; p[1]=0x09; p[2]=0x09;
            p[3]=0x09; p[4]=0x01;
            break;

        case 'G':
            p[0]=0x3E; p[1]=0x41; p[2]=0x49;
            p[3]=0x49; p[4]=0x7A;
            break;

        case 'H':
            p[0]=0x7F; p[1]=0x08; p[2]=0x08;
            p[3]=0x08; p[4]=0x7F;
            break;

        case 'I':
            p[0]=0x00; p[1]=0x41; p[2]=0x7F;
            p[3]=0x41; p[4]=0x00;
            break;

        case 'J':
            p[0]=0x20; p[1]=0x40; p[2]=0x41;
            p[3]=0x3F; p[4]=0x01;
            break;

        case 'K':
            p[0]=0x7F; p[1]=0x08; p[2]=0x14;
            p[3]=0x22; p[4]=0x41;
            break;

        case 'L':
            p[0]=0x7F; p[1]=0x40; p[2]=0x40;
            p[3]=0x40; p[4]=0x40;
            break;

        case 'M':
            p[0]=0x7F; p[1]=0x02; p[2]=0x0C;
            p[3]=0x02; p[4]=0x7F;
            break;

        case 'N':
            p[0]=0x7F; p[1]=0x02; p[2]=0x0C;
            p[3]=0x18; p[4]=0x7F;
            break;

        case 'O':
            p[0]=0x3E; p[1]=0x41; p[2]=0x41;
            p[3]=0x41; p[4]=0x3E;
            break;

        case 'P':
            p[0]=0x7F; p[1]=0x09; p[2]=0x09;
            p[3]=0x09; p[4]=0x06;
            break;

        case 'Q':
            p[0]=0x3E; p[1]=0x41; p[2]=0x51;
            p[3]=0x21; p[4]=0x5E;
            break;

        case 'R':
            p[0]=0x7F; p[1]=0x09; p[2]=0x19;
            p[3]=0x29; p[4]=0x46;
            break;

        case 'S':
            p[0]=0x46; p[1]=0x49; p[2]=0x49;
            p[3]=0x49; p[4]=0x31;
            break;

        case 'T':
            p[0]=0x01; p[1]=0x01; p[2]=0x7F;
            p[3]=0x01; p[4]=0x01;
            break;

        case 'U':
            p[0]=0x3F; p[1]=0x40; p[2]=0x40;
            p[3]=0x40; p[4]=0x3F;
            break;

        case 'V':
            p[0]=0x1F; p[1]=0x20; p[2]=0x40;
            p[3]=0x20; p[4]=0x1F;
            break;

        case 'W':
            p[0]=0x7F; p[1]=0x20; p[2]=0x18;
            p[3]=0x20; p[4]=0x7F;
            break;

        case 'X':
            p[0]=0x63; p[1]=0x14; p[2]=0x08;
            p[3]=0x14; p[4]=0x63;
            break;

        case 'Y':
            p[0]=0x07; p[1]=0x08; p[2]=0x70;
            p[3]=0x08; p[4]=0x07;
            break;

        case 'Z':
            p[0]=0x61; p[1]=0x51; p[2]=0x49;
            p[3]=0x45; p[4]=0x43;
            break;

        case '0':
            p[0]=0x3E; p[1]=0x51; p[2]=0x49;
            p[3]=0x45; p[4]=0x3E;
            break;

        case '1':
            p[0]=0x00; p[1]=0x42; p[2]=0x7F;
            p[3]=0x40; p[4]=0x00;
            break;

        case '2':
            p[0]=0x42; p[1]=0x61; p[2]=0x51;
            p[3]=0x49; p[4]=0x46;
            break;

        case '3':
            p[0]=0x21; p[1]=0x41; p[2]=0x45;
            p[3]=0x4B; p[4]=0x31;
            break;

        case '4':
            p[0]=0x18; p[1]=0x14; p[2]=0x12;
            p[3]=0x7F; p[4]=0x10;
            break;

        case '5':
            p[0]=0x27; p[1]=0x45; p[2]=0x45;
            p[3]=0x45; p[4]=0x39;
            break;

        case '6':
            p[0]=0x3C; p[1]=0x4A; p[2]=0x49;
            p[3]=0x49; p[4]=0x30;
            break;

        case '7':
            p[0]=0x01; p[1]=0x71; p[2]=0x09;
            p[3]=0x05; p[4]=0x03;
            break;

        case '8':
            p[0]=0x36; p[1]=0x49; p[2]=0x49;
            p[3]=0x49; p[4]=0x36;
            break;

        case '9':
            p[0]=0x06; p[1]=0x49; p[2]=0x49;
            p[3]=0x29; p[4]=0x1E;
            break;

        case '%':
            p[0]=0x63; p[1]=0x13; p[2]=0x08;
            p[3]=0x64; p[4]=0x63;
            break;

        case '.':
            p[0]=0x00; p[1]=0x60; p[2]=0x60;
            p[3]=0x00; p[4]=0x00;
            break;

        default:
            break;
    }

    oled_set_cursor(x, page);
    oled_data(p, 6);
}


// ============================================================
// OLED TEXT
// ============================================================

static void oled_text(
    int x,
    int page,
    const char *text)
{
    while (*text)
    {
        oled_char(
            x,
            page,
            *text
        );

        x += 6;
        text++;
    }
}


// ============================================================
// OLED NUMBER
// ============================================================

static void oled_number(
    int x,
    int page,
    int number)
{
    if (number < 0)
        number = -number;

    if (number >= 1000)
    {
        oled_char(
            x,
            page,
            '0' + number / 1000
        );

        x += 6;
        number %= 1000;
    }

    if (number >= 100)
    {
        oled_char(
            x,
            page,
            '0' + number / 100
        );

        x += 6;
        number %= 100;
    }

    if (number >= 10)
    {
        oled_char(
            x,
            page,
            '0' + number / 10
        );

        x += 6;
        number %= 10;
    }

    oled_char(
        x,
        page,
        '0' + number
    );
}


// ============================================================
// OLED TEMPERATURE
// ============================================================

static void oled_temperature(
    int x,
    int page,
    float temperature)
{
    int whole =
        (int)temperature;

    if (whole < 0)
        whole = -whole;

    int decimal =
        (int)(
            (temperature -
             (int)temperature) *
            10.0f
        );

    if (decimal < 0)
        decimal = -decimal;

    oled_number(
        x,
        page,
        whole
    );

    if (whole < 10)
        x += 6;
    else if (whole < 100)
        x += 12;
    else
        x += 18;

    oled_char(
        x,
        page,
        '.'
    );

    oled_char(
        x + 6,
        page,
        '0' + decimal
    );
}


// ============================================================
// DHT22 READING
// ============================================================

static bool dht22_read(
    float *temperature,
    float *humidity)
{
    uint8_t data[5] =
    {
        0, 0, 0, 0, 0
    };

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_OUTPUT
    );

    gpio_set_level(
        DHT_PIN,
        0
    );

    esp_rom_delay_us(1200);

    gpio_set_level(
        DHT_PIN,
        1
    );

    esp_rom_delay_us(30);

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(
        DHT_PIN
    );

    int64_t start =
        esp_timer_get_time();

    while (
        gpio_get_level(DHT_PIN) == 1)
    {
        if (
            esp_timer_get_time() - start
            > 100)
        {
            return false;
        }
    }

    start =
        esp_timer_get_time();

    while (
        gpio_get_level(DHT_PIN) == 0)
    {
        if (
            esp_timer_get_time() - start
            > 100)
        {
            return false;
        }
    }

    start =
        esp_timer_get_time();

    while (
        gpio_get_level(DHT_PIN) == 1)
    {
        if (
            esp_timer_get_time() - start
            > 100)
        {
            return false;
        }
    }

    for (int i = 0; i < 40; i++)
    {
        start =
            esp_timer_get_time();

        while (
            gpio_get_level(DHT_PIN) == 0)
        {
            if (
                esp_timer_get_time() - start
                > 100)
            {
                return false;
            }
        }

        int64_t high_start =
            esp_timer_get_time();

        while (
            gpio_get_level(DHT_PIN) == 1)
        {
            if (
                esp_timer_get_time() - high_start
                > 100)
            {
                return false;
            }
        }

        int64_t pulse_length =
            esp_timer_get_time()
            - high_start;

        int byte_index =
            i / 8;

        int bit_index =
            7 - (i % 8);

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
        return false;

    *humidity =
        ((data[0] << 8) | data[1])
        / 10.0f;

    int16_t raw_temperature =
        (data[2] << 8) | data[3];

    if (raw_temperature & 0x8000)
    {
        raw_temperature &=
            0x7FFF;

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


// ============================================================
// SENSOR TASK
// ============================================================

void sensorTask(void *parameter)
{
    SensorData sensorData =
    {
        24.0f,
        40.0f,
        0,
        false
    };

    TickType_t lastWakeTime =
        xTaskGetTickCount();

    adc_oneshot_unit_handle_t adc_handle =
        NULL;

    adc_oneshot_unit_init_cfg_t init_config = {};

    init_config.unit_id =
        ADC_UNIT_1;

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
        printf(
            "LDR ADC initialization failed\n"
        );

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

    while (1)
    {
        // DHT22

        float newTemperature;
        float newHumidity;

        if (
            dht22_read(
                &newTemperature,
                &newHumidity))
        {
            sensorData.temperature =
                newTemperature;

            sensorData.humidity =
                newHumidity;

            printf(
                "Temperature: %.2f C\n",
                sensorData.temperature
            );

            printf(
                "Humidity: %.2f %%\n",
                sensorData.humidity
            );
        }
        else
        {
            printf(
                "DHT22 reading failed\n"
            );
        }

        // LDR

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
                (int)(
                    (raw_value / 4095.0f)
                    * 100.0f
                );
        }

        printf(
            "Light Level: %d %%\n",
            sensorData.lightLevel
        );

        // Motion not yet installed

        sensorData.motionDetected =
            false;

        // Send data

        xQueueSend(
            displayQueue,
            &sensorData,
            pdMS_TO_TICKS(100)
        );

        printf(
            "Sensor data sent to queue\n"
        );

        // Required periodic timing

        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}


// ============================================================
// INPUT TASK
// ============================================================

void inputTask(void *parameter)
{
    gpio_set_direction(
        ENCODER_CLK,
        GPIO_MODE_INPUT
    );

    gpio_set_direction(
        ENCODER_DT,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(
        ENCODER_CLK
    );

    gpio_pullup_en(
        ENCODER_DT
    );

    DisplayMode currentMode =
        DisplayMode::TEMPERATURE;

    int lastCLK =
        gpio_get_level(
            ENCODER_CLK
        );

    while (1)
    {
        int currentCLK =
            gpio_get_level(
                ENCODER_CLK
            );

        if (
            lastCLK == 1 &&
            currentCLK == 0)
        {
            int currentDT =
                gpio_get_level(
                    ENCODER_DT
                );

            /*
             * IMPORTANT:
             *
             * This condition is reversed from
             * the previous version to match
             * the Wokwi encoder wiring.
             *
             * Clockwise:
             *
             * Temperature
             *      ↓
             * Humidity
             *      ↓
             * Light
             *      ↓
             * Motion
             *      ↓
             * Temperature
             */

            if (currentDT == currentCLK)
            {
                // CLOCKWISE

                switch (currentMode)
                {
                    case DisplayMode::TEMPERATURE:
                        currentMode =
                            DisplayMode::HUMIDITY;
                        break;

                    case DisplayMode::HUMIDITY:
                        currentMode =
                            DisplayMode::LIGHT;
                        break;

                    case DisplayMode::LIGHT:
                        currentMode =
                            DisplayMode::MOTION;
                        break;

                    case DisplayMode::MOTION:
                        currentMode =
                            DisplayMode::TEMPERATURE;
                        break;
                }

                printf(
                    "InputTask: clockwise\n"
                );
            }
            else
            {
                // COUNTERCLOCKWISE

                switch (currentMode)
                {
                    case DisplayMode::TEMPERATURE:
                        currentMode =
                            DisplayMode::MOTION;
                        break;

                    case DisplayMode::MOTION:
                        currentMode =
                            DisplayMode::LIGHT;
                        break;

                    case DisplayMode::LIGHT:
                        currentMode =
                            DisplayMode::HUMIDITY;
                        break;

                    case DisplayMode::HUMIDITY:
                        currentMode =
                            DisplayMode::TEMPERATURE;
                        break;
                }

                printf(
                    "InputTask: counterclockwise\n"
                );
            }

            xQueueOverwrite(
                modeQueue,
                &currentMode
            );

            vTaskDelay(
                pdMS_TO_TICKS(20)
            );
        }

        lastCLK =
            currentCLK;

        vTaskDelay(
            pdMS_TO_TICKS(5)
        );
    }
}


// ============================================================
// DRAW DISPLAY
// ============================================================

static void drawDisplay(
    DisplayMode mode,
    const SensorData &data)
{
    oled_clear();

    if (
        mode ==
        DisplayMode::TEMPERATURE)
    {
        oled_text(
            0,
            0,
            "ROOM MONITOR"
        );

        oled_text(
            0,
            2,
            "TEMPERATURE"
        );

        oled_temperature(
            0,
            4,
            data.temperature
        );

        oled_char(
            36,
            4,
            'C'
        );

        printf(
            "DisplayTask: Temperature page = %.2f C\n",
            data.temperature
        );
    }

    else if (
        mode ==
        DisplayMode::HUMIDITY)
    {
        oled_text(
            0,
            0,
            "ROOM MONITOR"
        );

        oled_text(
            0,
            2,
            "HUMIDITY"
        );

        oled_number(
            0,
            4,
            (int)data.humidity
        );

        oled_char(
            24,
            4,
            '%'
        );

        printf(
            "DisplayTask: Humidity page = %.2f %%\n",
            data.humidity
        );
    }

    else if (
        mode ==
        DisplayMode::LIGHT)
    {
        oled_text(
            0,
            0,
            "ROOM MONITOR"
        );

        oled_text(
            0,
            2,
            "LIGHT"
        );

        oled_number(
            0,
            4,
            data.lightLevel
        );

        oled_char(
            24,
            4,
            '%'
        );

        printf(
            "DisplayTask: Light page = %d %%\n",
            data.lightLevel
        );
    }

    else if (
        mode ==
        DisplayMode::MOTION)
    {
        oled_text(
            0,
            0,
            "ROOM MONITOR"
        );

        oled_text(
            0,
            2,
            "MOTION"
        );

        if (data.motionDetected)
        {
            oled_text(
                0,
                4,
                "DETECTED"
            );
        }
        else
        {
            oled_text(
                0,
                4,
                "NONE"
            );
        }

        printf(
            "DisplayTask: Motion page\n"
        );
    }
}


// ============================================================
// DISPLAY TASK
// ============================================================

void displayTask(void *parameter)
{
    SensorData sensorData =
    {
        24.0f,
        40.0f,
        0,
        false
    };

    DisplayMode currentMode =
        DisplayMode::TEMPERATURE;

    oled_init();

    // Show Temperature immediately

    drawDisplay(
        currentMode,
        sensorData
    );

    while (1)
    {
        bool displayChanged =
            false;

        // Check encoder mode

        DisplayMode newMode;

        if (
            xQueueReceive(
                modeQueue,
                &newMode,
                0
            ) == pdPASS)
        {
            currentMode =
                newMode;

            displayChanged =
                true;
        }

        // Check sensor data

        SensorData newSensorData;

        if (
            xQueueReceive(
                displayQueue,
                &newSensorData,
                0
            ) == pdPASS)
        {
            sensorData =
                newSensorData;

            displayChanged =
                true;
        }

        if (displayChanged)
        {
            drawDisplay(
                currentMode,
                sensorData
            );
        }

        vTaskDelay(
            pdMS_TO_TICKS(20)
        );
    }
}


// ============================================================
// MAIN
// ============================================================

extern "C" void app_main(void)
{
    printf("\n");
    printf(
        "BCA152 FreeRTOS Multisensor\n"
    );

    printf(
        "System starting...\n"
    );

    // DHT22

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(
        DHT_PIN
    );

    // Create queues

    displayQueue =
        xQueueCreate(
            5,
            sizeof(SensorData)
        );

    modeQueue =
        xQueueCreate(
            1,
            sizeof(DisplayMode)
        );

    if (
        displayQueue == NULL ||
        modeQueue == NULL)
    {
        printf(
            "ERROR: Queue creation failed\n"
        );

        return;
    }

    // SensorTask

    xTaskCreate(
        sensorTask,
        "SensorTask",
        6144,
        NULL,
        2,
        NULL
    );

    // InputTask

    xTaskCreate(
        inputTask,
        "InputTask",
        4096,
        NULL,
        2,
        NULL
    );

    // DisplayTask

    xTaskCreate(
        displayTask,
        "DisplayTask",
        4096,
        NULL,
        1,
        NULL
    );
}