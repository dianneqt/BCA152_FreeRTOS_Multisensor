#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "esp_err.h"

// ====================================================
// PIN DEFINITIONS
// ====================================================

#define DHT_PIN GPIO_NUM_4

#define LDR_ADC_CHANNEL ADC_CHANNEL_6   // GPIO34

#define OLED_SDA GPIO_NUM_21
#define OLED_SCL GPIO_NUM_22
#define OLED_ADDR 0x3C
#define I2C_PORT I2C_NUM_0

// Rotary Encoder
#define ENCODER_CLK GPIO_NUM_32
#define ENCODER_DT  GPIO_NUM_33
#define ENCODER_SW  GPIO_NUM_25

// PIR motion sensor
#define PIR_PIN GPIO_NUM_27

// ====================================================
// DHT22 SETTINGS
// ====================================================

#define DHT_TIMEOUT_US 150
#define DHT_BIT_TIMEOUT_US 120
#define DHT_ONE_THRESHOLD_US 50

// ====================================================
// TIMING SETTINGS
// ====================================================

// Laboratory testing value (step 31)
#define INACTIVITY_TIMEOUT_US 15000000LL

// ====================================================
// ENCODER SETTINGS
// ====================================================

// If clockwise ever moves BACKWARDS through the pages,
// change this to true.
constexpr bool ENCODER_REVERSE = false;

// ====================================================
// ALARM THRESHOLDS
// ====================================================
// Adjust these to match your lab specification.

constexpr float LOW_TEMPERATURE_THRESHOLD_C  = 18.0f;
constexpr float HIGH_TEMPERATURE_THRESHOLD_C = 30.0f;

// ====================================================
// ENUMERATIONS
// ====================================================

enum class DisplayMode
{
    TEMPERATURE,
    HUMIDITY,
    LIGHT,
    MOTION
};

// Step 32: system state machine
enum class SystemState
{
    ACTIVE,
    INACTIVE
};

// Step 30: alarm decision result
enum class AlarmState
{
    NORMAL,
    LOW_TEMPERATURE,
    HIGH_TEMPERATURE
};

// ====================================================
// SENSOR DATA
// ====================================================

struct SensorData
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
};

// ====================================================
// PURE ALARM LOGIC (no hardware, unit-testable)
// ====================================================
//
// Tip: move this function and the AlarmState enum into
// include/alarm_logic.h and src/alarm_logic.cpp (or lib/)
// so that test/ can include them without any ESP32 code.

AlarmState evaluateTemperature(float temperature)
{
    if (temperature < LOW_TEMPERATURE_THRESHOLD_C)
        return AlarmState::LOW_TEMPERATURE;

    if (temperature > HIGH_TEMPERATURE_THRESHOLD_C)
        return AlarmState::HIGH_TEMPERATURE;

    return AlarmState::NORMAL;
}

static const char *alarmStateToString(AlarmState state)
{
    switch (state)
    {
        case AlarmState::NORMAL:           return "NORMAL";
        case AlarmState::LOW_TEMPERATURE:  return "LOW_TEMPERATURE";
        case AlarmState::HIGH_TEMPERATURE: return "HIGH_TEMPERATURE";
    }

    return "UNKNOWN";
}

// ====================================================
// DISPLAY MODE HELPERS
// ====================================================

static DisplayMode nextMode(DisplayMode mode)
{
    switch (mode)
    {
        case DisplayMode::TEMPERATURE: return DisplayMode::HUMIDITY;
        case DisplayMode::HUMIDITY:    return DisplayMode::LIGHT;
        case DisplayMode::LIGHT:       return DisplayMode::MOTION;
        case DisplayMode::MOTION:      return DisplayMode::TEMPERATURE;
    }

    return DisplayMode::TEMPERATURE;
}

static DisplayMode previousMode(DisplayMode mode)
{
    switch (mode)
    {
        case DisplayMode::TEMPERATURE: return DisplayMode::MOTION;
        case DisplayMode::HUMIDITY:    return DisplayMode::TEMPERATURE;
        case DisplayMode::LIGHT:       return DisplayMode::HUMIDITY;
        case DisplayMode::MOTION:      return DisplayMode::LIGHT;
    }

    return DisplayMode::TEMPERATURE;
}

static const char *modeToString(DisplayMode mode)
{
    switch (mode)
    {
        case DisplayMode::TEMPERATURE: return "TEMPERATURE";
        case DisplayMode::HUMIDITY:    return "HUMIDITY";
        case DisplayMode::LIGHT:       return "LIGHT";
        case DisplayMode::MOTION:      return "MOTION";
    }

    return "UNKNOWN";
}

// ====================================================
// QUEUES
// ====================================================

QueueHandle_t displayQueue;
QueueHandle_t alarmQueue;
QueueHandle_t modeQueue;
QueueHandle_t encoderQueue;   // +1 = clockwise, -1 = counter-clockwise

// ====================================================
// SYSTEM STATE
// ====================================================

// ----------------------------------------------------
// SYSTEM EVENTS (step 35): one event group
// ----------------------------------------------------
//
// Each bit has exactly ONE producer (the only task that sets or
// clears it), which avoids write conflicts and removes the need
// for unsynchronized global variables.
//
// EVENT_ACTIVE  (BIT0)  system is in the ACTIVE state
//   Producer : MotionTask
//   Consumers: InputTask, DisplayTask, AlarmTask
//   Set      : at boot, and when PIR activity restores ACTIVE
//   Cleared  : when 15 s pass without PIR activity (INACTIVE)
//
// EVENT_MOTION  (BIT1)  motion detected (stays set while ACTIVE)
//   Producer : MotionTask
//   Consumers: SensorTask (copies it into SensorData),
//              DisplayTask (MOTION page)
//   Set      : on the PIR rising edge
//   Cleared  : when the 15 s inactivity timeout expires
//              (at the same moment EVENT_ACTIVE is cleared)
//
// EVENT_ALARM   (BIT2)  temperature alarm is active
//   Producer : AlarmTask
//   Consumers: DisplayTask (shows "ALARM" on the OLED)
//   Set      : evaluateTemperature() returns LOW or HIGH_TEMPERATURE
//   Cleared  : temperature returns to NORMAL, or system goes INACTIVE
//
// The event group holds STATE. To wake DisplayTask when something
// changes, task notifications are used (notifyDisplay()):
//   Producers: SensorTask, MotionTask, InputTask, AlarmTask
//   Consumer : DisplayTask   (wakes and redraws)
// and the PIR interrupt wakes MotionTask (vTaskNotifyGiveFromISR).
//
// ----------------------------------------------------

#define EVENT_ACTIVE (1U << 0)
#define EVENT_MOTION (1U << 1)
#define EVENT_ALARM  (1U << 2)

EventGroupHandle_t systemEvents = NULL;

static inline bool eventIsSet(EventBits_t bit)
{
    return (xEventGroupGetBits(systemEvents) & bit) != 0;
}

// Step 32 state machine, derived from EVENT_ACTIVE
static inline SystemState getSystemState()
{
    return eventIsSet(EVENT_ACTIVE)
        ? SystemState::ACTIVE
        : SystemState::INACTIVE;
}

static inline bool isMotionDetected()
{
    return eventIsSet(EVENT_MOTION);
}

static inline bool isAlarmActive()
{
    return eventIsSet(EVENT_ALARM);
}

// ====================================================
// HELPERS
// ====================================================

// pdMS_TO_TICKS() rounds DOWN. With a 100 Hz tick (10 ms per tick),
// pdMS_TO_TICKS(5) == 0, and vTaskDelay(0) does NOT block, which
// starves lower-priority tasks and triggers the task watchdog.
// This helper guarantees at least one tick so a task always blocks.
static inline TickType_t ms_to_ticks(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);

    return (ticks == 0) ? 1 : ticks;
}

// ----------------------------------------------------
// SHARED RESOURCE PROTECTION (step 36): serial output
// ----------------------------------------------------
//
// Resource : the serial console (UART0, used by printf)
// Users    : every task (Sensor, Motion, Input, Display, Alarm)
// Problem  : two tasks printing at the same time can interleave
//            their characters, mixing two messages into one line
// Solution : serialMutex. A task must hold it while printing.
//
// Use logPrint() instead of logPrint() everywhere.
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
// priority inheritance: a low-priority task holding the mutex is
// temporarily boosted so a high-priority task waiting for it is
// not blocked indefinitely by medium-priority tasks.
//
// ----------------------------------------------------

SemaphoreHandle_t serialMutex = NULL;

static void logPrint(const char *format, ...)
    __attribute__((format(printf, 1, 2)));

static void logPrint(const char *format, ...)
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

// Handle of DisplayTask so other tasks can wake it instantly.
TaskHandle_t displayTaskHandle = NULL;

// Wakes DisplayTask immediately (task notification) instead of
// waiting for its timeout. Safe to call from any task.
static void notifyDisplay()
{
    if (displayTaskHandle != NULL)
        xTaskNotifyGive(displayTaskHandle);
}

// Handle of MotionTask, set by MotionTask itself when it starts.
TaskHandle_t motionTaskHandle = NULL;

// PIR interrupt: fires on BOTH edges of the PIR output and wakes
// MotionTask immediately, so it does not have to poll.
static void IRAM_ATTR pirIsr(void *arg)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (motionTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(
            motionTaskHandle,
            &higherPriorityTaskWoken
        );
    }

    if (higherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

// ====================================================
// OLED
// ====================================================

static bool i2cErrorLogged = false;

static void report_i2c_error(esp_err_t err)
{
    // Log only the first failure so the serial monitor is not flooded.
    if (err != ESP_OK && !i2cErrorLogged)
    {
        i2cErrorLogged = true;

        logPrint(
            "OLED I2C error: %s "
            "(check SDA=21, SCL=22, address 0x3C)\n",
            esp_err_to_name(err)
        );
    }
}

static void oled_command(uint8_t command)
{
    uint8_t data[2] = {0x00, command};

    esp_err_t err =
        i2c_master_write_to_device(
            I2C_PORT,
            OLED_ADDR,
            data,
            sizeof(data),
            pdMS_TO_TICKS(100)
        );

    report_i2c_error(err);
}

static void oled_data(const uint8_t *data, size_t length)
{
    uint8_t buffer[129];

    if (length > 128)
        return;

    buffer[0] = 0x40;

    for (size_t i = 0; i < length; i++)
        buffer[i + 1] = data[i];

    esp_err_t err =
        i2c_master_write_to_device(
            I2C_PORT,
            OLED_ADDR,
            buffer,
            length + 1,
            pdMS_TO_TICKS(100)
        );

    report_i2c_error(err);
}

static void oled_init()
{
    i2c_config_t config = {};

    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = OLED_SDA;
    config.scl_io_num = OLED_SCL;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = 400000;

    esp_err_t err = i2c_param_config(I2C_PORT, &config);

    if (err != ESP_OK)
    {
        logPrint("i2c_param_config failed: %s\n", esp_err_to_name(err));
    }

    err = i2c_driver_install(
        I2C_PORT,
        config.mode,
        0,
        0,
        0
    );

    if (err != ESP_OK)
    {
        logPrint("i2c_driver_install failed: %s\n", esp_err_to_name(err));
    }

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

// Display power control: 0xAF = display ON, 0xAE = display OFF (sleep)
static void oled_power(bool on)
{
    oled_command(on ? 0xAF : 0xAE);
}

// Frame buffer: 128 columns x 8 pages. All drawing happens in RAM
// (instant); oled_flush() then sends the whole screen in a handful
// of I2C transactions instead of hundreds of tiny ones.
static uint8_t frameBuffer[128 * 8];

// Clears the RAM buffer only (call oled_flush() to show it)
static void oled_clear()
{
    for (int i = 0; i < 128 * 8; i++)
        frameBuffer[i] = 0;
}

// Sends the frame buffer to the display.
// Horizontal addressing mode is selected in oled_init() (0x20, 0x00),
// so after setting the window once, data flows page after page.
static void oled_flush()
{
    oled_command(0x21);   // column range
    oled_command(0x00);
    oled_command(0x7F);

    oled_command(0x22);   // page range
    oled_command(0x00);
    oled_command(0x07);

    for (int page = 0; page < 8; page++)
        oled_data(&frameBuffer[page * 128], 128);
}

// ====================================================
// OLED CHARACTER FONT (5x7 glyphs, 6 columns with spacing)
// ====================================================

static void get_glyph(char c, uint8_t p[6])
{
    for (int i = 0; i < 6; i++)
        p[i] = 0;

    switch (c)
    {
        case 'A': p[0]=0x7E; p[1]=0x11; p[2]=0x11; p[3]=0x11; p[4]=0x7E; break;
        case 'B': p[0]=0x7F; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x36; break;
        case 'C': p[0]=0x3E; p[1]=0x41; p[2]=0x41; p[3]=0x41; p[4]=0x22; break;
        case 'D': p[0]=0x7F; p[1]=0x41; p[2]=0x41; p[3]=0x22; p[4]=0x1C; break;
        case 'E': p[0]=0x7F; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x41; break;
        case 'F': p[0]=0x7F; p[1]=0x09; p[2]=0x09; p[3]=0x09; p[4]=0x01; break;
        case 'G': p[0]=0x3E; p[1]=0x41; p[2]=0x49; p[3]=0x49; p[4]=0x7A; break;
        case 'H': p[0]=0x7F; p[1]=0x08; p[2]=0x08; p[3]=0x08; p[4]=0x7F; break;
        case 'I': p[0]=0x00; p[1]=0x41; p[2]=0x7F; p[3]=0x41; p[4]=0x00; break;
        case 'J': p[0]=0x20; p[1]=0x40; p[2]=0x41; p[3]=0x3F; p[4]=0x01; break;
        case 'K': p[0]=0x7F; p[1]=0x08; p[2]=0x14; p[3]=0x22; p[4]=0x41; break;
        case 'L': p[0]=0x7F; p[1]=0x40; p[2]=0x40; p[3]=0x40; p[4]=0x40; break;
        case 'M': p[0]=0x7F; p[1]=0x02; p[2]=0x0C; p[3]=0x02; p[4]=0x7F; break;
        case 'N': p[0]=0x7F; p[1]=0x02; p[2]=0x0C; p[3]=0x18; p[4]=0x7F; break;
        case 'O': p[0]=0x3E; p[1]=0x41; p[2]=0x41; p[3]=0x41; p[4]=0x3E; break;
        case 'P': p[0]=0x7F; p[1]=0x09; p[2]=0x09; p[3]=0x09; p[4]=0x06; break;
        case 'Q': p[0]=0x3E; p[1]=0x41; p[2]=0x51; p[3]=0x21; p[4]=0x5E; break;
        case 'R': p[0]=0x7F; p[1]=0x09; p[2]=0x19; p[3]=0x29; p[4]=0x46; break;
        case 'S': p[0]=0x46; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x31; break;
        case 'T': p[0]=0x01; p[1]=0x01; p[2]=0x7F; p[3]=0x01; p[4]=0x01; break;
        case 'U': p[0]=0x3F; p[1]=0x40; p[2]=0x40; p[3]=0x40; p[4]=0x3F; break;
        case 'V': p[0]=0x1F; p[1]=0x20; p[2]=0x40; p[3]=0x20; p[4]=0x1F; break;
        case 'W': p[0]=0x7F; p[1]=0x20; p[2]=0x18; p[3]=0x20; p[4]=0x7F; break;
        case 'X': p[0]=0x63; p[1]=0x14; p[2]=0x08; p[3]=0x14; p[4]=0x63; break;
        case 'Y': p[0]=0x07; p[1]=0x08; p[2]=0x70; p[3]=0x08; p[4]=0x07; break;
        case 'Z': p[0]=0x61; p[1]=0x51; p[2]=0x49; p[3]=0x45; p[4]=0x43; break;

        case '0': p[0]=0x3E; p[1]=0x51; p[2]=0x49; p[3]=0x45; p[4]=0x3E; break;
        case '1': p[0]=0x00; p[1]=0x42; p[2]=0x7F; p[3]=0x40; p[4]=0x00; break;
        case '2': p[0]=0x42; p[1]=0x61; p[2]=0x51; p[3]=0x49; p[4]=0x46; break;
        case '3': p[0]=0x21; p[1]=0x41; p[2]=0x45; p[3]=0x4B; p[4]=0x31; break;
        case '4': p[0]=0x18; p[1]=0x14; p[2]=0x12; p[3]=0x7F; p[4]=0x10; break;
        case '5': p[0]=0x27; p[1]=0x45; p[2]=0x45; p[3]=0x45; p[4]=0x39; break;
        case '6': p[0]=0x3C; p[1]=0x4A; p[2]=0x49; p[3]=0x49; p[4]=0x30; break;
        case '7': p[0]=0x01; p[1]=0x71; p[2]=0x09; p[3]=0x05; p[4]=0x03; break;
        case '8': p[0]=0x36; p[1]=0x49; p[2]=0x49; p[3]=0x49; p[4]=0x36; break;
        case '9': p[0]=0x06; p[1]=0x49; p[2]=0x49; p[3]=0x29; p[4]=0x1E; break;

        case '%': p[0]=0x63; p[1]=0x13; p[2]=0x08; p[3]=0x64; p[4]=0x63; break;
        case '.': p[0]=0x00; p[1]=0x60; p[2]=0x60; p[3]=0x00; p[4]=0x00; break;
        case '-': p[0]=0x08; p[1]=0x08; p[2]=0x08; p[3]=0x08; p[4]=0x08; break;

        default:
            break;
    }
}

// Normal size: 6 px wide, 1 page (8 px) tall
static void oled_char(int x, int page, char c)
{
    uint8_t p[6];

    get_glyph(c, p);

    if (page < 0 || page > 7 || x < 0 || x + 6 > 128)
        return;

    for (int i = 0; i < 6; i++)
        frameBuffer[page * 128 + x + i] = p[i];
}

static void oled_text(int x, int page, const char *text)
{
    while (*text)
    {
        oled_char(x, page, *text);
        x += 6;
        text++;
    }
}

// Double size: 12 px wide, 2 pages (16 px) tall.
// Each glyph column is stretched vertically (bit doubling)
// and repeated horizontally.
static void oled_char_big(int x, int page, char c)
{
    uint8_t g[6];
    uint8_t top[12];
    uint8_t bottom[12];

    get_glyph(c, g);

    for (int i = 0; i < 6; i++)
    {
        uint8_t low  = 0;
        uint8_t high = 0;

        for (int b = 0; b < 4; b++)
        {
            if (g[i] & (1 << b))
                low |= (uint8_t)(3 << (2 * b));

            if (g[i] & (1 << (b + 4)))
                high |= (uint8_t)(3 << (2 * b));
        }

        top[2 * i]        = low;
        top[2 * i + 1]    = low;
        bottom[2 * i]     = high;
        bottom[2 * i + 1] = high;
    }

    if (page < 0 || page > 6 || x < 0 || x + 12 > 128)
        return;

    for (int i = 0; i < 12; i++)
    {
        frameBuffer[page * 128 + x + i]       = top[i];
        frameBuffer[(page + 1) * 128 + x + i] = bottom[i];
    }
}

static void oled_text_big(int x, int page, const char *text)
{
    while (*text)
    {
        oled_char_big(x, page, *text);
        x += 12;
        text++;
    }
}

// ====================================================
// DHT22
// ====================================================

static portMUX_TYPE dhtMux =
    portMUX_INITIALIZER_UNLOCKED;

static bool dht22_read(
    float *temperature,
    float *humidity
)
{
    uint8_t data[5] = {0, 0, 0, 0, 0};

    bool success = false;

    // Error text is stored here and printed AFTER the critical
    // section ends (a mutex must not be taken inside one).
    char errorMessage[64];
    errorMessage[0] = '\0';

    portENTER_CRITICAL(&dhtMux);

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

    gpio_pullup_en(DHT_PIN);

    // Sensor response LOW
    int64_t start =
        esp_timer_get_time();

    while (gpio_get_level(DHT_PIN) == 1)
    {
        if (
            (esp_timer_get_time() - start)
            > DHT_TIMEOUT_US
        )
        {
            break;
        }
    }

    bool responseLow =
        (gpio_get_level(DHT_PIN) == 0);

    // Sensor response HIGH
    if (responseLow)
    {
        start =
            esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 0)
        {
            if (
                (esp_timer_get_time() - start)
                > DHT_TIMEOUT_US
            )
            {
                break;
            }
        }
    }

    bool responseHigh =
        (gpio_get_level(DHT_PIN) == 1);

    if (responseLow && responseHigh)
    {
        start =
            esp_timer_get_time();

        while (gpio_get_level(DHT_PIN) == 1)
        {
            if (
                (esp_timer_get_time() - start)
                > DHT_TIMEOUT_US
            )
            {
                break;
            }
        }

        bool firstBitStarted =
            (gpio_get_level(DHT_PIN) == 0);

        if (firstBitStarted)
        {
            bool readOK = true;

            for (int i = 0; i < 40; i++)
            {
                start =
                    esp_timer_get_time();

                while (gpio_get_level(DHT_PIN) == 0)
                {
                    if (
                        (esp_timer_get_time() - start)
                        > DHT_BIT_TIMEOUT_US
                    )
                    {
                        readOK = false;
                        break;
                    }
                }

                if (!readOK)
                    break;

                int64_t highStart =
                    esp_timer_get_time();

                while (gpio_get_level(DHT_PIN) == 1)
                {
                    if (
                        (esp_timer_get_time() - highStart)
                        > DHT_BIT_TIMEOUT_US
                    )
                    {
                        readOK = false;
                        break;
                    }
                }

                if (!readOK)
                    break;

                int64_t pulseLength =
                    esp_timer_get_time()
                    - highStart;

                int byteIndex =
                    i / 8;

                int bitIndex =
                    7 - (i % 8);

                if (
                    pulseLength
                    > DHT_ONE_THRESHOLD_US
                )
                {
                    data[byteIndex] |=
                        (uint8_t)(
                            1U << bitIndex
                        );
                }
            }

            if (readOK)
            {
                uint8_t checksum =
                    (uint8_t)(
                        data[0] +
                        data[1] +
                        data[2] +
                        data[3]
                    );

                if (checksum == data[4])
                {
                    uint16_t rawHumidity =
                        ((uint16_t)data[0] << 8)
                        | data[1];

                    uint16_t rawTemperature =
                        ((uint16_t)data[2] << 8)
                        | data[3];

                    float newHumidity =
                        rawHumidity / 10.0f;

                    bool negative =
                        (rawTemperature & 0x8000U)
                        != 0;

                    rawTemperature &=
                        0x7FFFU;

                    float newTemperature =
                        rawTemperature / 10.0f;

                    if (negative)
                        newTemperature =
                            -newTemperature;

                    if (
                        newHumidity >= 0.0f &&
                        newHumidity <= 100.0f &&
                        newTemperature >= -40.0f &&
                        newTemperature <= 80.0f
                    )
                    {
                        *humidity =
                            newHumidity;

                        *temperature =
                            newTemperature;

                        success = true;
                    }
                    else
                    {
                        snprintf(
                            errorMessage,
                            sizeof(errorMessage),
                            "DHT22 invalid values: "
                            "T=%.2f C H=%.2f %%",
                            newTemperature,
                            newHumidity
                        );
                    }
                }
                else
                {
                    snprintf(
                        errorMessage,
                        sizeof(errorMessage),
                        "DHT22 checksum failed "
                        "(calc=%u received=%u)",
                        (unsigned)checksum,
                        (unsigned)data[4]
                    );
                }
            }
            else
            {
                snprintf(
                    errorMessage,
                    sizeof(errorMessage),
                    "DHT22 timing/read failed"
                );
            }
        }
    }

    if (
        !success &&
        !(responseLow && responseHigh)
    )
    {
        snprintf(
            errorMessage,
            sizeof(errorMessage),
            "DHT22 sensor response timeout"
        );
    }

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(DHT_PIN);

    portEXIT_CRITICAL(&dhtMux);

    if (errorMessage[0] != '\0')
    {
        logPrint(
            "%s\n",
            errorMessage
        );
    }

    return success;
}

// ====================================================
// SENSOR TASK
// ====================================================

void sensorTask(void *parameter)
{
    SensorData sensorData = {};

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
        logPrint(
            "LDR ADC initialization failed: %s\n",
            esp_err_to_name(result)
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
        logPrint(
            "LDR ADC channel configuration failed: %s\n",
            esp_err_to_name(result)
        );

        vTaskDelete(NULL);
        return;
    }

    sensorData.temperature = 0.0f;
    sensorData.humidity = 0.0f;
    sensorData.lightLevel = 0;
    sensorData.motionDetected = false;

    bool haveValidDhtReading =
        false;

    logPrint(
        "SensorTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        // --------------------------------------------
        // DHT22
        // --------------------------------------------

        float newTemperature =
            0.0f;

        float newHumidity =
            0.0f;

        if (
            dht22_read(
                &newTemperature,
                &newHumidity
            )
        )
        {
            sensorData.temperature =
                newTemperature;

            sensorData.humidity =
                newHumidity;

            haveValidDhtReading =
                true;
        }
        else
        {
            if (haveValidDhtReading)
            {
                logPrint(
                    "DHT22 reading failed - "
                    "keeping previous valid reading\n"
                );
            }
            else
            {
                logPrint(
                    "DHT22 reading failed - "
                    "no valid reading yet\n"
                );
            }
        }

        // --------------------------------------------
        // LDR (raw 0-4095 converted to 0-100 %)
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
                (int)(
                    (raw_value / 4095.0f)
                    * 100.0f
                );

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

        sensorData.motionDetected =
            isMotionDetected();

        // --------------------------------------------
        // SERIAL OUTPUT
        // --------------------------------------------

        // One protected write: the four lines cannot be split
        // by output from another task.
        logPrint(
            "Temperature: %.2f C\n"
            "Humidity: %.2f %%\n"
            "Light Level: %d %%\n"
            "Motion: %s\n",
            sensorData.temperature,
            sensorData.humidity,
            sensorData.lightLevel,
            sensorData.motionDetected
                ? "DETECTED"
                : "NONE"
        );

        // Only publish once a real DHT22 reading exists, so the
        // alarm never evaluates the placeholder 0.0 C value.
        if (haveValidDhtReading)
        {
            xQueueOverwrite(
                displayQueue,
                &sensorData
            );

            xQueueOverwrite(
                alarmQueue,
                &sensorData
            );

            logPrint(
                "Sensor data sent to queues\n"
            );

            notifyDisplay();
        }

        // Periodic sampling without drift (step 22).
        // DHT22 requires approximately 2 seconds between measurements.
        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}

// ====================================================
// MOTION TASK  (steps 31-34)
// ====================================================
//
// State machine:
//
//   ACTIVE   --(no PIR activity for 15 s)-->  INACTIVE
//   INACTIVE --(PIR activity)------------->  ACTIVE
//
// The inactivity timer starts at boot, so the system goes
// INACTIVE 15 s after startup if the PIR is never triggered.
//
// MotionTask only updates the system state. SensorTask owns
// the SensorData structure and DisplayTask owns the OLED.
//
// ====================================================

void motionTask(void *parameter)
{
    motionTaskHandle = xTaskGetCurrentTaskHandle();

    gpio_set_direction(
        PIR_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pulldown_en(PIR_PIN);

    // Interrupt on both edges of the PIR output
    gpio_set_intr_type(
        PIR_PIN,
        GPIO_INTR_ANYEDGE
    );

    gpio_isr_handler_add(
        PIR_PIN,
        pirIsr,
        NULL
    );

    int64_t lastMotionTime =
        esp_timer_get_time();

    bool lastPirLevel =
        false;

    logPrint(
        "MotionTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        // Sleep until the PIR output changes (interrupt) or 100 ms
        // pass (needed to check the inactivity timeout).
        ulTaskNotifyTake(
            pdTRUE,
            ms_to_ticks(100)
        );

        bool pirLevel =
            gpio_get_level(PIR_PIN) != 0;

        int64_t now =
            esp_timer_get_time();

        // --------------------------------------------
        // PIR OUTPUT CHANGED (either direction)
        // --------------------------------------------

        if (pirLevel != lastPirLevel)
        {
            lastPirLevel =
                pirLevel;

            if (pirLevel)
            {
                // Motion stays flagged until the inactivity
                // timeout, so it is NOT cleared on the falling edge.
                xEventGroupSetBits(
                    systemEvents,
                    EVENT_MOTION
                );

                logPrint(
                    "MotionTask: MOTION DETECTED\n"
                );
            }
            else
            {
                logPrint(
                    "MotionTask: motion ended\n"
                );
            }

            notifyDisplay();
        }

        // --------------------------------------------
        // PIR ACTIVE: restart the inactivity timer and
        // restore ACTIVE if the system was INACTIVE
        // --------------------------------------------

        if (pirLevel)
        {
            lastMotionTime =
                now;

            if (getSystemState() == SystemState::INACTIVE)
            {
                xEventGroupSetBits(
                    systemEvents,
                    EVENT_ACTIVE
                );

                logPrint(
                    "System entering ACTIVE mode\n"
                );

                notifyDisplay();
            }
        }

        // --------------------------------------------
        // INACTIVITY TIMEOUT
        // --------------------------------------------

        if (
            getSystemState() == SystemState::ACTIVE &&
            (now - lastMotionTime) >= INACTIVITY_TIMEOUT_US
        )
        {
            xEventGroupClearBits(
                systemEvents,
                EVENT_ACTIVE | EVENT_MOTION
            );

            logPrint(
                "MotionTask: NO MOTION - "
                "15 SECOND TIMEOUT\n"
                "System entering INACTIVE mode\n"
            );

            notifyDisplay();
        }
    }
}

// ====================================================
// ENCODER INTERRUPT
// ====================================================
//
// Wokwi's KY-040 only pulls CLK and DT low for a few
// milliseconds per click, so polling every 5-10 ms can
// miss the whole click. Instead, an interrupt fires on
// the falling edge of CLK and reads DT right away:
//
//   DT is HIGH  -> clockwise
//   DT is LOW   -> counter-clockwise
//
// (Clockwise: CLK goes low first, then DT.
//  Counter-clockwise: DT goes low first, then CLK.)
//
// ====================================================

static void IRAM_ATTR encoderIsr(void *arg)
{
    int8_t direction =
        (gpio_get_level(ENCODER_DT) != 0) ? 1 : -1;

    if (ENCODER_REVERSE)
        direction = -direction;

    BaseType_t higherPriorityTaskWoken = pdFALSE;

    xQueueSendFromISR(
        encoderQueue,
        &direction,
        &higherPriorityTaskWoken
    );

    if (higherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

// ====================================================
// INPUT TASK
// ====================================================
//
// Clockwise:         TEMPERATURE -> HUMIDITY -> LIGHT -> MOTION -> TEMPERATURE
// Counter-clockwise: reverse, with wraparound
// Button press:      next page (same order as clockwise)
//
// The task blocks on encoderQueue (max 20 ms), so it never
// busy-loops and the page changes immediately on each click.
//
// ====================================================

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

    gpio_set_direction(
        ENCODER_SW,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(
        ENCODER_CLK
    );

    gpio_pullup_en(
        ENCODER_DT
    );

    gpio_pullup_en(
        ENCODER_SW
    );

    // Interrupt on CLK falling edge
    gpio_set_intr_type(
        ENCODER_CLK,
        GPIO_INTR_NEGEDGE
    );

    esp_err_t err =
        gpio_install_isr_service(0);

    if (
        err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE
    )
    {
        logPrint(
            "gpio_install_isr_service failed: %s\n",
            esp_err_to_name(err)
        );
    }

    gpio_isr_handler_add(
        ENCODER_CLK,
        encoderIsr,
        NULL
    );

    DisplayMode currentMode =
        DisplayMode::TEMPERATURE;

    bool lastButton =
        gpio_get_level(
            ENCODER_SW
        );

    logPrint(
        "InputTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        // --------------------------------------------
        // ROTATION (event from the interrupt)
        // --------------------------------------------

        int8_t direction = 0;

        if (
            xQueueReceive(
                encoderQueue,
                &direction,
                ms_to_ticks(20)
            ) == pdPASS
        )
        {
            // Encoder is only active in the ACTIVE state (step 33/34)
            if (getSystemState() == SystemState::ACTIVE)
            {
                if (direction > 0)
                {
                    currentMode =
                        nextMode(currentMode);

                    logPrint(
                        "InputTask: clockwise -> %s\n",
                        modeToString(currentMode)
                    );
                }
                else
                {
                    currentMode =
                        previousMode(currentMode);

                    logPrint(
                        "InputTask: counter-clockwise -> %s\n",
                        modeToString(currentMode)
                    );
                }

                xQueueOverwrite(
                    modeQueue,
                    &currentMode
                );

                notifyDisplay();
            }
        }

        // --------------------------------------------
        // ENCODER BUTTON: next page
        // --------------------------------------------

        bool currentButton =
            gpio_get_level(
                ENCODER_SW
            );

        if (
            lastButton == 1 &&
            currentButton == 0 &&
            getSystemState() == SystemState::ACTIVE
        )
        {
            currentMode =
                nextMode(currentMode);

            xQueueOverwrite(
                modeQueue,
                &currentMode
            );

            notifyDisplay();

            logPrint(
                "InputTask: button pressed -> %s\n",
                modeToString(currentMode)
            );

            // Simple debounce
            vTaskDelay(
                ms_to_ticks(50)
            );
        }

        lastButton =
            currentButton;
    }
}

// ====================================================
// DISPLAY PAGE DRAWING (large text)
// ====================================================
//
// Layout (128 x 64, 8 pages of 8 px):
//   page 0     : "ROOM MONITOR"  (small)
//   pages 2-3  : page title      (double size)
//   pages 5-6  : value           (double size)
//
// ====================================================

static void drawPage(
    DisplayMode mode,
    const SensorData &data
)
{
    char line[16];

    oled_clear();

    oled_text(
        0,
        0,
        "ROOM MONITOR"
    );

    // EVENT_ALARM: show a small alarm marker on every page
    if (isAlarmActive())
    {
        oled_text(
            90,
            0,
            "ALARM"
        );
    }

    switch (mode)
    {
        case DisplayMode::TEMPERATURE:

            oled_text_big(0, 2, "TEMP");

            snprintf(
                line,
                sizeof(line),
                "%.1f C",
                data.temperature
            );

            oled_text_big(0, 5, line);

            logPrint(
                "DisplayTask: "
                "Temperature page = %.2f C\n",
                data.temperature
            );

            break;

        case DisplayMode::HUMIDITY:

            oled_text_big(0, 2, "HUMIDITY");

            snprintf(
                line,
                sizeof(line),
                "%d %%",
                (int)data.humidity
            );

            oled_text_big(0, 5, line);

            logPrint(
                "DisplayTask: "
                "Humidity page = %.2f %%\n",
                data.humidity
            );

            break;

        case DisplayMode::LIGHT:

            oled_text_big(0, 2, "LIGHT");

            snprintf(
                line,
                sizeof(line),
                "%d %%",
                data.lightLevel
            );

            oled_text_big(0, 5, line);

            logPrint(
                "DisplayTask: "
                "Light page = %d %%\n",
                data.lightLevel
            );

            break;

        case DisplayMode::MOTION:

            oled_text_big(0, 2, "MOTION");

            oled_text_big(
                0,
                5,
                isMotionDetected()
                    ? "DETECTED"
                    : "NONE"
            );

            logPrint(
                "DisplayTask: Motion page = %s\n",
                isMotionDetected()
                    ? "DETECTED"
                    : "NONE"
            );

            break;
    }
}

// ====================================================
// DISPLAY TASK
// ====================================================
//
// DisplayTask is the ONLY task that touches the OLED.
//
// It sleeps until another task notifies it (new sensor data,
// new page, ACTIVE/INACTIVE change), so updates appear right
// away, and it only redraws when something actually changed.
//
// ====================================================

void displayTask(void *parameter)
{
    SensorData sensorData = {};

    DisplayMode currentMode =
        DisplayMode::TEMPERATURE;

    bool haveData = false;
    bool displayOn = true;
    bool needRedraw = true;
    bool lastMotionShown = false;
    bool lastAlarmShown = false;

    oled_init();
    oled_clear();
    oled_flush();

    logPrint(
        "DisplayTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        // --------------------------------------------
        // SLEEP UNTIL NOTIFIED (new data, new page, state change)
        // or 100 ms have passed. Notifications wake it instantly.
        // --------------------------------------------

        ulTaskNotifyTake(
            pdTRUE,
            ms_to_ticks(100)
        );

        if (
            xQueueReceive(
                displayQueue,
                &sensorData,
                0
            ) == pdPASS
        )
        {
            haveData = true;
            needRedraw = true;
        }

        // --------------------------------------------
        // CHECK FOR NEW DISPLAY MODE
        // --------------------------------------------

        DisplayMode newMode;

        if (
            xQueueReceive(
                modeQueue,
                &newMode,
                0
            ) == pdPASS
        )
        {
            currentMode = newMode;
            needRedraw = true;
        }

        // --------------------------------------------
        // INACTIVE: blank the OLED once, then do nothing
        // --------------------------------------------

        if (getSystemState() != SystemState::ACTIVE)
        {
            if (displayOn)
            {
                oled_clear();
                oled_flush();
                oled_power(false);
                displayOn = false;

                logPrint(
                    "DisplayTask: OLED OFF - "
                    "system INACTIVE\n"
                );
            }

            continue;
        }

        // --------------------------------------------
        // ACTIVE: wake the OLED if it was off
        // --------------------------------------------

        if (!displayOn)
        {
            oled_power(true);
            displayOn = true;
            needRedraw = true;

            logPrint(
                "DisplayTask: OLED ON - "
                "system ACTIVE\n"
            );
        }

        // Redraw when EVENT_ALARM changes, or when EVENT_MOTION
        // changes while the motion page is showing.
        bool motionNow = isMotionDetected();
        bool alarmNow = isAlarmActive();

        if (
            (currentMode == DisplayMode::MOTION &&
             motionNow != lastMotionShown) ||
            alarmNow != lastAlarmShown
        )
        {
            needRedraw = true;
        }

        if (!haveData || !needRedraw)
            continue;

        needRedraw = false;
        lastMotionShown = motionNow;
        lastAlarmShown = alarmNow;

        drawPage(
            currentMode,
            sensorData
        );

        oled_flush();
    }
}

// ====================================================
// ALARM TASK
// ====================================================
//
// Decision logic lives in evaluateTemperature() (pure function).
// This task only consumes the queue and reacts to the result.
// Buzzer control would go here (there is no buzzer in the
// current diagram.json).
//
// ====================================================

void alarmTask(void *parameter)
{
    SensorData sensorData = {};

    AlarmState lastState =
        AlarmState::NORMAL;

    bool firstEvaluation = true;

    logPrint(
        "AlarmTask: Started on CPU %d\n",
        xPortGetCoreID()
    );

    while (1)
    {
        if (
            xQueueReceive(
                alarmQueue,
                &sensorData,
                portMAX_DELAY
            ) == pdPASS
        )
        {
            // Alarm is only active in ACTIVE state (step 33/34)
            if (getSystemState() != SystemState::ACTIVE)
            {
                if (isAlarmActive())
                {
                    xEventGroupClearBits(
                        systemEvents,
                        EVENT_ALARM
                    );

                    logPrint(
                        "Alarm cleared - system INACTIVE\n"
                    );
                }

                logPrint(
                    "Status: INACTIVE | Alarm: OFF\n"
                );

                firstEvaluation = true;

                continue;
            }

            AlarmState state =
                evaluateTemperature(
                    sensorData.temperature
                );

            if (firstEvaluation || state != lastState)
            {
                logPrint(
                    "Alarm State: %s (%.1f C)\n",
                    alarmStateToString(state),
                    sensorData.temperature
                );

                // Publish the result as an event
                if (state == AlarmState::NORMAL)
                {
                    xEventGroupClearBits(
                        systemEvents,
                        EVENT_ALARM
                    );
                }
                else
                {
                    xEventGroupSetBits(
                        systemEvents,
                        EVENT_ALARM
                    );
                }

                // Hardware hook: drive the buzzer here based on 'state'.

                notifyDisplay();

                lastState = state;
                firstEvaluation = false;
            }

            // One status line per sample so the state is always visible
            logPrint(
                "Status: ACTIVE | Alarm: %s\n",
                alarmStateToString(state)
            );
        }
    }
}

// ====================================================
// SERIAL MUTEX TEST (step 36) - proves the mutex works
// ====================================================
//
// Two extra tasks print one message in several separate pieces,
// on different CPU cores, at the same time.
//
//   SERIAL_MUTEX_TEST 0  -> normal program (test tasks not created)
//   SERIAL_MUTEX_TEST 1  -> start the two test tasks
//
//   SERIAL_MUTEX_TEST_USE_MUTEX 0 -> NO protection: the pieces of
//        message [A] and [B] get mixed together in the log
//   SERIAL_MUTEX_TEST_USE_MUTEX 1 -> mutex held for the whole
//        message: every message comes out complete
//
// Set SERIAL_MUTEX_TEST back to 0 when you are done testing.
//
// ====================================================

#define SERIAL_MUTEX_TEST 0
#define SERIAL_MUTEX_TEST_USE_MUTEX 0

#if SERIAL_MUTEX_TEST

static void serialTestTask(void *parameter)
{
    const char *name = (const char *)parameter;

    while (1)
    {
#if SERIAL_MUTEX_TEST_USE_MUTEX
        xSemaphoreTake(serialMutex, portMAX_DELAY);
#endif

        // One message written in four separate pieces
        printf("[%s] one, ", name);
        printf("two, ");
        printf("three, ");
        printf("four\n");

#if SERIAL_MUTEX_TEST_USE_MUTEX
        xSemaphoreGive(serialMutex);
#endif

        vTaskDelay(ms_to_ticks(10));
    }
}

#endif

// ====================================================
// MAIN
// ====================================================

extern "C" void app_main(void)
{
    // Create the serial mutex before anything prints
    serialMutex =
        xSemaphoreCreateMutex();

    logPrint("\n");

    logPrint(
        "====================================\n"
    );

    logPrint(
        "BCA152 FreeRTOS Multisensor\n"
    );

    logPrint(
        "System starting...\n"
    );

    logPrint(
        "====================================\n"
    );

    // --------------------------------------------
    // DHT22 IDLE STATE
    // --------------------------------------------

    gpio_set_direction(
        DHT_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pullup_en(
        DHT_PIN
    );

    // --------------------------------------------
    // PIR
    // --------------------------------------------

    gpio_set_direction(
        PIR_PIN,
        GPIO_MODE_INPUT
    );

    gpio_pulldown_en(
        PIR_PIN
    );

    // --------------------------------------------
    // GPIO INTERRUPT SERVICE (needed by encoder and PIR)
    // --------------------------------------------

    esp_err_t isrResult =
        gpio_install_isr_service(0);

    if (
        isrResult != ESP_OK &&
        isrResult != ESP_ERR_INVALID_STATE
    )
    {
        logPrint(
            "gpio_install_isr_service failed: %s\n",
            esp_err_to_name(isrResult)
        );
    }

    // --------------------------------------------
    // CREATE QUEUES
    // --------------------------------------------

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

    if (
        displayQueue == NULL ||
        alarmQueue == NULL ||
        modeQueue == NULL ||
        encoderQueue == NULL ||
        systemEvents == NULL
    )
    {
        logPrint(
            "ERROR: Failed to create queues\n"
        );

        return;
    }

    logPrint(
        "Queues and event group created successfully.\n"
    );

    // The system starts in the ACTIVE state
    xEventGroupSetBits(
        systemEvents,
        EVENT_ACTIVE
    );

    // --------------------------------------------
    // CPU ASSIGNMENT
    // --------------------------------------------
    //
    // CPU0:
    // SensorTask
    // MotionTask
    //
    // CPU1:
    // InputTask
    // DisplayTask
    // AlarmTask
    //
    // --------------------------------------------

    BaseType_t result;

    // --------------------------------------------
    // SENSOR TASK
    // --------------------------------------------

    result =
        xTaskCreatePinnedToCore(
            sensorTask,
            "SensorTask",
            6144,
            NULL,
            2,
            NULL,
            0
        );

    if (result != pdPASS)
    {
        logPrint(
            "ERROR: SensorTask creation failed\n"
        );

        return;
    }

    // --------------------------------------------
    // MOTION TASK
    // --------------------------------------------

    result =
        xTaskCreatePinnedToCore(
            motionTask,
            "MotionTask",
            4096,
            NULL,
            2,
            NULL,
            0
        );

    if (result != pdPASS)
    {
        logPrint(
            "ERROR: MotionTask creation failed\n"
        );

        return;
    }

    // --------------------------------------------
    // INPUT TASK
    // --------------------------------------------

    result =
        xTaskCreatePinnedToCore(
            inputTask,
            "InputTask",
            4096,
            NULL,
            2,
            NULL,
            1
        );

    if (result != pdPASS)
    {
        logPrint(
            "ERROR: InputTask creation failed\n"
        );

        return;
    }

    // --------------------------------------------
    // DISPLAY TASK
    // --------------------------------------------

    result =
        xTaskCreatePinnedToCore(
            displayTask,
            "DisplayTask",
            4096,
            NULL,
            1,
            &displayTaskHandle,
            1
        );

    if (result != pdPASS)
    {
        logPrint(
            "ERROR: DisplayTask creation failed\n"
        );

        return;
    }

    // --------------------------------------------
    // ALARM TASK
    // --------------------------------------------

    result =
        xTaskCreatePinnedToCore(
            alarmTask,
            "AlarmTask",
            4096,
            NULL,
            1,
            NULL,
            1
        );

    if (result != pdPASS)
    {
        logPrint(
            "ERROR: AlarmTask creation failed\n"
        );

        return;
    }

#if SERIAL_MUTEX_TEST
    // Test tasks on different cores so they really run in parallel
    xTaskCreatePinnedToCore(serialTestTask, "SerialTestA", 3072, (void *)"A", 3, NULL, 0);
    xTaskCreatePinnedToCore(serialTestTask, "SerialTestB", 3072, (void *)"B", 3, NULL, 1);
#endif

    logPrint(
        "All tasks started successfully.\n"
    );
}