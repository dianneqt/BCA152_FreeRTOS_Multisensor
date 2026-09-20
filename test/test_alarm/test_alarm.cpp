#include <unity.h>

enum class AlarmState
{
    NORMAL,
    LOW_TEMPERATURE,
    HIGH_TEMPERATURE
};


AlarmState evaluateTemperature(float temperature)
{
    if (temperature < 20.0f)
    {
        return AlarmState::LOW_TEMPERATURE;
    }
    else if (temperature > 30.0f)
    {
        return AlarmState::HIGH_TEMPERATURE;
    }
    else
    {
        return AlarmState::NORMAL;
    }
}


void test_low_temperature()
{
    TEST_ASSERT_EQUAL_INT(
        (int)AlarmState::LOW_TEMPERATURE,
        (int)evaluateTemperature(19.0f)
    );
}


void test_normal_temperature()
{
    TEST_ASSERT_EQUAL_INT(
        (int)AlarmState::NORMAL,
        (int)evaluateTemperature(25.0f)
    );
}


void test_high_temperature()
{
    TEST_ASSERT_EQUAL_INT(
        (int)AlarmState::HIGH_TEMPERATURE,
        (int)evaluateTemperature(31.0f)
    );
}


void test_temperature_at_20()
{
    TEST_ASSERT_EQUAL_INT(
        (int)AlarmState::NORMAL,
        (int)evaluateTemperature(20.0f)
    );
}


void test_temperature_at_30()
{
    TEST_ASSERT_EQUAL_INT(
        (int)AlarmState::NORMAL,
        (int)evaluateTemperature(30.0f)
    );
}


extern "C" void app_main()
{
    UNITY_BEGIN();

    RUN_TEST(test_low_temperature);
    RUN_TEST(test_normal_temperature);
    RUN_TEST(test_high_temperature);
    RUN_TEST(test_temperature_at_20);
    RUN_TEST(test_temperature_at_30);

    UNITY_END();
}