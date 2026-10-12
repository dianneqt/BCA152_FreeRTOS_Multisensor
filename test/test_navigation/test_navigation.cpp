#include <unity.h>
#include <string.h>

#include "display_mode.h"

void setUp(void) {}
void tearDown(void) {}

// Clockwise: TEMPERATURE -> HUMIDITY -> LIGHT -> MOTION
void test_next_follows_page_order(void)
{
    TEST_ASSERT_EQUAL(DisplayMode::HUMIDITY, nextMode(DisplayMode::TEMPERATURE));
    TEST_ASSERT_EQUAL(DisplayMode::LIGHT,    nextMode(DisplayMode::HUMIDITY));
    TEST_ASSERT_EQUAL(DisplayMode::MOTION,   nextMode(DisplayMode::LIGHT));
}

// MOTION -> TEMPERATURE (wraparound)
void test_next_wraps_around(void)
{
    TEST_ASSERT_EQUAL(DisplayMode::TEMPERATURE, nextMode(DisplayMode::MOTION));
}

// Counter-clockwise: MOTION -> LIGHT -> HUMIDITY -> TEMPERATURE
void test_previous_follows_reverse_order(void)
{
    TEST_ASSERT_EQUAL(DisplayMode::LIGHT,       previousMode(DisplayMode::MOTION));
    TEST_ASSERT_EQUAL(DisplayMode::HUMIDITY,    previousMode(DisplayMode::LIGHT));
    TEST_ASSERT_EQUAL(DisplayMode::TEMPERATURE, previousMode(DisplayMode::HUMIDITY));
}

// TEMPERATURE -> MOTION (wraparound)
void test_previous_wraps_around(void)
{
    TEST_ASSERT_EQUAL(DisplayMode::MOTION, previousMode(DisplayMode::TEMPERATURE));
}

// next then previous returns to the starting page, for every page
void test_next_then_previous_is_identity(void)
{
    DisplayMode pages[] = {
        DisplayMode::TEMPERATURE, DisplayMode::HUMIDITY,
        DisplayMode::LIGHT,       DisplayMode::MOTION
    };

    for (DisplayMode page : pages)
        TEST_ASSERT_EQUAL(page, previousMode(nextMode(page)));
}

// four clicks in one direction visit every page and come back
void test_four_clicks_return_to_start(void)
{
    DisplayMode mode = DisplayMode::TEMPERATURE;

    for (int i = 0; i < 4; i++)
        mode = nextMode(mode);

    TEST_ASSERT_EQUAL(DisplayMode::TEMPERATURE, mode);
}

void test_mode_names(void)
{
    TEST_ASSERT_EQUAL_STRING("TEMPERATURE", modeToString(DisplayMode::TEMPERATURE));
    TEST_ASSERT_EQUAL_STRING("HUMIDITY",    modeToString(DisplayMode::HUMIDITY));
    TEST_ASSERT_EQUAL_STRING("LIGHT",       modeToString(DisplayMode::LIGHT));
    TEST_ASSERT_EQUAL_STRING("MOTION",      modeToString(DisplayMode::MOTION));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_next_follows_page_order);
    RUN_TEST(test_next_wraps_around);
    RUN_TEST(test_previous_follows_reverse_order);
    RUN_TEST(test_previous_wraps_around);
    RUN_TEST(test_next_then_previous_is_identity);
    RUN_TEST(test_four_clicks_return_to_start);
    RUN_TEST(test_mode_names);
    return UNITY_END();
}