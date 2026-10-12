#include <unity.h>

#include "state_logic.h"

void setUp(void) {}
void tearDown(void) {}

// ACTIVE, no motion, timeout not reached -> stays ACTIVE
void test_active_no_timeout_stays_active(void)
{
    TEST_ASSERT_TRUE(evaluateSystemState(true, false, 5000, INACTIVITY_TIMEOUT_MS));
}

// ACTIVE, no motion, timeout reached -> INACTIVE
void test_active_timeout_becomes_inactive(void)
{
    TEST_ASSERT_FALSE(evaluateSystemState(true, false, 15000, INACTIVITY_TIMEOUT_MS));
    TEST_ASSERT_FALSE(evaluateSystemState(true, false, 20000, INACTIVITY_TIMEOUT_MS));
}

// INACTIVE, no motion -> stays INACTIVE
void test_inactive_no_motion_stays_inactive(void)
{
    TEST_ASSERT_FALSE(evaluateSystemState(false, false, 0, INACTIVITY_TIMEOUT_MS));
    TEST_ASSERT_FALSE(evaluateSystemState(false, false, 60000, INACTIVITY_TIMEOUT_MS));
}

// INACTIVE, motion -> ACTIVE (wake up)
void test_inactive_motion_becomes_active(void)
{
    TEST_ASSERT_TRUE(evaluateSystemState(false, true, 60000, INACTIVITY_TIMEOUT_MS));
}

// Boundary: 1 ms before the timeout is still ACTIVE
void test_timeout_boundary(void)
{
    TEST_ASSERT_TRUE (evaluateSystemState(true, false, 14999, INACTIVITY_TIMEOUT_MS));
    TEST_ASSERT_FALSE(evaluateSystemState(true, false, 15000, INACTIVITY_TIMEOUT_MS));
}

// Motion keeps an ACTIVE system active even past the timeout value
void test_motion_keeps_active(void)
{
    TEST_ASSERT_TRUE(evaluateSystemState(true, true, 99999, INACTIVITY_TIMEOUT_MS));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_active_no_timeout_stays_active);
    RUN_TEST(test_active_timeout_becomes_inactive);
    RUN_TEST(test_inactive_no_motion_stays_inactive);
    RUN_TEST(test_inactive_motion_becomes_active);
    RUN_TEST(test_timeout_boundary);
    RUN_TEST(test_motion_keeps_active);
    return UNITY_END();
}