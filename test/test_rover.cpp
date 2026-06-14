#include <unity.h>
#include "rover.h"

void test_rover_initialization(void) {
    MineDetectionRover rover;
    TEST_ASSERT_NOT_NULL(&rover);
}

void setUp(void) {
}

void tearDown(void) {
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_rover_initialization);
    return UNITY_END();
}
