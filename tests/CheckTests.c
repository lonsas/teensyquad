#include <stdlib.h>
#include <check.h>
#include "CheckTests.h"

int main(void)
{
    int number_failed;
    SRunner *sr;

    sr = srunner_create(CoreSuite());
    srunner_add_suite(sr, PidSuite());
    srunner_add_suite(sr, GyroControlSuite());
    srunner_add_suite(sr, MixSuite());
    srunner_add_suite(sr, SensorSuite());
    srunner_add_suite(sr, QuadStateSuite());
    srunner_add_suite(sr, EscControlSuite());

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);
    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
