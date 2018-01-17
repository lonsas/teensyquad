#include <check.h>

#include "PID.h"


START_TEST(testPidSetup)
{
    PidParameters tParameters;
    tParameters.K = 1;
    Pid tTestPid;
    Pid tEmptyPid;
    tTestPid = tPidSetup(tParameters);
    ck_assert_mem_ne(&tEmptyPid, &tTestPid, sizeof(Pid));
}
END_TEST


Suite * PidSuite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Pid");

    /* Core test case */
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, testPidSetup);
    suite_add_tcase(s, tc_core);

    return s;
}
