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


START_TEST(testPidP)
{
    double u;
    const double uExpected = 100;
    const double tol = 1e-6;
    PidParameters tParameters;
    memset(&tParameters, 0, sizeof(tParameters));
    tParameters.K = 1;
    tParameters.b = 1;
    tParameters.limit = 1e9;
    Pid tTestPid;
    tTestPid = tPidSetup(tParameters);
    u = dbCalculateAndUpdate(&tTestPid, 100, 0, 0);
    ck_assert_double_eq_tol(u, uExpected, tol);
}
END_TEST


START_TEST(testPidI)
{
    double u;
    const double uExpected = 100;
    const double tol = 1e-6;
    PidParameters tParameters;
    memset(&tParameters, 0, sizeof(tParameters));
    tParameters.K = 1;
    tParameters.b = 1;
    tParameters.Ti = 1;
    tParameters.h = 1;
    tParameters.limit = 1e9;
    Pid tTestPid;
    tTestPid = tPidSetup(tParameters);
    for(int i = 0; i < 100; i++) {
        u = dbCalculateAndUpdate(&tTestPid, 1, 0, 0);
    }
    ck_assert_double_eq_tol(u, uExpected, tol);
}
END_TEST

START_TEST(testPidTracking)
{
    double u;
    const double uExpected = 1;
    const double tol = 1e-6;
    PidParameters tParameters;
    memset(&tParameters, 0, sizeof(tParameters));
    tParameters.K = 1;
    tParameters.Ti = 1; /* Off */
    tParameters.h = 1;
    tParameters.Tt = 1;
    tParameters.limit = 1e9;
    Pid tTestPid;
    tTestPid = tPidSetup(tParameters);
    for(int i = 0; i < 100; i++) {
        u = dbCalculateAndUpdate(&tTestPid, 1, 0, 0);
    }
    ck_assert_double_eq_tol(u, uExpected, tol);
}
END_TEST

START_TEST(testPidD)
{
    double u;
    const double uExpected = -2;
    const double tol = 1e-6;
    PidParameters tParameters;
    memset(&tParameters, 0, sizeof(tParameters));
    tParameters.K = 1;
    tParameters.b = 1;
    tParameters.h = 1;
    tParameters.Td = 1;
    tParameters.N = 1e9; /* No derivative gain limit */
    tParameters.limit = 1e9;
    Pid tTestPid;
    tTestPid = tPidSetup(tParameters);
    for(int i = 1; i < 100; i++) {
        u = dbCalculateAndUpdate(&tTestPid, 2*i, 2*i, 0);
        ck_assert_double_eq_tol(u, uExpected, tol);
    }
}
END_TEST


START_TEST(testPidIZeroOrderSystemStep)
{
    double u = 0;
    const double uExpected = 0.63;
    const double tol = 5e-3;
    const double h = 0.001;
    PidParameters tParameters;
    memset(&tParameters, 0, sizeof(tParameters));
    tParameters.K = 0.0001; /* We do not want proportional gain */
    /* Choose Ti so that it reflects the time constant for the complete system */
    tParameters.Ti = 1*tParameters.K;
    tParameters.b = 0;
    tParameters.h = h;
    tParameters.Td = 0;
    tParameters.N = 1e9; /* No derivative gain limit */
    tParameters.limit = 1e9;
    Pid tTestPid;
    tTestPid = tPidSetup(tParameters);
    for(int i = 0; i*h < 1; i++) {
        u = dbCalculateAndUpdate(&tTestPid, 1, u, u);
    }
    ck_assert_double_eq_tol(u, uExpected, tol);
}
END_TEST

START_TEST(testPidPFirstOrderSystemStep)
{
    double u = 0;
    double x = 0;
    const double xExpected = 0.63;
    const double tol = 5e-3;
    const double h = 0.001;
    PidParameters tParameters;
    memset(&tParameters, 0, sizeof(tParameters));
    tParameters.K = 1;
    tParameters.Ti = 0;
    tParameters.b = 1;
    tParameters.h = h;
    tParameters.Td = 0;
    tParameters.N = 1e9; /* No derivative gain limit */
    tParameters.limit = 1e9;
    Pid tTestPid;
    tTestPid = tPidSetup(tParameters);
    for(int i = 0; i*h < 1; i++) {
        u = dbCalculateAndUpdate(&tTestPid, 1, x, u);
        x += u*h;
    }
    ck_assert_double_eq_tol(x, xExpected, tol);
}
END_TEST

START_TEST(testPidPDFirstOrderSystemStep)
{
    double u = 0;
    double x = 0;
    const double xExpectedMax = 0.6;
    const double xExpectedMin = 0.5;
    const double h = 0.001;
    PidParameters tParameters;
    memset(&tParameters, 0, sizeof(tParameters));
    tParameters.K = 1;
    tParameters.Ti = 0;
    tParameters.b = 1;
    tParameters.h = h;
    tParameters.Td = 0.1;
    tParameters.N = 1e9; /* No derivative gain limit */
    tParameters.limit = 1e9;
    Pid tTestPid;
    tTestPid = tPidSetup(tParameters);
    for(int i = 0; i*h < 1; i++) {
        u = dbCalculateAndUpdate(&tTestPid, 1, x, u);
        x += u*h;
    }
    ck_assert_double_lt(x, xExpectedMax);
    ck_assert_double_gt(x, xExpectedMin);
}
END_TEST

START_TEST(testPidPIFirstOrderSystemStepDisturb)
{
    double u = 0;
    double x = 0;
    const double xExpected = 1;
    const double h = 0.001;
    const double tol = 1e-2;
    PidParameters tParameters;
    memset(&tParameters, 0, sizeof(tParameters));
    tParameters.K = 1;
    tParameters.Ti = 1;
    tParameters.b = 1;
    tParameters.h = h;
    tParameters.Td = 0;
    tParameters.N = 1e9; /* No derivative gain limit */
    tParameters.limit = 1e9;
    Pid tTestPid;
    tTestPid = tPidSetup(tParameters);
    for(int i = 0; i*h < 10; i++) {
        u = dbCalculateAndUpdate(&tTestPid, 1, x, u);
        x += u*h-0.1*h;
    }
    ck_assert_double_eq_tol(x, xExpected, tol);
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
    tcase_add_test(tc_core, testPidP);
    tcase_add_test(tc_core, testPidI);
    tcase_add_test(tc_core, testPidD);
    tcase_add_test(tc_core, testPidTracking);
    tcase_add_test(tc_core, testPidIZeroOrderSystemStep);
    tcase_add_test(tc_core, testPidPFirstOrderSystemStep);
    tcase_add_test(tc_core, testPidPDFirstOrderSystemStep);
    tcase_add_test(tc_core, testPidPIFirstOrderSystemStepDisturb);
    suite_add_tcase(s, tc_core);

    return s;
}
