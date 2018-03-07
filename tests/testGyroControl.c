#include <check.h>

#include "GyroControl.h"
#include "PIDConf.h"

static void setPidParameters()
{
    PidParameters pidParameters = {0};
    pidParameters.K = 1;
    pidParameters.b = 1;
    pidParameters.limit = 1e9;

    g_gyroRollPidParameters = pidParameters;
    g_gyroPitchPidParameters = pidParameters;
    g_gyroYawPidParameters = pidParameters;
}

START_TEST(testGyroControlSetup)
{
    double dbOmegaDotRoll;
    double dbOmegaDotPitch;
    double dbOmegaDotYaw;
    const double dbOmegaDotRollExpected = 0;
    const double dbOmegaDotPitchExpected = 10;
    const double dbOmegaDotYawExpected = 1e3;

    /* Override global PIDParameters */
    setPidParameters();

    gyroControlSetup();
    setOmegaRef(dbOmegaDotRollExpected, dbOmegaDotPitchExpected, dbOmegaDotYawExpected);
    gyroCalculateControl(&dbOmegaDotRoll, &dbOmegaDotPitch, &dbOmegaDotYaw);
    ck_assert_double_eq_tol(dbOmegaDotRoll, dbOmegaDotRollExpected, 1e-3);
    ck_assert_double_eq_tol(dbOmegaDotPitch, dbOmegaDotPitchExpected, 1e-3);
    ck_assert_double_eq_tol(dbOmegaDotYaw, dbOmegaDotYawExpected, 1e-3);
}
END_TEST


Suite * GyroControlSuite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("GyroControl");

    /* Core test case */
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, testGyroControlSetup);

    suite_add_tcase(s, tc_core);

    return s;
}
