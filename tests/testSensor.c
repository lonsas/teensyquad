#include <check.h>
#include <Sensor.h>
#include <math.h>
#include <stdio.h>

/* Sensor raw value dummies */
extern int16_t g_ax;
extern int16_t g_ay;
extern int16_t g_az;
extern int16_t g_gx;
extern int16_t g_gy;
extern int16_t g_gz;

START_TEST(testSensorInit)
{
    double ax;
    double ay;
    double az;
    double gx;
    double gy;
    double gz;
    SensorSetup();
    SensorGetOmega(&gx, &gy, &gz);
    ck_assert_double_eq(gx, 0);
    ck_assert_double_eq(gy, 0);
    ck_assert_double_eq(gz, 0);

    SensorGetAngle(&ax, &ay, &az);
    ck_assert_double_eq(ax, 0);
    ck_assert_double_eq(ay, 0);
    ck_assert_double_eq(az, 0);
}
END_TEST

START_TEST(testSensorUpdateNoSensor)
{
    double ax;
    double ay;
    double az;
    double gx;
    double gy;
    double gz;
    SensorSetup();
    for(int i = 0; i < 100; i++) {
        SensorUpdate();
    }
    /* No sensor input, values should not have moved */
    SensorGetOmega(&gx, &gy, &gz);
    ck_assert_double_eq(gx, 0);
    ck_assert_double_eq(gy, 0);
    ck_assert_double_eq(gz, 0);

    SensorGetAngle(&ax, &ay, &az);
    ck_assert_double_eq(ax, 0);
    ck_assert_double_eq(ay, 0);
    ck_assert_double_eq(az, 0);
}
END_TEST

START_TEST(testSensorUpdateAccelerationChange)
{
    double aRoll;
    double aPitch;
    double aYaw;
    double gRoll;
    double gPitch;
    double gYaw;
    SensorSetup();
    g_ax = 0;
    g_ay = 10;
    g_az = 10;
    g_gx = 0;
    g_gy = 0;
    g_gz = 0;
    for(int i = 0; i < 500; i++) {
        SensorUpdate();
    }

    SensorGetOmega(&gRoll, &gPitch, &gYaw);
    SensorGetAngle(&aRoll, &aPitch, &aYaw);

    ck_assert_double_eq_tol(gRoll, 0, 1e-3);
    ck_assert_double_eq_tol(gPitch, 0, 1e-3);
    ck_assert_double_eq_tol(gYaw, 0, 1e-3);
    ck_assert_double_eq_tol(aRoll, 3.14/4, 1e-3);
    ck_assert_double_eq_tol(aPitch, 0, 1e-3);
    ck_assert_double_eq_tol(aYaw, 0, 1e-3);
}
END_TEST

START_TEST(testSensorUpdateGyro)
{
    double aRoll;
    double aPitch;
    double aYaw;
    double gRoll;
    double gPitch;
    double gYaw;
    SensorSetup();
    g_ax = 0;
    g_ay = 10;
    g_az = 10;
    g_gx = 1*(1.0/(250.0/(1<<15)));
    g_gy = 0;
    g_gz = 0;
    for(int i = 0; i < 100; i++) {
        SensorUpdate();
    }

    SensorGetOmega(&gRoll, &gPitch, &gYaw);
    SensorGetAngle(&aRoll, &aPitch, &aYaw);

    ck_assert_double_eq_tol(gRoll, 1, 1e-3);
    ck_assert_double_eq_tol(gPitch, 0, 1e-3);
    ck_assert_double_eq_tol(gYaw, 0, 1e-3);
    ck_assert_double_gt(aRoll, 3.14/4);
    ck_assert_double_eq_tol(aPitch, 0, 1e-3);
    ck_assert_double_eq_tol(aYaw, 0, 1e-3);
}
END_TEST

Suite * SensorSuite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Sensor");

    /* Core test case */
    tc_core = tcase_create("Sensor");

    tcase_add_test(tc_core, testSensorInit);
    tcase_add_test(tc_core, testSensorUpdateNoSensor);
    tcase_add_test(tc_core, testSensorUpdateAccelerationChange);
    tcase_add_test(tc_core, testSensorUpdateGyro);

    suite_add_tcase(s, tc_core);

    return s;
}

