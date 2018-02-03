#include <check.h>
#include <mix.h>


extern void mixDistribute(double *roll, double *pitch, double *yaw, double *mixed);
extern void unmix(double *roll, double *pitch, double *yaw, double *mixed);
extern void mixOutput(double throttle, double dbBatVolt, double *mixed, double *output);
START_TEST(testMixDistribute)
{
    double output[4];
    double roll = 20;
    double pitch = 300;
    double yaw = 1000;
    double expectedOutput[4] = {(pitch+roll+yaw)/4,
                                (-pitch+roll-yaw)/4,
                                (pitch-roll-yaw)/4,
                                (-pitch-roll+yaw)/4};
    mixDistribute(&roll, &pitch, &yaw, output);
    for(int i = 0; i < 4; i++) {
        ck_assert_double_eq_tol(output[i], expectedOutput[i], 1e-6);
    }
}
END_TEST


START_TEST(testMixUnmix)
{
    double output[4];
    double roll = 20;
    double croll = roll;
    double pitch = 300;
    double cpitch = pitch;
    double yaw = 1000;
    double cyaw = yaw;
    mixDistribute(&roll, &pitch, &yaw, output);
    unmix(&roll, &pitch, &yaw, output);

    ck_assert_double_eq_tol(roll, croll, 1e-9);
    ck_assert_double_eq_tol(pitch, cpitch, 1e-9);
    ck_assert_double_eq_tol(yaw, cyaw, 1e-9);
    
}
END_TEST

START_TEST(testMixOutput)
{
    double roll;
    double pitch;
    double yaw;
    double mixed[4] = {0};
    const double cmixed[4] = {0};
    double output[4];
    mixOutput(1, 2, mixed, output);
    unmix(&roll, &pitch, &yaw, mixed);
    mixDistribute(&roll, &pitch, &yaw, mixed);
    for(int i = 0; i < 4; i++) {
        ck_assert_double_eq_tol(output[i], 1/2.0, 1e-9);
        ck_assert_double_eq_tol(mixed[i], cmixed[i], 1e-9);
    }
}
END_TEST

START_TEST(testMixOutputSaturated)
{
    double roll;
    double pitch;
    double yaw;
    double mixed[4] = {0};
    const double cmixed[4] = {0};
    double output[4];
    mixOutput(2, 1, mixed, output);
    unmix(&roll, &pitch, &yaw, mixed);
    mixDistribute(&roll, &pitch, &yaw, mixed);
    for(int i = 0; i < 4; i++) {
        ck_assert_double_eq_tol(output[i], 1*THROTTLE_MAX, 1e-9);
        ck_assert_double_eq_tol(mixed[i], cmixed[i], 1e-9);
    }
}
END_TEST

Suite * MixSuite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Mix");

    /* Core test case */
    tc_core = tcase_create("Mix");

    tcase_add_test(tc_core, testMixDistribute);
    tcase_add_test(tc_core, testMixUnmix);
    tcase_add_test(tc_core, testMixOutput);
    tcase_add_test(tc_core, testMixOutputSaturated);
    suite_add_tcase(s, tc_core);

    return s;
}
