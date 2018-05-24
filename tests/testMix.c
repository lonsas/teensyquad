#include <check.h>
#include <mix.h>

/* mix module local functions */
extern void mixDistribute(double *roll, double *pitch, double *yaw, double *mixed);
extern void unmix(double *roll, double *pitch, double *yaw, double *mixed);
extern void mixOutput(double throttle, double dbBatVolt, double *mixed, double *output);

START_TEST(testMixDistribute)
{
    double output[4];
    double roll = 20;
    double pitch = 300;
    double yaw = 1000;
    double expectedOutput[4] = {(-pitch+roll+yaw)/4,
                                (pitch+roll-yaw)/4,
                                (-pitch-roll-yaw)/4,
                                (pitch-roll+yaw)/4};
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
    mixOutput(0.5, 2, mixed, output);
    unmix(&roll, &pitch, &yaw, mixed);
    mixDistribute(&roll, &pitch, &yaw, mixed);
    for(int i = 0; i < 4; i++) {
        ck_assert_double_eq_tol(output[i], 1/2.0, 1e-9);
        ck_assert_double_eq_tol(mixed[i], cmixed[i], 1e-9);
    }
}
END_TEST

START_TEST(testMixOutputThrottleSaturated)
{
    double roll;
    double pitch;
    double yaw;
    double mixed[4] = {0};
    const double cmixed[4] = {0};
    double output[4];
    mixOutput(1.1, 12, mixed, output);
    unmix(&roll, &pitch, &yaw, mixed);
    mixDistribute(&roll, &pitch, &yaw, mixed);
    for(int i = 0; i < 4; i++) {
        ck_assert_double_eq_tol(output[i], THROTTLE_MAX, 1e-9);
        ck_assert_double_eq_tol(mixed[i], cmixed[i], 1e-9);
    }
}
END_TEST

START_TEST(testMixOutputSaturated)
{
    double roll;
    double pitch;
    double yaw;
    double mixed[4] = {6,6,0,0};
    const double cmixed[4] = {2.5,2.5,-2.5,-2.5};
    double output[4];
    const double coutput[4] = {1,1,0.5,0.5};
    mixOutput(0.5, 10, mixed, output);
    unmix(&roll, &pitch, &yaw, mixed);
    mixDistribute(&roll, &pitch, &yaw, mixed);
    for(int i = 0; i < 4; i++) {
        ck_assert_double_eq_tol(output[i], coutput[i], 1e-9);
        ck_assert_double_eq_tol(mixed[i], cmixed[i], 1e-9);
    }
}
END_TEST

START_TEST(testMix)
{
    double throttle = 0.5;
    double roll = 2;
    double pitch = 3;
    double yaw = 5;
    double battery = 10;
    double croll = 2;
    double cpitch = 3;
    double cyaw = 5;

    double output[4];
    double coutput[4] = {(throttle * battery + croll/4 - cpitch/4 - cyaw/4) / battery,
        (throttle * battery + croll/4 + cpitch/4 + cyaw/4) / battery,
        (throttle * battery - croll/4 - cpitch/4 + cyaw/4) / battery,
        (throttle * battery - croll/4 + cpitch/4 - cyaw/4) / battery};

    mix(throttle, battery, &roll, &pitch, &yaw, output);

    ck_assert_double_eq_tol(roll, croll, 1e-9);
    ck_assert_double_eq_tol(pitch, cpitch, 1e-9);
    ck_assert_double_eq_tol(yaw, cyaw, 1e-9);

    ck_assert_double_eq_tol(output[0], coutput[0], 1e-9);
    ck_assert_double_eq_tol(output[1], coutput[1], 1e-9);
    ck_assert_double_eq_tol(output[2], coutput[2], 1e-9);
    ck_assert_double_eq_tol(output[3], coutput[3], 1e-9);

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
    tcase_add_test(tc_core, testMixOutputThrottleSaturated);
    tcase_add_test(tc_core, testMixOutputSaturated);
    tcase_add_test(tc_core, testMix);
    suite_add_tcase(s, tc_core);

    return s;
}
