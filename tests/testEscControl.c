#include <check.h>
#include "EscControl.h"

extern int pinArray[100];

static double scale(double val)
{
    return (val * 1000 + 1000) * SCALING;
}


START_TEST(testEscSetup)
{
    EscControlSetup();
    ck_assert(!EscControlIsArmed());
    ck_assert_double_eq_tol(pinArray[MOTOR0_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR1_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR2_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR3_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
}
END_TEST

START_TEST(testEscArm)
{
    EscControlSetup();
    EscControlArm();
    ck_assert(EscControlIsArmed());
    ck_assert_double_eq_tol(pinArray[MOTOR0_PIN], MOTOR_MICROS_ARM*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR1_PIN], MOTOR_MICROS_ARM*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR2_PIN], MOTOR_MICROS_ARM*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR3_PIN], MOTOR_MICROS_ARM*SCALING, 1.5e-0);
}
END_TEST

START_TEST(testEscDisarm)
{
    EscControlSetup();
    EscControlArm();
    EscControlDisarm();
    ck_assert(!EscControlIsArmed());
    ck_assert_double_eq_tol(pinArray[MOTOR0_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR1_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR2_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR3_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
}
END_TEST

START_TEST(testEscOutput)
{
    double motor[4];
    motor[0] = 1;
    motor[1] = 0.99;
    motor[2] = 0.1;
    motor[3] = MOTOR_MIN + 1e-3;
    EscControlSetup();
    EscControlArm();
    EscControlOutput(motor);
    ck_assert_double_eq_tol(pinArray[MOTOR0_PIN], scale(motor[0]), 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR1_PIN], scale(motor[1]), 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR2_PIN], scale(motor[2]), 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR3_PIN], scale(motor[3]), 1.5e-0);
}
END_TEST

START_TEST(testEscSaturationUpper)
{
    double motor[4];
    motor[0] = 1.1;
    motor[1] = 1e9;
    motor[2] = 1 + 1e-3;
    motor[3] = 1;
    EscControlSetup();
    EscControlArm();
    EscControlOutput(motor);
    ck_assert_double_eq_tol(pinArray[MOTOR0_PIN], scale(1), 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR1_PIN], scale(1), 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR2_PIN], scale(1), 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR3_PIN], scale(1), 1.5e-0);
}
END_TEST

START_TEST(testEscSaturationLower)
{
    double motor[4];
    motor[0] = 0;
    motor[1] = MOTOR_MIN;
    motor[2] = MOTOR_MIN - 1e-3;
    motor[3] = -1e9;
    EscControlSetup();
    EscControlArm();
    EscControlOutput(motor);
    ck_assert_double_eq_tol(pinArray[MOTOR0_PIN], scale(MOTOR_MIN), 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR1_PIN], scale(MOTOR_MIN), 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR2_PIN], scale(MOTOR_MIN), 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR3_PIN], scale(MOTOR_MIN), 1.5e-0);
}
END_TEST


START_TEST(testEscDisarmOutput)
{
    double motor[4];
    EscControlSetup();
    motor[0] = 1;
    motor[1] = 0.99;
    motor[2] = 0.1;
    motor[3] = MOTOR_MIN + 1e-3;
    EscControlOutput(motor);
    ck_assert_double_eq_tol(pinArray[MOTOR0_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR1_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR2_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
    ck_assert_double_eq_tol(pinArray[MOTOR3_PIN], MOTOR_MICROS_OFF*SCALING, 1.5e-0);
}
END_TEST

Suite * EscControlSuite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("ESC control");

    /* Core test case */
    tc_core = tcase_create("ESC control");

    tcase_add_test(tc_core, testEscSetup);
    tcase_add_test(tc_core, testEscArm);
    tcase_add_test(tc_core, testEscDisarm);
    tcase_add_test(tc_core, testEscOutput);
    tcase_add_test(tc_core, testEscSaturationUpper);
    tcase_add_test(tc_core, testEscSaturationLower);
    tcase_add_test(tc_core, testEscDisarmOutput);
    suite_add_tcase(s, tc_core);

    return s;
}
