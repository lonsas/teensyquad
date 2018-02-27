#include <check.h>
#include "QuadState.h"
#include "Receiver.h"


extern int16_t g_ax;
extern int16_t g_gx;

static void initPwmWidth()
{
    for(int i = 0; i < 6; i++) {
        receiverSetManualPW(i, 1000);
    }
}

static void gotoReadyWait()
{
    stateInit();
    initPwmWidth();
    stateUpdate();
}

static void gotoArmed()
{
    gotoReadyWait();
    receiverSetManualPW(AUX1, 2000);
    stateUpdate();
}

START_TEST(testStateInit)
{
    stateInit();
    ck_assert_int_eq(getCurrState(), STARTUP);

}
END_TEST

START_TEST(testStateStartup)
{
    stateInit();

    stateUpdate();
    ck_assert_int_eq(getCurrState(), STARTUP);

    /* Should be stuck in STARTUP as receiver is not ready */
    stateUpdate();
    ck_assert_int_eq(getCurrState(), STARTUP);

    initPwmWidth();
    stateUpdate();
    ck_assert_int_eq(getCurrState(), READY_WAIT);
}
END_TEST

START_TEST(testStateNoCommand)
{
    stateInit();
    stateUpdate();
    initPwmWidth();
    for(int i = 0; i < 10; i++) {
        stateUpdate();
        ck_assert_int_eq(getCurrState(), READY_WAIT);
    }
}
END_TEST

START_TEST(testStateArmDisarm)
{
    gotoReadyWait();
    receiverSetManualPW(AUX1, 2000);
    for(int i = 0; i < 10; i++) {
        stateUpdate();
        ck_assert_int_eq(getCurrState(), ARMED);
    }
    /* Disarm */
    receiverSetManualPW(AUX1, 1000);
    stateUpdate();
    ck_assert_int_eq(getCurrState(), READY_WAIT);
}
END_TEST

START_TEST(testStateArmConditionThrottle)
{
    gotoReadyWait();
    receiverSetManualPW(AUX1, 2000);

    receiverSetManualPW(THROTTLE, 1500);
    stateUpdate();
    ck_assert_int_eq(getCurrState(), READY_WAIT);
}
END_TEST

START_TEST(testStateArmConditionSensor)
{
    gotoReadyWait();
    receiverSetManualPW(AUX1, 2000);

    g_gx = 200;
    g_ax = 100;
    stateDo();
    stateUpdate();
    ck_assert_int_eq(getCurrState(), READY_WAIT);
}
END_TEST


START_TEST(testStateArmConditionReset)
{
    gotoReadyWait();
    receiverSetManualPW(AUX1, 2000);

    receiverSetManualPW(THROTTLE, 1500);
    stateUpdate();
    receiverSetManualPW(THROTTLE, 1000);
    stateUpdate();
    ck_assert_int_eq(getCurrState(), READY_WAIT);
    receiverSetManualPW(AUX1, 1000);
    stateUpdate();
    ck_assert_int_eq(getCurrState(), READY_WAIT);
    receiverSetManualPW(AUX1, 2000);
    stateUpdate();
    ck_assert_int_eq(getCurrState(), ARMED);
}
END_TEST

START_TEST(createCoverage)
{
    gotoArmed();
    for(int i = 0; i < 100; i++) {
        stateDo();
        stateUpdate();
    }
}
END_TEST

Suite * QuadStateSuite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Quad state");

    /* Core test case */
    tc_core = tcase_create("Quad state");

    tcase_add_test(tc_core, testStateInit);
    tcase_add_test(tc_core, testStateStartup);
    tcase_add_test(tc_core, testStateNoCommand);
    tcase_add_test(tc_core, testStateArmDisarm);
    tcase_add_test(tc_core, testStateArmConditionThrottle);
    tcase_add_test(tc_core, testStateArmConditionSensor);
    tcase_add_test(tc_core, testStateArmConditionReset);
    tcase_add_test(tc_core, createCoverage);
    suite_add_tcase(s, tc_core);

    return s;
}
