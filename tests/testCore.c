#include <check.h>

START_TEST(testAlwaysSucceeds)
{
    ck_assert(1);
}
END_TEST

Suite * CoreSuite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Core");

    /* Core test case */
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, testAlwaysSucceeds);
    suite_add_tcase(s, tc_core);

    return s;
}
