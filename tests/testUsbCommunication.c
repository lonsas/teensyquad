#include <check.h>
#include "UsbCommunication.h"
#include "dummyHostFunction.h"
#include "COBS.h"
#include <stdlib.h>
#include <stdio.h>
#include <alloca.h>

struct command {
    uint32_t command;
    uint8_t data[];
};

static size_t dummyUsbReceive(uint8_t * buffer)
{
    size_t size;
    size = usbOutputEndIdx - usbOutputCurrIdx;
    memcpy(buffer, &usbOutputBuffer[usbOutputCurrIdx], size);
    usbOutputCurrIdx = usbOutputEndIdx;
    return size;
}

static void setup()
{
    usbInputBuffer[0] = 0;
    memset(&usbInputBuffer[1], 0xFF, USB_DATA_MAX_SIZE-1);
    usbInputEndIdx = 1;
    usbInputCurrIdx = 0;
    usbOutputEndIdx = 0;
    usbOutputCurrIdx = 0;
    usbSetup();
}

static void teardown()
{
    usbInputEndIdx = 0;
    usbInputCurrIdx = 0;
    usbOutputEndIdx = 0;
    usbOutputCurrIdx = 0;
}

START_TEST(testUpdateEmpty)
{
    usbUpdate();
    ck_assert_int_eq(usbOutputEndIdx, 0); /* Make sure nothing is sent */
}
END_TEST

START_TEST(testSendResponse)
{
    struct command * command;
    command = malloc(sizeof(command));
    command->command = USB_LOG_START;
    usbInputEndIdx += StuffData((uint8_t *)command, sizeof(command->command), &usbInputBuffer[usbInputEndIdx]);
    free(command);
    usbInputBuffer[usbInputEndIdx++] = 0;

    usbUpdate();
    /* Make sure something is sent */
    ck_assert_int_ne(usbOutputEndIdx, 0);
    /* Make sure the packet is zero terminated */
    ck_assert_int_eq(usbOutputBuffer[usbOutputEndIdx - 1], 0);
}
END_TEST

START_TEST(testLog)
{
    struct command * command;
    struct command * response;
    uint8_t usbBuffer[USB_DATA_MAX_SIZE];
    size_t responseLength;
    size_t usbBufferLength;

    command = alloca(sizeof(command));
    response = alloca(USB_DATA_MAX_SIZE);

    /* Set command to start logging */
    command->command = USB_LOG_START;
    usbInputEndIdx += StuffData((uint8_t *)command, sizeof(command->command), &usbInputBuffer[usbInputEndIdx]);

    /* Check that log data is sent back */
    for(int i = 0; i < 10; i++) {
        usbUpdate();
        usbBufferLength = dummyUsbReceive(usbBuffer);
        responseLength = UnStuffData(usbBuffer, usbBufferLength - 1, (uint8_t *)response);

        ck_assert_int_eq(response->command, USB_LOG_SENSOR);
        ck_assert_int_eq(responseLength, sizeof(uint32_t) + 6*sizeof(double));
    }

    /* Turn of data logging */
    command->command = USB_LOG_STOP;
    usbInputEndIdx += StuffData((uint8_t *)command, sizeof(command->command), &usbInputBuffer[usbInputEndIdx]);

    /* Check that there is no more data output */
    usbUpdate();
    usbBufferLength = dummyUsbReceive(usbBuffer);
    ck_assert_int_eq(usbBufferLength, 0);
}
END_TEST

Suite * UsbCommunicationSuite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("USB");

    /* Core test case */
    tc_core = tcase_create("USB");
    tcase_add_checked_fixture(tc_core, setup, teardown);
    tcase_add_test(tc_core, testUpdateEmpty);
    tcase_add_test(tc_core, testSendResponse);
    tcase_add_test(tc_core, testLog);
    suite_add_tcase(s, tc_core);

    return s;
}
