#include <check.h>
#include "UsbCommunication.h"
#include "dummyHostFunction.h"
#include "COBS.h"
#include <stdlib.h>
#include <stdio.h>
#include "PIDConf.h"

static PidParameters dummyPidParameters = {
  .K = 1,
  .Ti = 10,
  .Td = 0,
  .Tt = 10,
  .b = 1,
  .h = 0.001,
  .N = 10,
  .limit = 1e9,
};

static int findZero(uint8_t * buffer, int length)
{
    for(int i = 0; i < length; i++) {
        if(buffer[i] == 0) {
            return i;
        }
    }
    return -1;
}

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
    Command command = USB_LOG_START;
    usbInputEndIdx += StuffData((uint8_t *)&command, sizeof(command), &usbInputBuffer[usbInputEndIdx]);

    usbUpdate();
    /* Make sure something is sent */
    ck_assert_int_ne(usbOutputEndIdx, 0);
    /* Make sure the packet is zero terminated */
    ck_assert_int_eq(usbOutputBuffer[usbOutputEndIdx - 1], 0);
}
END_TEST

START_TEST(testLog)
{
    Command command = USB_LOG_START;
    struct UsbLogPacket response;
    uint8_t usbBuffer[USB_DATA_MAX_SIZE];
    size_t responseLength;
    size_t usbBufferLength;
    int len;

    /* Set command to start logging */
    usbInputEndIdx += StuffData((uint8_t *)&command, sizeof(command), &usbInputBuffer[usbInputEndIdx]);

    /* Check that log data is sent back */
    for(int i = 0; i < 10; i++) {
        usbUpdate();
        usbBufferLength = dummyUsbReceive(usbBuffer);
        len = findZero(usbBuffer, usbBufferLength);
        responseLength = UnStuffData(usbBuffer, len, (uint8_t *)&response);

        ck_assert_int_eq(response.command, USB_LOG_SENSOR);
        ck_assert_int_eq(responseLength, sizeof(struct UsbLogPacket));
    }

    /* Turn of data logging */
    command = USB_LOG_STOP;
    usbInputEndIdx += StuffData((uint8_t *)&command, sizeof(command), &usbInputBuffer[usbInputEndIdx]);

    /* Check that there is no more data output */
    usbUpdate();
    usbBufferLength = dummyUsbReceive(usbBuffer);
    ck_assert_int_eq(usbBufferLength, 0);
}
END_TEST

START_TEST(testReadPidParameters)
{
    Command command = USB_READ_GYRO_PID;
    struct UsbPidPacket response;
    uint8_t usbBuffer[USB_DATA_MAX_SIZE];
    size_t responseLength;
    size_t usbBufferLength;


    memcpy(&g_gyroRollPidParameters, &dummyPidParameters, sizeof(PidParameters));
    usbInputEndIdx += StuffData((uint8_t *)&command, sizeof(command), &usbInputBuffer[usbInputEndIdx]);

    usbUpdate();
    usbBufferLength = dummyUsbReceive(usbBuffer);
    responseLength = UnStuffData(usbBuffer, usbBufferLength - 1, (uint8_t *)&response);

    ck_assert_int_eq(response.command, USB_READ_GYRO_PID);
    ck_assert_int_eq(responseLength, sizeof(response));
    ck_assert_mem_eq(&response.pidParameters[0], &dummyPidParameters, sizeof(PidParameters));
}
END_TEST

START_TEST(testWritePidParameters)
{
    struct UsbPidPacket command;

    command.command = USB_WRITE_GYRO_PID;

    memcpy(&command.pidParameters[0], &dummyPidParameters, sizeof(PidParameters));
    usbInputEndIdx += StuffData((uint8_t *)&command, sizeof(command), &usbInputBuffer[usbInputEndIdx]);

    usbUpdate();

    ck_assert_mem_eq(&dummyPidParameters, &g_gyroRollPidParameters, sizeof(PidParameters));

    /* No response */
    ck_assert_int_eq(usbOutputEndIdx, 0);
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
    tcase_add_test(tc_core, testReadPidParameters);
    tcase_add_test(tc_core, testWritePidParameters);
    suite_add_tcase(s, tc_core);

    return s;
}
