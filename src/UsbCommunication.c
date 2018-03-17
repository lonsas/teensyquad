#include "UsbCommunication.h"
#include "core_pins.h"
#include "MCUConf.h"
#include <stdint.h>
#include <stddef.h>
#include "usb_serial.h"
#include <string.h>
#include "PIDConf.h"
#include "Sensor.h"

static bool m_correctData;
static bool m_sendLog;

static size_t StuffData(const uint8_t *ptr, size_t length, uint8_t *dst);
static size_t UnStuffData(const uint8_t *ptr, size_t length, uint8_t *dst);
static void sendPacket(uint32_t command, void * data, size_t length);
static void sendLog();
static bool receiveData(void * commandBuf, size_t * commandLength);
static void commandDo(void * commandBuf, size_t length);
static void receivePidParameters(double * pidParameterSet);
static void sendPidParameters();

void usbSetup()
{
    pinMode(USB_VOLT_PIN, INPUT);
    m_correctData = false;
}
bool usbConnected()
{
    return (digitalRead(USB_VOLT_PIN) == HIGH);
}

void usbUpdate()
{
    uint8_t commandBuf[USB_DATA_MAX_SIZE];
    size_t length = 0;
    while(usb_serial_available()) {
        if(receiveData(commandBuf, &length)) {
            commandDo(commandBuf, length);
        }
    }
    sendLog();
}

static void commandDo(void * commandBuf, size_t length)
{
    uint32_t command = ((uint32_t *)commandBuf)[0];
    switch(command) {
        case(USB_LOG_START):
            m_sendLog = true;
            break;
        case(USB_LOG_STOP):
            m_sendLog = false;
            break;
        case(USB_WRITE_PID):
            if((length - 1) != USB_PID_LENGTH) {
                receivePidParameters(commandBuf + sizeof(uint32_t));
            }
            break;
        case(USB_READ_PID):
            sendPidParameters();
            break;
        default:
            sendPacket(USB_LOG_INVALID, NULL, 0);
            break;
    }

}
static void sendPacket(uint32_t command, void * data, size_t length)
{
    /* FIXME: too much memory copying... */
    uint8_t dataBuf[USB_DATA_MAX_SIZE];
    uint8_t usbPacket[USB_DATA_MAX_SIZE];
    size_t size;

    dataBuf[0] = command;
    memcpy(&dataBuf[1], data, length);
    dataBuf[length + 1] = 0;
    size = StuffData(dataBuf, length + 2, usbPacket);
    usb_serial_write(usbPacket, size);
}

static void sendLog()
{
    double sensorData[6];

    SensorGetOmega(&sensorData[0], &sensorData[1], &sensorData[2]);
    SensorGetAngle(&sensorData[3], &sensorData[4], &sensorData[5]);
    sendPacket(USB_LOG_SENSOR, sensorData, 6);
}

static bool receiveData(void * commandBuf, size_t * commandLength)
{
    uint8_t data;
    size_t count;
    uint8_t dataBuf[USB_DATA_MAX_SIZE];

    /* Wait for 0 indicating start of new package */
    if(!m_correctData) {
        for(count = 0; count < USB_DATA_MAX_SIZE; count++) {
        if(usb_serial_available()) {
            data = usb_serial_getchar();
        } else {
            return false;
        }
        if(data == 0) {
            count = 0;
            break;
        }
      }
    }
    for(count = 0; count < USB_DATA_MAX_SIZE; count++) {
        if(usb_serial_available()) {
            dataBuf[count] = usb_serial_getchar();
            if(dataBuf[count] == 0) {
                m_correctData = true;
                break;
            }
        } else {
            m_correctData = false;
            return false;
        }
    }

    *commandLength = UnStuffData(dataBuf, count, commandBuf);
    return true;
}

static void receivePidParameters(double * pidParameterSets)
{
    PidParameters * currParameterSet[6] = {
        &g_gyroRollPidParameters,
        &g_gyroPitchPidParameters,
        &g_gyroYawPidParameters,
        &g_angleRollPidParameters,
        &g_anglePitchPidParameters,
        &g_angleYawPidParameters,
    };
    for(int i = 0; i < 6; i++) {
        /* PidParameters only contain double, no packing issues */
        memcpy(currParameterSet[i],
               &pidParameterSets[i * sizeof(PidParameters)],
               sizeof(PidParameters));
    }
    PIDConfSave();

}

static void sendPidParameters()
{
    uint8_t data[USB_PID_LENGTH];
    PidParameters * currParameterSet[6] = {
        &g_gyroRollPidParameters,
        &g_gyroPitchPidParameters,
        &g_gyroYawPidParameters,
        &g_angleRollPidParameters,
        &g_anglePitchPidParameters,
        &g_angleYawPidParameters,
    };
    for(int i = 0; i < 6; i++) {
        /* PidParameters only contain double, no packing issues */
        memcpy(&data[i * sizeof(PidParameters)],
               currParameterSet[i],
               sizeof(PidParameters));
    }
    sendPacket(USB_READ_PID, data, USB_PID_LENGTH);
}


/**********************************************************************
 * COBS, wikipedia implementation
 *********************************************************************/
/*
 * StuffData byte stuffs "length" bytes of data
 * at the location pointed to by "ptr", writing
 * the output to the location pointed to by "dst".
 *
 * Returns the length of the encoded data, which is
 * guaranteed to be <= length + 1 + (length - 1)/254.
 */
#define FinishBlock() (*code_ptr = code, code_ptr = dst++, code = 0x01)

static size_t StuffData(const uint8_t *ptr, size_t length, uint8_t *dst)
{
  const uint8_t *start = dst, *end = ptr + length;
  uint8_t *code_ptr = dst++;  /* Where to insert the leading count */
  uint8_t code = 0x01;

  for (; ptr < end; ptr++) {
    if (*ptr != 0) {
      *dst++ = *ptr;
      if (++code != 0xFF)
        continue;
    }
    FinishBlock();
  }

  FinishBlock();
  return dst - start;
}

/*
 * UnStuffData decodes "length" bytes of data at
 * the location pointed to by "ptr", writing the
 * output to the location pointed to by "dst".
 *
 * Returns the length of the decoded data (which is
 * guaranteed to be <= length.)
 */
static size_t UnStuffData(const uint8_t *ptr, size_t length, uint8_t *dst)
{
  const uint8_t *start = dst, *end = ptr + length;
  uint8_t code = 0xFF, copy = 0;

  while (ptr < end) {
    if (copy != 0) {
      *dst++ = *ptr++;
    } else {
      if (code != 0xFF)
        *dst++ = 0;
      copy = code = *ptr++;
      if (code == 0)
        break;   /* Should never happen */
    }
    copy--;
  }
  return dst - start;
}

