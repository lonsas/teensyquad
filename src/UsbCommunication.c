#include "UsbCommunication.h"
#include "core_pins.h"
#include "MCUConf.h"
#include <stdint.h>
#include <stddef.h>
#include "usb_serial.h"
#include <string.h>
#include "PIDConf.h"
#include "Sensor.h"
#include "COBS.h"
#include <alloca.h>

static bool m_correctData;
static bool m_sendLog;


static void sendPacket(void * data, size_t length);
static void sendLog();
static bool receiveData(void * commandBuf, size_t * commandLength);
static void commandDo(void * commandBuf, size_t length);
static void receivePidParameters(double * pidParameterSet);
static void sendPidParameters();

struct command {
    uint32_t command;
    uint8_t data[];
};

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
    uint32_t * command = commandBuf;
    switch(command[0]) {
        case(USB_LOG_START):
            m_sendLog = true;
            break;
        case(USB_LOG_STOP):
            m_sendLog = false;
            break;
        case(USB_WRITE_PID):
            if((length - 1) == USB_PID_LENGTH) {
                receivePidParameters(commandBuf + sizeof(uint32_t));
            } else {
                sendPacket(USB_LOG_INVALID, sizeof(command));
            }
            break;
        case(USB_READ_PID):
            sendPidParameters();
            break;
        default:
            sendPacket(USB_LOG_INVALID, sizeof(command));
            break;
    }

}
static void sendPacket(void * data, size_t length)
{
    /* FIXME: too much memory copying... */
    uint8_t usbPacket[USB_DATA_MAX_SIZE];
    size_t size;

    size = StuffData(data, length, usbPacket);
    usb_serial_write(usbPacket, size);
}

static void sendLog()
{
    size_t command_size = sizeof(uint32_t) + 6 * sizeof(double);
    struct command * command = alloca(command_size);
    command->command = USB_LOG_SENSOR;
    double * sensorData = (void *)command->data;
    if(m_sendLog) {
        SensorGetOmega(&sensorData[0], &sensorData[1], &sensorData[2]);
        SensorGetAngle(&sensorData[3], &sensorData[4], &sensorData[5]);
        sendPacket(command, command_size);
    }
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
                m_correctData = true;
                break;
            }
        }
    }
    if(!m_correctData) {
        return false;
    }
    for(count = 0; count < USB_DATA_MAX_SIZE; count++) {
        if(usb_serial_available()) {
            dataBuf[count] = usb_serial_getchar();
            if(dataBuf[count] == 0) {
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
    size_t command_size = sizeof(uint32_t) + 6 * sizeof(PidParameters);
    struct command * command = alloca(command_size);
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
        memcpy(&command->data[i * sizeof(PidParameters)],
               currParameterSet[i],
               sizeof(PidParameters));
    }
    sendPacket(command, command_size);
}



