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
static void receiveGyroPidParameters(struct UsbPidPacket * packet);
static void receiveAnglePidParameters(struct UsbPidPacket * packet);
static void sendGyroPidParameters();
static void sendAnglePidParameters();

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
        case(USB_WRITE_GYRO_PID):
            if(length == sizeof(struct UsbPidPacket)) {
                receiveGyroPidParameters(commandBuf);
            } else {
                sendPacket(USB_LOG_INVALID, sizeof(command));
            }
            break;
        case(USB_WRITE_ANGLE_PID):
            if(length == sizeof(struct UsbPidPacket)) {
                receiveAnglePidParameters(commandBuf);
            } else {
                sendPacket(USB_LOG_INVALID, sizeof(command));
            }
            break;
        case(USB_READ_GYRO_PID):
            sendGyroPidParameters();
            break;
        case(USB_READ_ANGLE_PID):
            sendAnglePidParameters();
            break;
        default:
            sendPacket(USB_LOG_INVALID, sizeof(command));
            break;
    }

}
static void sendPacket(void * data, size_t length)
{
    uint8_t usbPacket[USB_DATA_MAX_SIZE];
    size_t size;

    size = StuffData(data, length, usbPacket);
    usb_serial_write(usbPacket, size);
}

static void sendLog()
{
    struct UsbLogPacket packet;
    size_t size = sizeof(packet);
    double * sensorData = packet.sensorData;
    packet.command = USB_LOG_SENSOR;
    if(m_sendLog) {
        SensorGetOmega(&sensorData[0], &sensorData[1], &sensorData[2]);
        SensorGetAngle(&sensorData[3], &sensorData[4], &sensorData[5]);
        sendPacket(&packet, size);
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

static void receiveGyroPidParameters(struct UsbPidPacket * packet)
{
    PidParameters * currParameterSet[3] = {
        &g_gyroRollPidParameters,
        &g_gyroPitchPidParameters,
        &g_gyroYawPidParameters,
    };
    for(int i = 0; i < 3; i++) {
        /* PidParameters only contain double, no packing issues */
        memcpy(currParameterSet[i],
               &packet->pidParameters[i],
               sizeof(PidParameters));
    }
    PIDConfSave();

}

static void receiveAnglePidParameters(struct UsbPidPacket * packet)
{
    PidParameters * currParameterSet[3] = {

        &g_angleRollPidParameters,
        &g_anglePitchPidParameters,
        &g_angleYawPidParameters,
    };
    for(int i = 0; i < 3; i++) {
        /* PidParameters only contain double, no packing issues */
        memcpy(currParameterSet[i],
               &packet->pidParameters[i],
               sizeof(PidParameters));
    }
    PIDConfSave();

}

static void sendGyroPidParameters()
{
    struct UsbPidPacket packet;
    size_t size = sizeof(packet);
    packet.command = USB_READ_GYRO_PID;
    PidParameters * currParameterSet[3] = {
        &g_gyroRollPidParameters,
        &g_gyroPitchPidParameters,
        &g_gyroYawPidParameters,

    };
    for(int i = 0; i < 3; i++) {
        /* PidParameters only contain double, no packing issues */
        memcpy(&packet.pidParameters[i],
               currParameterSet[i],
               sizeof(PidParameters));
    }
    sendPacket(&packet, size);
}

static void sendAnglePidParameters()
{
    struct UsbPidPacket packet;
    size_t size = sizeof(packet);
    packet.command = USB_READ_ANGLE_PID;
    PidParameters * currParameterSet[3] = {
        &g_angleRollPidParameters,
        &g_anglePitchPidParameters,
        &g_angleYawPidParameters,
    };
    for(int i = 0; i < 3; i++) {
        /* PidParameters only contain double, no packing issues */
        memcpy(&packet.pidParameters[i],
               currParameterSet[i],
               sizeof(PidParameters));
    }
    sendPacket(&packet, size);
}
