#ifndef USB_COMMUNICATION_H
#define USB_COMMUNICATION_H
#include <stdbool.h>
#include "PID_types.h"
#include <inttypes.h>


#define USB_DATA_MAX_SIZE 255

#define USB_LOG_INVALID 0
#define USB_LOG_START 1
#define USB_LOG_STOP 2
#define USB_LOG_SENSOR 3
#define USB_WRITE_GYRO_PID 5
#define USB_READ_GYRO_PID 6
#define USB_WRITE_ANGLE_PID 7
#define USB_READ_ANGLE_PID 8

typedef uint32_t Command;

struct UsbLogPacket {
    Command command;
    double sensorData[6];
};

struct UsbPidPacket {
    Command command;
    PidParameters pidParameters[3];
};


void usbSetup();
void usbUpdate();
bool usbConnected();


#endif /* USB_COMMUNICATION_H */
