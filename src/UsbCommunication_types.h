#ifndef USB_COMMUNICATION_TYPES_H
#define USB_COMMUNICATION_TYPES_H

#include <inttypes.h>
#include "PID_types.h"


#define USB_DATA_MAX_SIZE 255

#define USB_INVALID 0
#define USB_LOG_START 1
#define USB_LOG_STOP 2
#define USB_LOG_SENSOR 3
#define USB_WRITE_GYRO_PID 4
#define USB_READ_GYRO_PID 5
#define USB_WRITE_ANGLE_PID 6
#define USB_READ_ANGLE_PID 7

typedef uint8_t Command;

struct UsbLogPacket {
    Command command;
    double sensorData[6];
};

struct UsbPidPacket {
    Command command;
    PidParameters pidParameters[3];
};

#endif /* USB_COMMUNICATION_TYPES_H */
