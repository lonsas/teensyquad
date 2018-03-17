#ifndef USB_COMMUNICATION_H
#define USB_COMMUNICATION_H
#include "stdbool.h"
#include "PID_types.h"


#define USB_DATA_MAX_SIZE 255

#define USB_LOG_INVALID 0
#define USB_LOG_START 1
#define USB_LOG_STOP 2
#define USB_LOG_SENSOR 3
#define USB_WRITE_PID 4
#define USB_READ_PID 5

#define USB_PID_LENGTH (6 * sizeof(PidParameters))

void usbSetup();
void usbUpdate();
bool usbConnected();


#endif /* USB_COMMUNICATION_H */
