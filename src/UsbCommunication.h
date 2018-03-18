#ifndef USB_COMMUNICATION_H
#define USB_COMMUNICATION_H
#include <stdbool.h>
#include <inttypes.h>
#include "UsbCommunication_types.h"

void usbSetup();
void usbUpdate();
bool usbConnected();


#endif /* USB_COMMUNICATION_H */
