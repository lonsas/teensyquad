#ifndef USBserial_h_ /*Teensy3 header guard */
#define USBserial_h_

#include "inttypes.h"

int usb_serial_getchar(void);
int usb_serial_peekchar(void);
int usb_serial_available(void);
int usb_serial_read(void *buffer, uint32_t size);
void usb_serial_flush_input(void);
int usb_serial_putchar(uint8_t c);
int usb_serial_write(const void *buffer, uint32_t size);
int usb_serial_write_buffer_free(void);
void usb_serial_flush_output(void);

#endif
