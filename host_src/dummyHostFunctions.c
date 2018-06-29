#include "dummyHostFunction.h"
#include "MPU9150_c.h"
#include "Receiver.h"
#include "inttypes.h"
#include "core_pins.h"
#include "EscControl.h"
#include "string.h"

/* mpu9150 host */
int16_t g_ax;
int16_t g_ay;
int16_t g_az;
int16_t g_gx;
int16_t g_gy;
int16_t g_gz;

void mpu9150_initialize() {
    g_ax = 0;
    g_ay = 0;
    g_az = 0;
    g_gx = 0;
    g_gy = 0;
    g_gz = 0;
}

void mpu9150_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    *ax = g_ax;
    *ay = g_ay;
    *az = g_az;
    *gx = g_gx;
    *gy = g_gy;
    *gz = g_gz;
}



/* core stuff */
int pinArray[100];

void attachInterrupt(uint8_t pin, void (*function)(void), int mode)
{
    return;
}

uint32_t micros(void)
{
    return 0;
}

void pinMode(uint8_t pin, uint8_t mode)
{
    return;
}

void analogWrite(uint8_t pin, int val)
{
    pinArray[pin] = val;
}


void analogWriteFrequency(uint8_t pin, float frequency)
{
    return;
}
void analogWriteResolution(uint32_t bits)
{
    return;
}

uint8_t digitalRead(uint8_t pin)
{
    return pinArray[pin] >= 1;
}

/* EEPROM */
uint8_t eeprom[2048];
void eeprom_read_block(void *buf, const void *addr, uint32_t len)
{
  memcpy(buf, &eeprom[(uintptr_t)addr], len);
}
void eeprom_write_block(const void *buf, void *addr, uint32_t len)
{
  memcpy(&eeprom[(uintptr_t)addr], buf, len);
}

/* usb_serial */
#define USB_BUFFER_SIZE 8192
uint8_t usbInputBuffer[USB_BUFFER_SIZE];
uint8_t usbOutputBuffer[USB_BUFFER_SIZE];
unsigned int usbInputCurrIdx = 0;
unsigned int usbInputEndIdx = 0;
unsigned int usbOutputCurrIdx = 0;
unsigned int usbOutputEndIdx = 0;

int usb_serial_getchar()
{
    if(usbInputCurrIdx < usbInputEndIdx) {
        return usbInputBuffer[(usbInputCurrIdx++) % USB_BUFFER_SIZE];
    } else {
        return -1;
    }
}

int usb_serial_available()
{
    if((usbInputCurrIdx >= USB_BUFFER_SIZE) && (usbInputEndIdx >= USB_BUFFER_SIZE)) {
        usbInputEndIdx = usbInputEndIdx % USB_BUFFER_SIZE;
        usbInputCurrIdx = usbInputCurrIdx % USB_BUFFER_SIZE;
    }
    return usbInputEndIdx - usbInputCurrIdx;
}

int usb_serial_write(const void *buffer, uint32_t size)
{
    memcpy(&usbOutputBuffer[usbOutputEndIdx], buffer, size);
    usbOutputEndIdx += size;
    return size;
}

int usb_serial_peekchar()
{
    if(usbInputCurrIdx < usbInputEndIdx) {
        return usbInputBuffer[(usbInputCurrIdx) % USB_BUFFER_SIZE];
    } else {
        return -1;
    }
}

/* Model stuff */

void setMotion6(int ax, int ay, int az, int gx, int gy, int gz)
{
    g_ax = ax;
    g_ay = ay;
    g_az = az;
    g_gx = gx;
    g_gy = gy;
    g_gz = gz;
}

void getMotorOutput(int * motor)
{
    motor[0] = pinArray[MOTOR0_PIN];
    motor[1] = pinArray[MOTOR1_PIN];
    motor[2] = pinArray[MOTOR2_PIN];
    motor[3] = pinArray[MOTOR3_PIN];
}

