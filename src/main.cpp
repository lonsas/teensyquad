#include "WProgram.h"

/* Used for interrupts */
#include "core_pins.h"
#include "i2c_t3.h"
#include "I2Cdev.h"
#include "MPU9150.h"


#define RADIO_PINS 6

#define SENSOR_ADDRESS 0x68

const static uint8_t RADIOPIN[RADIO_PINS] = {3,4,5,6,7,8};
/* Make sure the RISE pin is on a failsafe signal i.e throttle */
const static uint8_t RADIOPIN_RISE = 2;
const static uint8_t MOTORPIN = 23;

volatile uint32_t radio_rise = 0;
volatile uint32_t width[6];

int16_t acc[3];
int16_t gyro[3];
int16_t mag[3];

struct serialData {
    int16_t acc[3];
    int16_t gyro[3];
    uint32_t t;
    uint32_t dt;
} serialData;

MPU9150 mpu9150;

void radio_pw_rise_isr() {
    radio_rise = micros();
}

void radio_pw0_isr() {
        width[0] = micros() - radio_rise;
}
void radio_pw1_isr() {
        width[1] = micros() - radio_rise;
}
void radio_pw2_isr() {
        width[2] = micros() - radio_rise;
}
void radio_pw3_isr() {
        width[3] = micros() - radio_rise;
}
void radio_pw4_isr() {
        width[4] = micros() - radio_rise;
}
void radio_pw5_isr() {
        width[5] = micros() - radio_rise;
}

void setup_radio() {
    
    pinMode(RADIOPIN_RISE, INPUT);
    for(int i = 0; i < RADIO_PINS; i++) {
        pinMode(RADIOPIN[i], INPUT);
    }
    attachInterrupt(RADIOPIN_RISE, radio_pw_rise_isr, RISING);
    attachInterrupt(RADIOPIN[0], radio_pw0_isr, FALLING);
    attachInterrupt(RADIOPIN[1], radio_pw1_isr, FALLING);
    attachInterrupt(RADIOPIN[2], radio_pw2_isr, FALLING);
    attachInterrupt(RADIOPIN[3], radio_pw3_isr, FALLING);
    attachInterrupt(RADIOPIN[4], radio_pw4_isr, FALLING);
    attachInterrupt(RADIOPIN[5], radio_pw5_isr, FALLING);
}

void setup_motor() {
    pinMode(MOTORPIN, OUTPUT);
    analogWriteFrequency(MOTORPIN, 400);
    analogWriteResolution(12);
}

uint16_t read_16bit_register(uint8_t high) {
    uint16_t result;
    Wire.beginTransmission(SENSOR_ADDRESS);
    Wire.write(high);
    Wire.endTransmission(I2C_STOP);
    Wire.requestFrom(SENSOR_ADDRESS, 1, I2C_NOSTOP);
    result = (Wire.readByte() << 8);
    
    Wire.beginTransmission(SENSOR_ADDRESS);
    Wire.write(high+1);
    Wire.endTransmission(I2C_STOP);
    Wire.requestFrom(SENSOR_ADDRESS, 1, I2C_NOSTOP);
    result |= Wire.readByte();
    return result;
}


void sensor_test() {
    //mpu9150.getMotion9(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2], &mag[0], &mag[1], &mag[2]);
    mpu9150.getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
}

void setup_sensor() {
    Wire.begin(I2C_MASTER,0x0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    
    mpu9150.initialize();
}




extern "C" int main(void)
{
    setup_radio();
    setup_motor();
    setup_sensor();
	pinMode(13, OUTPUT);
    uint32_t i = 0;
    uint32_t t_start = micros();
    uint16_t output;
	while (1) {
            sensor_test();
            i++;
            output = width[2]*1.6384;
            analogWrite(MOTORPIN, output);
            serialData.acc[0] = acc[0];
            serialData.acc[1] = acc[1];
            serialData.acc[2] = acc[2];
            serialData.gyro[0] = gyro[0];
            serialData.gyro[1] = gyro[1];
            serialData.gyro[2] = gyro[2];
            serialData.t = micros();
            serialData.dt = serialData.t - t_start;
            Serial.write((char *)&serialData, sizeof(serialData));

/*            Serial.print("\tch1:");
            Serial.print(width[0]);
            Serial.print("\tch2:");
            Serial.print(width[1]);
            Serial.print("\tch3:");
            Serial.print(width[2]);
            Serial.print("\tch4:");
            Serial.print(width[3]);
            Serial.print("\tch5:");
            Serial.print(width[4]);
            Serial.print("\tch6:");
            Serial.print(width[5]);
            Serial.print("\tmotor:");
            Serial.print(output);
            
            Serial.println("ax: ");
            Serial.println(acc[0]);
            Serial.print("\tay: ");
            Serial.print(acc[1]);
            Serial.print("\taz: ");
            Serial.print(acc[2]);
            Serial.print("\tgx: ");
            Serial.print(gyro[0]);
            Serial.print("\tgy: ");
            Serial.print(gyro[1]);
            Serial.print("\tgz: ");
            Serial.print(gyro[2]);
            Serial.print("\tdt: ");
            Serial.print(micros() - t_start);
            Serial.println("");
            */
            t_start = micros();
            
    
    }
}

