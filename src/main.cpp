#include "WProgram.h"

/* Used for interrupts */
#include "core_pins.h"
#include "i2c_t3.h"


#define RADIO_PINS 6

#define SENSOR_ADDRESS 0x68

const static uint8_t RADIOPIN[RADIO_PINS] = {3,4,5,6,7,8};
/* Make sure the RISE pin is on a failsafe signal i.e throttle */
const static uint8_t RADIOPIN_RISE = 2;
const static uint8_t MOTORPIN = 23;

volatile uint32_t radio_rise = 0;
volatile uint32_t width[6];


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

int16_t sensor_test() {
    int16_t result;
    Wire.beginTransmission(SENSOR_ADDRESS);
    Wire.write(0x3b);
    Wire.endTransmission();
    Wire.requestFrom(SENSOR_ADDRESS, 1, I2C_NOSTOP);
    result = (Wire.readByte() << 8);

    Wire.beginTransmission(SENSOR_ADDRESS);
    Wire.write(0x3c);
    Wire.endTransmission();
    Wire.requestFrom(SENSOR_ADDRESS, 1, I2C_NOSTOP);
    result |= Wire.readByte();
    return result;
}

void setup_sensor() {
    delay(1000);
    Wire.begin(I2C_MASTER,0x0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);
    Wire.beginTransmission(SENSOR_ADDRESS);
    Wire.write(0x6b);
    Wire.write(0);
    Wire.endTransmission(I2C_STOP);
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
            i++;
            output = width[2]*1.6384;
            analogWrite(MOTORPIN, output);
            Serial.print(width[0]);
            Serial.print(" ");
            Serial.print(width[1]);
            Serial.print(" ");
            Serial.print(width[2]);
            Serial.print(" ");
            Serial.print(width[3]);
            Serial.print(" ");
            Serial.print(width[4]);
            Serial.print(" ");
            Serial.print(width[5]);
            Serial.print(" ");
            Serial.print(output);
            Serial.print(" ");
            Serial.print(sensor_test());
            Serial.print(" ");
            Serial.print(micros() - t_start);
            Serial.println("");
            t_start = micros();

            
    
    }
}

