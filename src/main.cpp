#include "WProgram.h"

/* Used for interrupts */
#include "core_pins.h"
#include "i2c_t3.h"
#include "PID.h"
#include "I2Cdev.h"
#include "MPU9150.h"
#include "sensor_fusion.h"
#include "mix.h"
#include "esc_control.h"

#define FIXEDPT_WBITS 4
#include "fixedptc.h"

#define RADIO_PINS 6

#define SENSOR_ADDRESS 0x68

#define X 0
#define Y 1
#define Z 2
#define ROLL 0
#define PITCH 1
#define THROTTLE 2
#define YAW 3
#define AUX1 4
#define AUX2 5

const static uint8_t RADIOPIN[RADIO_PINS] = {3,4,5,6,7,8};
/* Make sure the RISE pin is on a failsafe signal i.e throttle */
const static uint8_t RADIOPIN_RISE = 2;


volatile uint32_t radio_rise = 0;
volatile uint32_t width[6];

int16_t acc[3];
int16_t gyro[3];
int16_t mag[3];
int16_t acc_offset[3];
int16_t gyro_offset[3];

struct serialData {
    int16_t acc[3];
    int16_t gyro[3];
    uint32_t t;
    uint32_t dt;
    float pitch;
    float roll;
    float yaw;
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

void sendserialData(uint32_t t, uint32_t dt) {
    serialData.acc[0] = acc[0];
    serialData.acc[1] = acc[1];
    serialData.acc[2] = acc[2];
    serialData.gyro[0] = gyro[0];
    serialData.gyro[1] = gyro[1];
    serialData.gyro[2] = gyro[2];
    serialData.t = t;
    serialData.dt = dt;
    if(Serial.dtr()) {
        Serial.write(1);
        Serial.write((char *)&serialData, sizeof(serialData));
        Serial.write(2);
        Serial.send_now();
    }
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



uint16_t read_16bit_register(uint8_t high) {
    uint16_t result;
    Wire.beginTransmission(SENSOR_ADDRESS);
    Wire.write(high);
    Wire.endTransmission(I2C_STOP);
    Wire.requestFrom(SENSOR_ADDRESS, 1, I2C_STOP);
    result = (Wire.readByte() << 8);
    
    Wire.beginTransmission(SENSOR_ADDRESS);
    Wire.write(high+1);
    Wire.endTransmission(I2C_STOP);
    Wire.requestFrom(SENSOR_ADDRESS, 1, I2C_STOP);
    result |= Wire.readByte();
    return result;
}


void read_sensors9() {
    mpu9150.getMotion9(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2], &mag[X], &mag[Y], &mag[Z]);
    gyro[X] = gyro[X] - gyro_offset[X];
    gyro[Y] = gyro[Y] - gyro_offset[Y];
    gyro[Z] = gyro[Z] - gyro_offset[Z];
}

void read_sensors6() {
    mpu9150.getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
    gyro[X] = gyro[X] - gyro_offset[X];
    gyro[Y] = gyro[Y] - gyro_offset[Y];
    gyro[Z] = gyro[Z] - gyro_offset[Z];
}
bool read_sensors() {
	static uint32_t t_prevmagread = 0;
	uint32_t t = micros();
	if(t-t_prevmagread < 125000) {
		read_sensors6();
		return false;
	} else {
		read_sensors9();
		t_prevmagread = t;
		return true;
	}

}


void setup_mpu() {
    Wire.begin(I2C_MASTER,0x0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    delay(1000); 
    mpu9150.initialize();
    
    read_sensors9();
    gyro_offset[X] = gyro[X];
    gyro_offset[Y] = gyro[Y];
    gyro_offset[Z] = gyro[Z];

}





extern "C" int main(void)
{
    setup_radio();
    setup_mpu();
	pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    uint32_t h = 1000;
    PID rollPID(h);
    PID pitchPID(h);
    PID yawPID(h);
    
    esc_control motors;

    complementary_filter sensor_fusion;
    read_sensors();
    sensor_fusion.calibrateAngle(acc[X], acc[Y], acc[Z]);

    uint32_t t_start = micros();
    uint32_t t_end = t_start;
    uint32_t dt = 0;
	while (1) {
            digitalWrite(13, HIGH);
            read_sensors6();
            sensor_fusion.update(gyro[X]/250.0l, gyro[Y]/250.0l, gyro[Z]/250.0l, acc[X], acc[Y], acc[Z], h/1000000.0l);
            double roll = sensor_fusion.getRoll();
            double pitch = sensor_fusion.getPitch();
            double yaw = sensor_fusion.getYaw();

            //Normalize reference
            double rroll = (width[ROLL] - 1500) / 500.0l;
            double rpitch = (width[PITCH] - 1500) / 500.0l;
            double ryaw = (width[YAW] - 1500) / 500.0l;
            double rthrottle = (width[THROTTLE] - 1000) / 500.0l;

            double croll = rollPID.calculateOutput(rroll, roll);
            double cpitch = pitchPID.calculateOutput(rpitch, pitch);
            double cyaw = yawPID.calculateOutput(ryaw, yaw);

            double output[4];
            mix(rthrottle, cpitch, croll, cyaw, output);

            motors.output(output);



            //TODO: Fix proper output limitation
            rollPID.updateState(croll);
            pitchPID.updateState(cpitch);
            yawPID.updateState(cyaw);

            serialData.roll = roll;
            serialData.pitch = pitch;
            serialData.yaw = yaw;
            sendserialData(t_end, dt);

            t_end = micros();
            dt = t_end - t_start;
            digitalWrite(13, LOW);
            /* Fixed sample rate */
            while(micros() - t_start < h);
            t_start = micros();
    }
}

