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

#define GYROSCALE (250.0/(1<<15))

const static uint8_t RADIOPIN[RADIO_PINS] = {3,4,5,6,7,8};
/* Make sure the RISE pin is on a failsafe signal i.e throttle */
const static uint8_t RADIOPIN_RISE = 2;


volatile uint32_t radio_rise = 0;
volatile int32_t width[6] = {1500, 1500, 1000, 1500, 1500, 1500};

int16_t acc[3];
int16_t gyro[3];
int16_t mag[3];

struct serialData {
    float data[6];
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
}

void read_sensors6() {
    mpu9150.getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
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

    read_sensors6();
}





extern "C" int main(void)
{
    setup_radio();
    setup_mpu();
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    uint32_t h = 1000;
    PID rollPID(h/1000000.0);
    PID pitchPID(h/1000000.0);
    PID yawPID(h/1000000.0);

    esc_control motors;
    motors.arm();

    complementary_filter sensor_fusion;
    read_sensors();
    sensor_fusion.calibrateAngle(acc[X], acc[Y], acc[Z]);

    uint32_t t_start = micros();
    uint32_t t_end = t_start;
    uint32_t dt = 0;
    bool armed = false;
    bool throttle_off = true;
    while (1) {

        armed = true;
        if(armed) {
            double roll;
            double pitch;
            double yaw;

            double rroll;
            double rpitch;
            double ryaw;
            double rthrottle;

            double croll;
            double cpitch;
            double cyaw;

            digitalWrite(13, HIGH);
            read_sensors6();
            sensor_fusion.update(gyro[X]*GYROSCALE, gyro[Y]*GYROSCALE, gyro[Z]*GYROSCALE, acc[X], acc[Y], acc[Z]);
            roll = sensor_fusion.getRoll();
            pitch = sensor_fusion.getPitch();
            yaw = sensor_fusion.getYaw();

            //Normalize reference
            //TODO: Error checking on signals
            rroll = (width[ROLL] - 1500) / 500.0;
            rpitch = (width[PITCH] - 1500) / 500.0;
            ryaw = (width[YAW] - 1500) / 500.0;
            rthrottle = (width[THROTTLE] - 1000) / 1000.0;

            double output[4];
            if(!throttle_off) {
                croll = rollPID.calculateOutput(rroll, roll);
                cpitch = pitchPID.calculateOutput(rpitch, pitch);
                cyaw = yawPID.calculateOutput(ryaw, yaw);


                mix(rthrottle, &cpitch, &croll, &cyaw, output);


                //TODO: Fix proper output limitation
                rollPID.updateState(croll);
                pitchPID.updateState(cpitch);
                yawPID.updateState(cyaw);
                motors.output(output);
            } else {
                motors.off();
            }



            serialData.roll = pitch;
            serialData.pitch = roll;
            serialData.yaw = yaw;
            serialData.data[0] = output[0];
            serialData.data[1] = output[1];
            serialData.data[2] = output[2];
            serialData.data[3] = output[3];
        } else {
            digitalWrite(13, LOW);
        }

        //Throttle off
        if(width[THROTTLE] < 1000) {
            if(width[AUX1] > 1800) {
                if(!armed) {
                    armed = true;
                    read_sensors6();
                    sensor_fusion.calibrateAngle(acc[X], acc[Y], acc[Z]);
                    sensor_fusion.calibrateGyro(gyro[X], gyro[Y], gyro[Z]);
                    sensor_fusion.reset(acc[X], acc[Y], acc[Z]);
                    rollPID.resetState();
                    pitchPID.resetState();
                    yawPID.resetState();

                }
            } else {
                armed = false;
                motors.off();
            }
            throttle_off = true;
        } else {
            throttle_off = false;
        }

        sendserialData(t_end, dt);

        t_end = micros();
        dt = t_end - t_start;

        /* Fixed sample rate */
        while(micros() - t_start < h);
        t_start = micros();
    }
}

