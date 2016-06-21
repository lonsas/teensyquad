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
#include "MadgwickAHRS.h"

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

#define GYROMAX 250.0
#define GYROSCALE (GYROMAX/(1<<15))

static inline bool isHigh(double x) {
    return(x > 0.9);
}
static inline bool isCenter(double x) {
    return(x > -0.1 && x < 0.1);
}
static inline bool isLow(double x) {
    return(x < -0.9);
}
static inline bool isOff(double x) {
    return(x < -1);
}

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

double pitch_offset;
double roll_offset;
void calibrateAngle(int ax, int ay, int az) {
    pitch_offset = atan2(ay, az)/PI;
    roll_offset = atan2(ax, az)/PI;

}
double gx_offset;
double gy_offset;
double gz_offset;
void calibrateGyro(int gx, int gy, int gz) {
    gx_offset = gx;
    gy_offset = gy;
    gz_offset = gz;

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

void blink(int n, int time) {
    for(int i = 0; i < n; i++) {
        digitalWrite(13, HIGH);
        delay(time);
        digitalWrite(13, LOW);
        delay(time);
    }

}

extern "C" int main(void)
{
    setup_radio();
    setup_mpu();
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    uint32_t h = 1000;
    double hseconds = h/1000000.0;
    PID rollPID(hseconds);
    PID pitchPID(hseconds);
    PID yawPID(hseconds);

    esc_control motors;
    motors.arm();

    Madgwick sensor_fusion;
    read_sensors();
    sensor_fusion.begindt(hseconds);

    uint32_t t_start = micros();
    uint32_t t_end = t_start;
    uint32_t dt = 0;
    bool armed = false;

    double ryaw = 0;
    while (1) {
        //Normalize reference
        //TODO: Error checking on signals
        double ch[RADIO_PINS];
        ch[ROLL] = (width[ROLL] - 1500) / 500.0;
        ch[PITCH] = (width[PITCH] - 1500) / 500.0;
        ch[YAW] = (width[YAW] - 1500) / 500.0;
        ch[THROTTLE] = (width[THROTTLE] - 1500) / 500.0;
        ch[AUX1] = (width[AUX1] - 1500) / 500.0;
        ch[AUX2] = (width[AUX2] - 1500) / 500.0;
//        armed = true;
        if(armed) {
            double roll;
            double pitch;
            double yaw;

            double rroll;
            double rpitch;

            double rthrottle;

            double croll;
            double cpitch;
            double cyaw;

            digitalWrite(13, HIGH);
            read_sensors6();
            sensor_fusion.updateIMU(gyro[X]*GYROSCALE, gyro[Y]*GYROSCALE, gyro[Z]*GYROSCALE, acc[X], acc[Y], acc[Z]);
            roll = sensor_fusion.getRollRadians()/PI;
            pitch = sensor_fusion.getPitchRadians()/PI;
            yaw = sensor_fusion.getYawRadians()/PI;

            rthrottle = ch[THROTTLE]*2 - 1;
            rroll = ch[ROLL];
            rpitch = ch[PITCH];
            ryaw += ch[YAW]*hseconds;


            double output[4];
            //TODO: Implement air-mode where things still happen when throttle is off
            if(!isOff(ch[THROTTLE])) {
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
            serialData.data[0] = ryaw;
            serialData.data[1] = rpitch;
            serialData.data[2] = rroll;
            serialData.data[3] = 0;

            //Input logic
            if(isOff(ch[THROTTLE]) && isLow(ch[AUX1])) {
                armed = false;
            }
        } else { //Unarmed, wait input logic
            digitalWrite(13, LOW);

            /* Using gotos to make sure only one command is registered
             * This makes it easy so that we do not need to send/make global all objects
             * that possibly needs changing.
             * Sample time may be broken here for confirmation and more advanced inputs
             */

            // Arm
            if(isOff(ch[THROTTLE]) && isHigh(ch[AUX1])) {
                armed = true;
                rollPID.resetState();
                pitchPID.resetState();
                yawPID.resetState();

                blink(2, 100);
                goto INPUT_DONE;
            }

            // Calibrate
            if(isLow(ch[PITCH]) && isLow(ch[ROLL])) {
                read_sensors6();
                calibrateAngle(acc[X], acc[Y], acc[Z]);
                calibrateGyro(gyro[X], gyro[Y], gyro[Z]);


                blink(5, 100);
                goto INPUT_DONE;
            }


        }
        INPUT_DONE:




        sendserialData(t_end, dt);

        t_end = micros();
        dt = t_end - t_start;

        /* Fixed sample rate */
        while(micros() - t_start < h);
        t_start = micros();
    }
}

