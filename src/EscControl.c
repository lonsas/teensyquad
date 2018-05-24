/*
 * EscControl.c
 */

#include "EscControl.h"
#include "core_pins.h"


static bool m_boArmed;

static void writeMicros(uint8_t pin, double micros) {
    analogWrite(pin, micros*SCALING);
}

void EscControlSetup() {

    pinMode(MOTOR0_PIN, OUTPUT);
    analogWriteFrequency(MOTOR0_PIN, PWM_RATE);

    pinMode(MOTOR1_PIN, OUTPUT);
    analogWriteFrequency(MOTOR1_PIN, PWM_RATE);

    pinMode(MOTOR2_PIN, OUTPUT);
    analogWriteFrequency(MOTOR2_PIN, PWM_RATE);

    pinMode(MOTOR3_PIN, OUTPUT);
    analogWriteFrequency(MOTOR3_PIN, PWM_RATE);

    analogWriteResolution(PWM_RES);

    EscControlDisarm();
}

void EscControlOutput(double *motor) {
    double motor0_pwm, motor1_pwm, motor2_pwm, motor3_pwm;
    if(m_boArmed) {
        if(motor[0] <= MOTOR_MIN) {
            motor0_pwm = MOTOR_MICROS_MIN;
        } else if (motor[0] > 1) {
            motor0_pwm = MOTOR_MICROS_MAX;
        } else {
            motor0_pwm = motor[0] * 1000+1000;
        }


        if(motor[1] <= MOTOR_MIN) {
            motor1_pwm = MOTOR_MICROS_MIN;
        } else if (motor[1] > 1) {
            motor1_pwm = MOTOR_MICROS_MAX;
        } else {
            motor1_pwm = motor[1] * 1000+1000;
        }

        if(motor[2] <= MOTOR_MIN) {
            motor2_pwm = MOTOR_MICROS_MIN;
        } else if (motor[2] > 1) {
            motor2_pwm = MOTOR_MICROS_MAX;
        } else {
            motor2_pwm = motor[2] * 1000+1000;
        }

        if(motor[3] <= MOTOR_MIN) {
            motor3_pwm = MOTOR_MICROS_MIN;
        } else if (motor[3] > 1) {
            motor3_pwm = MOTOR_MICROS_MAX;
        } else {
            motor3_pwm = motor[3] * 1000+1000;
        }
    } else {
        motor0_pwm = motor1_pwm = motor2_pwm = motor3_pwm = MOTOR_MICROS_OFF;
    }

    writeMicros(MOTOR0_PIN, motor0_pwm);
    writeMicros(MOTOR1_PIN, motor1_pwm);
    writeMicros(MOTOR2_PIN, motor2_pwm);
    writeMicros(MOTOR3_PIN, motor3_pwm);
}

void EscControlArm() {
    writeMicros(MOTOR0_PIN, MOTOR_MICROS_ARM);
    writeMicros(MOTOR1_PIN, MOTOR_MICROS_ARM);
    writeMicros(MOTOR2_PIN, MOTOR_MICROS_ARM);
    writeMicros(MOTOR3_PIN, MOTOR_MICROS_ARM);
    m_boArmed = true;
}

bool EscControlIsArmed() {
        return m_boArmed;
}

void EscControlDisarm() {
    writeMicros(MOTOR0_PIN, MOTOR_MICROS_OFF);
    writeMicros(MOTOR1_PIN, MOTOR_MICROS_OFF);
    writeMicros(MOTOR2_PIN, MOTOR_MICROS_OFF);
    writeMicros(MOTOR3_PIN, MOTOR_MICROS_OFF);
    m_boArmed = false;
}

