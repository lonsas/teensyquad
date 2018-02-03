/*
 * esc_control.cpp
 */

#include "esc_control.h"
#include "core_pins.h"


esc_control::esc_control() {
    armed = true;

    pinMode(MOTOR0_PIN, OUTPUT);
    analogWriteFrequency(MOTOR0_PIN, PWM_RATE);

    pinMode(MOTOR1_PIN, OUTPUT);
    analogWriteFrequency(MOTOR1_PIN, PWM_RATE);

    pinMode(MOTOR2_PIN, OUTPUT);
    analogWriteFrequency(MOTOR2_PIN, PWM_RATE);

    pinMode(MOTOR3_PIN, OUTPUT);
    analogWriteFrequency(MOTOR3_PIN, PWM_RATE);

    analogWriteResolution(PWM_RES);
}

void esc_control::output(double *motor) {
    int32_t motor0_pwm, motor1_pwm, motor2_pwm, motor3_pwm;
    if(armed) {
        if(motor[0] <= MOTOR_MIN) {
            motor0_pwm = MOTOR_OFF;
        } else if (motor[0] > 1) {
            motor0_pwm = MOTOR_MAX;
        } else {
            motor0_pwm = motor[0] * 1000+1000;
        }


        if(motor[1] <= MOTOR_MIN) {
            motor1_pwm = MOTOR_OFF;
        } else if (motor[1]> 1) {
            motor1_pwm = MOTOR_MAX;
        } else {
            motor1_pwm = motor[1] * 1000+1000;
        }

        if(motor[2] <= MOTOR_MIN) {
            motor2_pwm = MOTOR_OFF;
        } else if (motor[2] > 1) {
            motor2_pwm = MOTOR_MAX;
        } else {
            motor2_pwm = motor[2] * 1000+1000;
        }

        if(motor[3] <= 0) {
            motor3_pwm = MOTOR_OFF;
        } else if (motor[3] > 1) {
            motor3_pwm = MOTOR_MAX;
        } else {
            motor3_pwm = motor[3] * 1000+1000;
        }
    } else {
        motor0_pwm = motor1_pwm = motor2_pwm = motor3_pwm = MOTOR_OFF;
    }

    writeMicros(MOTOR0_PIN, motor0_pwm);
    writeMicros(MOTOR1_PIN, motor1_pwm);
    writeMicros(MOTOR2_PIN, motor2_pwm);
    writeMicros(MOTOR3_PIN, motor3_pwm);
}

void esc_control::arm() {
    writeMicros(MOTOR0_PIN, MOTOR_ARM);
    writeMicros(MOTOR1_PIN, MOTOR_ARM);
    writeMicros(MOTOR2_PIN, MOTOR_ARM);
    writeMicros(MOTOR3_PIN, MOTOR_ARM);
    delay(1000);
    armed = true;
}

bool esc_control::is_armed() {
        return armed;
}

void esc_control::off() {
    writeMicros(MOTOR0_PIN, MOTOR_OFF);
    writeMicros(MOTOR1_PIN, MOTOR_OFF);
    writeMicros(MOTOR2_PIN, MOTOR_OFF);
    writeMicros(MOTOR3_PIN, MOTOR_OFF);
}

void esc_control::writeMicros(uint8_t pin, int micros) {
    analogWrite(pin, micros*SCALING);
}
