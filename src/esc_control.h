/*
 * esc_control.h
 */
#include "inttypes.h"

#ifndef SRC_ESC_CONTROL_H_
#define SRC_ESC_CONTROL_H_

#define MOTOR_ARM 960
#define MOTOR_OFF 960
#define MOTOR_MAX 2000
#define MOTOR_MIN 0.04

#define MOTOR0_PIN ((uint8_t)23)
#define MOTOR1_PIN ((uint8_t)22)
#define MOTOR2_PIN ((uint8_t)21)
#define MOTOR3_PIN ((uint8_t)20)

#define PWM_RES 12
#define PWM_RATE 400
#define SCALING (PWM_RATE*(1<<(int32_t)PWM_RES)/1000000.0)

class esc_control {
private:
    bool armed;
    void writeMicros(uint8_t, int);
public:
    esc_control();
    void output(double *);
    void arm();
    bool is_armed();
    void off();
};

#endif /* SRC_ESC_CONTROL_H_ */
