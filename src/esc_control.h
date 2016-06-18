/*
 * esc_control.h
 */

#ifndef SRC_ESC_CONTROL_H_
#define SRC_ESC_CONTROL_H_

#define ARM_PWM 960
#define MOTOR_OFF 960
#define MOTOR_MAX 2000

#define MOTOR0_PIN ((uint8_t)23)
#define MOTOR1_PIN ((uint8_t)24)
#define MOTOR2_PIN ((uint8_t)25)
#define MOTOR3_PIN ((uint8_t)26)

#define PWM_RES 12
#define PWM_RATE 400

class esc_control {
private:
    bool armed;
public:
    esc_control();
    void output(double *);
    void arm(bool);
    bool is_armed();
};

#endif /* SRC_ESC_CONTROL_H_ */
