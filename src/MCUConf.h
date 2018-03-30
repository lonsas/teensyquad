#ifndef ECU_CONF_H
#define ECU_CONF_H

#define SAMPLE_TIME 1000
#define SAMPLE_TIME_S (SAMPLE_TIME/1000000.0)

#define STARTUP_TIME 10000

#define ANGLE_TOL 0.1
#define OMEGA_TOL 0.1

#define USB_VOLT_PIN 19


/* EEPROM addresses */
#define PID_ROLL_OMEGA_ADDR (void*)0
#define PID_PITCH_OMEGA_ADDR (void*)32
#define PID_YAW_OMEGA_ADDR (void*)64

#define PID_ROLL_ANGLE_ADDR (void*)96
#define PID_PITCH_ANGLE_ADDR (void*)128
#define PID_YAW_ANGLE_ADDR (void*)150

#endif /* ECU_CONF_H */
