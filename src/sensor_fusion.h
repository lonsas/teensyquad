#ifndef __SENSOR_FUSION_H
#define __SENSOR_FUSION_H


#include "inttypes.h"

#define X 0
#define Y 1
#define Z 2



typedef struct {
    int16_t acc[3];
    int16_t gyro[3];
/*    int16_t mag[3];*/
} SensorData;


double calculateRoll(SensorData *);

class complementary_filter {
private:
    int16_t acc[3];
    int16_t gyro[3];
    double roll;
    double pitch;
    double yaw;
    double aroll;
    double apitch;
    double ayaw;
    double pitch_offset;
    double roll_offset;
    double yaw_offset;
public:
    complementary_filter(void);
    void begin(uint32_t dt);
    void update(double gx, double gy, double gz, double ax, double ay, double az, double dt);
    double getRoll() {
        return roll;
    }
    double getPitch() {
        return pitch;
    }
    double getYaw() {
        return yaw;
    }
    void calibrateAngle(double ax, double ay, double az);
};
#endif

