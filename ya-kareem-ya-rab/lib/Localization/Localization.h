#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <Arduino.h>
#include <math.h>

class Localization {
private:
    const float wheel_radius;
    const float wheel_base;
    const int right_ticks_per_rev;
    const int left_ticks_per_rev;

    float right_distance_per_tick;
    float left_distance_per_tick;

    float x = 0.0;
    float y = 0.0;
    float theta = 0.0;

    long prev_left_ticks = 0;
    long prev_right_ticks = 0;
    float prev_imu_heading = 0.0;

public:
    Localization(float wheelRadius, float wheelBase, int rightTicksPerRev, int leftTicksPerRev);

    void update(long left_ticks, long right_ticks, float imu_heading);

    float getX() const;
    float getY() const;
    float getHeading() const;
};

#endif // LOCALIZATION_H
