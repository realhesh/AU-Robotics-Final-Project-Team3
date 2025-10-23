#include <math.h>
#include "Localization.h"
using namespace std;

    Localization::Localization(float wheelRadius, float wheelBase, int rightTicksPerRev, int leftTicksPerRev)
        : wheel_radius(wheelRadius),
          wheel_base(wheelBase),
          right_ticks_per_rev(rightTicksPerRev),
          left_ticks_per_rev(leftTicksPerRev)
    {
        right_distance_per_tick = 2 * M_PI * wheel_radius / right_ticks_per_rev;
        left_distance_per_tick = 2 * M_PI * wheel_radius / left_ticks_per_rev;
    }

    void Localization::update(long left_ticks, long right_ticks, float imu_heading)
    {
        float left_dist = left_distance_per_tick * (left_ticks - prev_left_ticks);
        float right_dist = right_distance_per_tick * (right_ticks - prev_right_ticks);
        float center_dist = (left_dist + right_dist) / 2.0;
        // float delta_theta = (right_dist - left_dist) / 2.0;

        // apply kalman filter and get esmited_heading and use it instead
        // x += center_dist * sin(estimated_heading);
        // y += center_dist * cos(estimated_heading);

        x += center_dist * sin(imu_heading);
        y += center_dist * cos(imu_heading);

        // x += center_dist * sin(delta_theta / 2.0);
        // y += center_dist * cos(delta_theta / 2.0);
        // theta += delta_theta;
        // while(theta > M_PI) theta -= 2*M_PI;
        // while(theta < -M_PI)theta += 2*M_PI;

        prev_left_ticks = left_ticks;
        prev_right_ticks = right_ticks;
        prev_imu_heading = imu_heading;
    }

    float Localization::getX() const { return x; }
    float Localization::getY() const { return y; }
    float Localization::getHeading() const { return prev_imu_heading; }
    // float getTheta() const { return theta; }
