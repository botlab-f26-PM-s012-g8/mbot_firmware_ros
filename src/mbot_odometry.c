#include "mbot_odometry.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define GYRODOM_THRESHOLD 0.001f   // ~0.1 rad/s difference at 100 Hz

void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta) {
    // Update pose
    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);
    *theta += wz * dt;

    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;
}

void mbot_calculate_gyrodometry(float vx, float vy, float wz, float dt, float gyro_z, float* x, float* y, float* theta) {
    // Update pose using gyroscope data for orientation
    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);
    float delta_theta_gyro = gyro_z * dt;  // Change in orientation from gyroscope
    float delta_theta_wheel = wz * dt;  // Change in orientation from wheel odometry
    float delta_theta = (fabsf(delta_theta_gyro - delta_theta_wheel) > GYRODOM_THRESHOLD) ? delta_theta_gyro : delta_theta_wheel;
    *theta += delta_theta;  // Use the more reliable delta_theta for orientation update

    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;
}