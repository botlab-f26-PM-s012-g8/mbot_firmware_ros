#include "mbot_odometry.h"
#include <math.h>
#include <mbot/defs/mbot_params.h>
#include "config/mbot_classic_config.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define GYRODOM_THRESHOLD 0.0005f   // ~0.05 rad/s difference at 100 Hz

extern mbot_params_t params;

/* Velocity-based odometry (replaced by tick-based versions below)
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
*/

// Convert encoder tick deltas to forward distance and heading change of the body
static void mbot_ticks_to_body_delta(int32_t dticks_l, int32_t dticks_r, float* ds, float* dtheta) {
    const float rad_per_tick = 2.0f * PI / (float)(GEAR_RATIO * ENCODER_RES);
    float dl = DIFF_L_WHEEL_RADIUS * params.encoder_polarity[MOT_L] * dticks_l * rad_per_tick;
    float dr = DIFF_R_WHEEL_RADIUS * params.encoder_polarity[MOT_R] * dticks_r * rad_per_tick;
    *ds = (dl - dr) / 2.0f;  // Forward distance traveled (right wheel is mirrored)
    *dtheta = (-dl - dr) / (2.0f * DIFF_BASE_RADIUS);  // Change in orientation
}

// Update pose using the midpoint heading of the step, then normalize theta to [-pi, pi]
static void mbot_update_pose(float ds, float dtheta, float* x, float* y, float* theta) {
    float theta_mid = *theta + 0.5f * dtheta;
    *x += ds * cosf(theta_mid);
    *y += ds * sinf(theta_mid);
    *theta += dtheta;

    while (*theta > PI) *theta -= 2.0f * PI;
    while (*theta <= -PI) *theta += 2.0f * PI;
}

void mbot_calculate_odometry_ticks(int32_t dticks_l, int32_t dticks_r, float* x, float* y, float* theta) {
    float ds, dtheta;
    mbot_ticks_to_body_delta(dticks_l, dticks_r, &ds, &dtheta);
    mbot_update_pose(ds, dtheta, x, y, theta);
}

void mbot_calculate_gyrodometry_ticks(int32_t dticks_l, int32_t dticks_r, float dt, float gyro_z, float* x, float* y, float* theta) {
    float ds, delta_theta_wheel;
    mbot_ticks_to_body_delta(dticks_l, dticks_r, &ds, &delta_theta_wheel);
    float delta_theta_gyro = gyro_z * dt;  // Change in orientation from gyroscope
    // Trust the gyro when it disagrees with the wheels (e.g. slip), otherwise use the wheels
    float delta_theta = (fabsf(delta_theta_gyro - delta_theta_wheel) > GYRODOM_THRESHOLD) ? delta_theta_gyro : delta_theta_wheel;
    mbot_update_pose(ds, delta_theta, x, y, theta);
}
