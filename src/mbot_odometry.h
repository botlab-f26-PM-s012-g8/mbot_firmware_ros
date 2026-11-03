#ifndef MBOT_ODOMETRY_H
#define MBOT_ODOMETRY_H

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/* Velocity-based odometry (replaced by tick-based versions)
void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta);
void mbot_calculate_gyrodometry(float vx, float vy, float wz, float dt, float gyro_z, float* x, float* y, float* theta);
*/

/**
 * @brief Calculate odometry from encoder tick deltas
 *
 * @param dticks_l Change in left wheel encoder ticks
 * @param dticks_r Change in right wheel encoder ticks
 * @param x Current x position (will be updated)
 * @param y Current y position (will be updated)
 * @param theta Current orientation (will be updated)
 */
void mbot_calculate_odometry_ticks(int32_t dticks_l, int32_t dticks_r, float* x, float* y, float* theta);

/**
 * @brief Calculate gyrodometry from encoder tick deltas and gyro readings
 *
 * @param dticks_l Change in left wheel encoder ticks
 * @param dticks_r Change in right wheel encoder ticks
 * @param dt Measured time step in seconds (used to integrate the gyro rate)
 * @param gyro_z Gyro z angular rate in rad/s
 * @param x Current x position (will be updated)
 * @param y Current y position (will be updated)
 * @param theta Current orientation (will be updated)
 */
void mbot_calculate_gyrodometry_ticks(int32_t dticks_l, int32_t dticks_r, float dt, float gyro_z, float* x, float* y, float* theta);

#ifdef __cplusplus
}
#endif

#endif // MBOT_ODOMETRY_H