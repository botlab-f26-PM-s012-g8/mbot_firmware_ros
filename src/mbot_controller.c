#include "mbot_controller.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <mbot/defs/mbot_params.h>
#include "config/mbot_classic_config.h"
#include "config/mbot_classic_default_pid.h"
#include <rc/math/filter.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <mbot/fram/fram.h>

// Access to global params struct defined in mbot_classic_ros.c
extern mbot_params_t params;

// Global PID controller variables
mbot_pid_config_t pid_gains = MBOT_DEFAULT_PID_GAINS;
control_mode_t control_mode = (control_mode_t)MBOT_DEFAULT_CONTROL_MODE;

static bool pid_updated = false;

void mbot_read_pid_gains(const mbot_params_t* params) {
    pid_gains.left_wheel.kp  = params->left_wheel_vel_pid[0];
    pid_gains.left_wheel.ki  = params->left_wheel_vel_pid[1];
    pid_gains.left_wheel.kd  = params->left_wheel_vel_pid[2];
    pid_gains.left_wheel.tf  = params->left_wheel_vel_pid[3];

    pid_gains.right_wheel.kp = params->right_wheel_vel_pid[0];
    pid_gains.right_wheel.ki = params->right_wheel_vel_pid[1];
    pid_gains.right_wheel.kd = params->right_wheel_vel_pid[2];
    pid_gains.right_wheel.tf = params->right_wheel_vel_pid[3];

    control_mode = (control_mode_t)params->control_mode;
}

// Wheel PID state. The integral is stored separately from the output so that
// clamping the output can't corrupt it (rc_filter's difference equation can't do this).
typedef struct {
    float integral;
    float deriv;
    float prev_vel;
    bool has_prev_vel;
    float min_out;
    float max_out;
} wheel_pid_t;

static wheel_pid_t left_wheel_pid;
static wheel_pid_t right_wheel_pid;
rc_filter_t body_vel_vx_pid;
rc_filter_t body_vel_wz_pid;

static float clamp_pid(float val, float min_val, float max_val) {
    return fminf(fmaxf(val, min_val), max_val);
}

static void wheel_pid_reset(wheel_pid_t* pid) {
    pid->integral = 0.0f;
    pid->deriv = 0.0f;
    pid->prev_vel = 0.0f;
    pid->has_prev_vel = false;
}

static float wheel_pid_march(wheel_pid_t* pid, const pid_params_t* gains,
                             float target_vel, float current_vel) {
    float error = target_vel - current_vel;
    float p_term = gains->kp * error;

    // Filtered derivative on measurement, so a target step gives no derivative kick
    if (gains->tf > MAIN_LOOP_PERIOD / 2.0f) {
        if (pid->has_prev_vel) {
            pid->deriv = (1.0f - MAIN_LOOP_PERIOD / gains->tf) * pid->deriv
                         - (gains->kd / gains->tf) * (current_vel - pid->prev_vel);
        }
    } else {
        pid->deriv = 0.0f;
    }
    pid->prev_vel = current_vel;
    pid->has_prev_vel = true;

    // Limits move with the feedforward, so keep the stored integral inside them
    pid->integral = clamp_pid(pid->integral, pid->min_out, pid->max_out);

    // Anti-windup: don't integrate further into a saturated output
    float integral_step = gains->ki * MAIN_LOOP_PERIOD * error;
    float unclamped = p_term + pid->integral + integral_step + pid->deriv;
    bool pushing_past_max = (unclamped > pid->max_out) && (error > 0.0f);
    bool pushing_past_min = (unclamped < pid->min_out) && (error < 0.0f);
    if (!pushing_past_max && !pushing_past_min) {
        pid->integral = clamp_pid(pid->integral + integral_step, pid->min_out, pid->max_out);
    }

    return clamp_pid(p_term + pid->integral + pid->deriv, pid->min_out, pid->max_out);
}

int mbot_controller_init(void) {
    // Initialize PID controllers
    body_vel_vx_pid = rc_filter_empty();
    body_vel_wz_pid = rc_filter_empty();

    wheel_pid_reset(&left_wheel_pid);
    wheel_pid_reset(&right_wheel_pid);

    // Limit outputs between -1.0 and 1.0 until feedforward limits are set
    left_wheel_pid.min_out = -1.0f;
    left_wheel_pid.max_out = 1.0f;
    right_wheel_pid.min_out = -1.0f;
    right_wheel_pid.max_out = 1.0f;

    return MBOT_OK;
}

void mbot_motor_vel_controller(float target_left_vel, float target_right_vel,
                              float current_left_vel, float current_right_vel,
                              float* left_correction, float* right_correction) {
    // Run PID controllers for each wheel
    *left_correction = wheel_pid_march(&left_wheel_pid, &pid_gains.left_wheel,
                                       target_left_vel, current_left_vel);
    *right_correction = wheel_pid_march(&right_wheel_pid, &pid_gains.right_wheel,
                                        target_right_vel, current_right_vel);
}

void mbot_motor_vel_controller_reset(void) {
    wheel_pid_reset(&left_wheel_pid);
    wheel_pid_reset(&right_wheel_pid);
}

// Limit PID output so that ff + polarity*pid stays within [-1, 1]
static void set_pid_limits(wheel_pid_t* pid, float ff, int polarity) {
    float a = polarity * (-1.0f - ff);
    float b = polarity * ( 1.0f - ff);
    pid->min_out = fminf(a, b);
    pid->max_out = fmaxf(a, b);
}

void mbot_motor_vel_controller_set_limits(float ff_left, int pol_left,
                                          float ff_right, int pol_right) {
    set_pid_limits(&left_wheel_pid, ff_left, pol_left);
    set_pid_limits(&right_wheel_pid, ff_right, pol_right);
}

int init_parameter_server(rclc_parameter_server_t* parameter_server, rcl_node_t* node) {
    rcl_ret_t ret;
    
    // Initialize parameter server with options for low memory mode
    rclc_parameter_options_t options = {
        .notify_changed_over_dds = false,
        .max_params = 9,  
        .allow_undeclared_parameters = false,
        .low_mem_mode = false
    };
    
    ret = rclc_parameter_server_init_with_option(parameter_server, node, &options);
    if (ret != RCL_RET_OK) {
        printf("[FATAL] Failed to init parameter server: %d\n", ret);
        return MBOT_ERROR;
    }

    // Add parameters for all PID gains
    ret = rclc_add_parameter(parameter_server, "left_wheel.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "left_wheel.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "left_wheel.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "left_wheel.tf", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_add_parameter(parameter_server, "right_wheel.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "right_wheel.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "right_wheel.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "right_wheel.tf", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_add_parameter(parameter_server, "control_mode", RCLC_PARAMETER_INT);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    // Set initial values for all PID gains from pid_gains
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.kp", pid_gains.left_wheel.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.ki", pid_gains.left_wheel.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.kd", pid_gains.left_wheel.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.tf", pid_gains.left_wheel.tf);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_parameter_set_double(parameter_server, "right_wheel.kp", pid_gains.right_wheel.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "right_wheel.ki", pid_gains.right_wheel.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "right_wheel.kd", pid_gains.right_wheel.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "right_wheel.tf", pid_gains.right_wheel.tf);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    ret = rclc_parameter_set_int(parameter_server, "control_mode", control_mode);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    return MBOT_OK;
}

bool parameter_callback(const Parameter * old_param, const Parameter * new_param, void * context) {
    if (new_param == NULL) {
        // Parameter deletion not allowed
        return false;
    }

    const char* param_name = new_param->name.data;

    if (strcmp(param_name, "left_wheel.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.kp = new_param->value.double_value;
            pid_updated = true;
        }
    } else if (strcmp(param_name, "left_wheel.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.ki = new_param->value.double_value;
            pid_updated = true;
        }
    } else if (strcmp(param_name, "left_wheel.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.kd = new_param->value.double_value;
            pid_updated = true;
        }
    } else if (strcmp(param_name, "left_wheel.tf") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.tf = new_param->value.double_value;
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.kp = new_param->value.double_value;
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.ki = new_param->value.double_value;
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.kd = new_param->value.double_value;
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.tf") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.tf = new_param->value.double_value;
            pid_updated = true;
        }
    } else if (strcmp(param_name, "control_mode") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_INT) {
            int val = new_param->value.integer_value;
            control_mode = (control_mode_t)val;
            pid_updated = true;
        }
    }

    if (pid_updated) {
        mbot_save_params_to_fram();
    }
    return pid_updated;
}

// Save current PID gains and control mode to FRAM
int mbot_save_params_to_fram(void) {
    params.left_wheel_vel_pid[0]  = pid_gains.left_wheel.kp;
    params.left_wheel_vel_pid[1]  = pid_gains.left_wheel.ki;
    params.left_wheel_vel_pid[2]  = pid_gains.left_wheel.kd;
    params.left_wheel_vel_pid[3]  = pid_gains.left_wheel.tf;

    params.right_wheel_vel_pid[0] = pid_gains.right_wheel.kp;
    params.right_wheel_vel_pid[1] = pid_gains.right_wheel.ki;
    params.right_wheel_vel_pid[2] = pid_gains.right_wheel.kd;
    params.right_wheel_vel_pid[3] = pid_gains.right_wheel.tf;

    params.control_mode = control_mode;

    // Write entire struct to FRAM
    return mbot_write_fram(0, sizeof(params), (uint8_t*)&params);
}
