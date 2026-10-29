#include "mbot_ros_comms.h"
#include "mbot_classic_ros.h" // For mbot_state_t, mbot_cmd_t, and config defines
#include <string.h>            // For strlen, snprintf in message init
#include "pico/time.h"
#include <mbot/motor/motor.h>  // For mbot_motor_set_duty

// Define ROS Objects (matching extern declarations in .h)
rcl_publisher_t imu_publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t gyrodom_publisher;
rcl_publisher_t motor_vel_publisher;
rcl_publisher_t tf_publisher;
rcl_publisher_t encoders_publisher;
rcl_publisher_t battery_publisher;

sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
nav_msgs__msg__Odometry gyrodom_msg;
mbot_interfaces__msg__MotorVelocity motor_vel_msg;
tf2_msgs__msg__TFMessage tf_msg;
mbot_interfaces__msg__Encoders encoders_msg;
mbot_interfaces__msg__BatteryADC battery_msg;

rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t motor_vel_cmd_subscriber;
rcl_subscription_t motor_pwm_cmd_subscriber;

geometry_msgs__msg__Twist cmd_vel_msg_buffer;
mbot_interfaces__msg__MotorVelocity motor_vel_cmd_msg_buffer;
mbot_interfaces__msg__MotorPWM motor_pwm_cmd_msg_buffer;

rcl_service_t reset_odometry_service;
std_srvs__srv__Trigger_Request reset_odom_req;
std_srvs__srv__Trigger_Response reset_odom_res;

rcl_service_t lidar_power_service;
std_srvs__srv__SetBool_Request lidar_power_req;
std_srvs__srv__SetBool_Response lidar_power_res;

#define FRAME_ID_CAPACITY 16

static char imu_frame_id_buf[FRAME_ID_CAPACITY];
static char odom_frame_id_buf[FRAME_ID_CAPACITY];
static char odom_child_frame_id_buf[FRAME_ID_CAPACITY];

int mbot_ros_comms_init_messages(rcl_allocator_t* allocator) {
    // Initialize messages with dynamic fields
    sensor_msgs__msg__Imu__init(&imu_msg);

    nav_msgs__msg__Odometry__init(&odom_msg);
    nav_msgs__msg__Odometry__init(&gyrodom_msg);
    tf2_msgs__msg__TFMessage__init(&tf_msg);
    geometry_msgs__msg__TransformStamped__Sequence__init(&tf_msg.transforms, 1);

    // Initialize service response messages
    std_srvs__srv__Trigger_Response__init(&reset_odom_res);
    reset_odom_res.message.data = (char*)allocator->allocate(128, allocator->state);
    reset_odom_res.message.capacity = 128;
    reset_odom_res.message.size = 0;

    std_srvs__srv__SetBool_Response__init(&lidar_power_res);
    lidar_power_res.message.data = (char*)allocator->allocate(128, allocator->state);
    lidar_power_res.message.capacity = 128;
    lidar_power_res.message.size = 0;

    // IMU message initialization
    imu_msg.header.frame_id.data = imu_frame_id_buf;
    imu_msg.header.frame_id.capacity = FRAME_ID_CAPACITY;
    snprintf(imu_msg.header.frame_id.data, FRAME_ID_CAPACITY, "base_link");
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);

    // Odometry message initialization
    odom_msg.header.frame_id.data = odom_frame_id_buf;
    odom_msg.header.frame_id.capacity = FRAME_ID_CAPACITY;
    snprintf(odom_msg.header.frame_id.data, FRAME_ID_CAPACITY, "odom");
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);

    odom_msg.child_frame_id.data = odom_child_frame_id_buf;
    odom_msg.child_frame_id.capacity = FRAME_ID_CAPACITY;
    snprintf(odom_msg.child_frame_id.data, FRAME_ID_CAPACITY, "base_footprint");
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);

    // Gyorodom message initialization
    gyrodom_msg.header.frame_id.data = odom_frame_id_buf;
    gyrodom_msg.header.frame_id.capacity = FRAME_ID_CAPACITY;
    snprintf(gyrodom_msg.header.frame_id.data, FRAME_ID_CAPACITY, "odom");
    gyrodom_msg.header.frame_id.size = strlen(gyrodom_msg.header.frame_id.data);

    gyrodom_msg.child_frame_id.data = odom_child_frame_id_buf;
    gyrodom_msg.child_frame_id.capacity = FRAME_ID_CAPACITY;
    snprintf(gyrodom_msg.child_frame_id.data, FRAME_ID_CAPACITY, "base_footprint");
    gyrodom_msg.child_frame_id.size = strlen(gyrodom_msg.child_frame_id.data);

    // Zero all message structs for safe initialization
    memset(&cmd_vel_msg_buffer, 0, sizeof(cmd_vel_msg_buffer));
    memset(&motor_vel_msg, 0, sizeof(motor_vel_msg));
    memset(&motor_vel_cmd_msg_buffer, 0, sizeof(motor_vel_cmd_msg_buffer)); // subscriber buffer
    memset(&motor_pwm_cmd_msg_buffer, 0, sizeof(motor_pwm_cmd_msg_buffer)); // subscriber buffer
    memset(&encoders_msg, 0, sizeof(encoders_msg));
    memset(&battery_msg, 0, sizeof(battery_msg));
    return MBOT_OK;
}

int mbot_ros_comms_init_publishers(rcl_node_t *node) {
    rcl_ret_t ret;
    ret = rclc_publisher_init_best_effort(
        &imu_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu");
    if (ret != RCL_RET_OK) { printf("[FATAL] Failed to init imu_publisher: %d\n", ret); fflush(stdout); return MBOT_ERROR; }

    ret = rclc_publisher_init_default(
        &gyrodom_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "gyrodom");
    if (ret != RCL_RET_OK) { printf("[FATAL] Failed to init gyrodom_publisher: %d\n", ret); fflush(stdout); return MBOT_ERROR; }

    ret = rclc_publisher_init_default(
        &odom_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom");
    if (ret != RCL_RET_OK) { printf("[FATAL] Failed to init odom_publisher: %d\n", ret); fflush(stdout); return MBOT_ERROR; }

    ret = rclc_publisher_init_best_effort(
        &motor_vel_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(mbot_interfaces, msg, MotorVelocity),
        "motor_vel");
    if (ret != RCL_RET_OK) { printf("[FATAL] Failed to init motor_vel_publisher: %d\n", ret); fflush(stdout); return MBOT_ERROR; }

    ret = rclc_publisher_init_default(
        &tf_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        "tf");
    if (ret != RCL_RET_OK) { printf("[FATAL] Failed to init tf_publisher: %d\n", ret); fflush(stdout); return MBOT_ERROR; }

    ret = rclc_publisher_init_best_effort(
        &encoders_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(mbot_interfaces, msg, Encoders),
        "encoders");
    if (ret != RCL_RET_OK) { printf("[FATAL] Failed to init encoders_publisher: %d\n", ret); fflush(stdout); return MBOT_ERROR; }

    ret = rclc_publisher_init_best_effort(
        &battery_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(mbot_interfaces, msg, BatteryADC),
        "battery_adc");
    if (ret != RCL_RET_OK) { printf("[FATAL] Failed to init battery_publisher: %d\n", ret); fflush(stdout); return MBOT_ERROR; }

    return MBOT_OK;
}

int mbot_ros_comms_init_subscribers(rcl_node_t *node) {
    rcl_ret_t ret;
    ret = rclc_subscription_init_default(
        &cmd_vel_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
    if (ret != RCL_RET_OK) { printf("[FATAL] Failed to init cmd_vel_subscriber: %d\n", ret); fflush(stdout); return MBOT_ERROR; }

    ret = rclc_subscription_init_default(
        &motor_vel_cmd_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(mbot_interfaces, msg, MotorVelocity),
        "motor_vel_cmd");
    if (ret != RCL_RET_OK) { printf("[FATAL] Failed to init motor_vel_cmd_subscriber: %d\n", ret); fflush(stdout); return MBOT_ERROR; }

    ret = rclc_subscription_init_default(
        &motor_pwm_cmd_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(mbot_interfaces, msg, MotorPWM),
        "motor_pwm_cmd");
    if (ret != RCL_RET_OK) { printf("[FATAL] Failed to init motor_pwm_cmd_subscriber: %d\n", ret); fflush(stdout); return MBOT_ERROR; }

    return MBOT_OK;
}

int mbot_ros_comms_init_services(rcl_node_t *node) {
    rcl_ret_t ret;

    // Initialize reset odometry service
    ret = rclc_service_init_default(
        &reset_odometry_service,
        node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "reset_odometry");
    if (ret != RCL_RET_OK) {
        printf("[FATAL] Failed to init reset_odometry_service: %d\n", ret);
        fflush(stdout);
        return MBOT_ERROR;
    }

    // Initialize LiDAR power service
    ret = rclc_service_init_default(
        &lidar_power_service,
        node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
        "lidar_power");
    if (ret != RCL_RET_OK) {
        printf("[FATAL] Failed to init lidar_power_service: %d\n", ret);
        fflush(stdout);
        return MBOT_ERROR;
    }

    return MBOT_OK;
}

void reset_odometry_callback(const void * req, void * res) {
    (void)req; // Request is empty for Trigger service
    std_srvs__srv__Trigger_Response * response = (std_srvs__srv__Trigger_Response *)res;

    // Reset odometry in a thread-safe manner
    ENTER_CRITICAL();
    mbot_state.odom_x = 0.0f;
    mbot_state.odom_y = 0.0f;
    mbot_state.odom_theta = 0.0f;
    mbot_state.gyrodom_x = 0.0f;
    mbot_state.gyrodom_y = 0.0f;
    mbot_state.gyrodom_theta = 0.0f;
    EXIT_CRITICAL();

    // Set response
    response->success = true;
    snprintf(response->message.data, response->message.capacity, "Odometry reset to (0, 0, 0)");
    response->message.size = strlen(response->message.data);

    printf("[INFO] Odometry reset to origin\n");
}

void lidar_power_callback(const void * req, void * res) {
    const std_srvs__srv__SetBool_Request * request = (const std_srvs__srv__SetBool_Request *)req;
    std_srvs__srv__SetBool_Response * response = (std_srvs__srv__SetBool_Response *)res;

    // Set LiDAR duty cycle: ON (true) = 1.0 (100%), OFF (false) = 0.0 (0%)
    float duty = request->data ? 1.0f : 0.0f;
    mbot_motor_set_duty(MOT_LIDAR, duty);

    // Set response
    response->success = true;
    snprintf(response->message.data, response->message.capacity,
             "LiDAR power %s (duty: %.0f%%)", request->data ? "ON" : "OFF", duty * 100.0f);
    response->message.size = strlen(response->message.data);

    printf("[INFO] LiDAR power %s\n", request->data ? "ON" : "OFF");
}

void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;
    mbot_cmd.timestamp_us = time_us_64();
    mbot_cmd.vx = twist_msg->linear.x;
    mbot_cmd.vy = twist_msg->linear.y;
    mbot_cmd.wz = twist_msg->angular.z;
    mbot_cmd.drive_mode = MODE_MBOT_VEL;
}

void motor_vel_cmd_callback(const void * msgin) {
    const mbot_interfaces__msg__MotorVelocity * vel_msg = (const mbot_interfaces__msg__MotorVelocity *)msgin;
    mbot_cmd.timestamp_us = time_us_64();
    mbot_cmd.wheel_vel[MOT_L] = vel_msg->velocity[MOT_L];
    mbot_cmd.wheel_vel[MOT_R] = vel_msg->velocity[MOT_R];
    mbot_cmd.drive_mode = MODE_MOTOR_VEL;
}

void motor_pwm_cmd_callback(const void * msgin) {
    const mbot_interfaces__msg__MotorPWM * pwm_msg = (const mbot_interfaces__msg__MotorPWM *)msgin;
    mbot_cmd.timestamp_us = time_us_64();
    mbot_cmd.motor_pwm[MOT_L] = pwm_msg->pwm[MOT_L];
    mbot_cmd.motor_pwm[MOT_R] = pwm_msg->pwm[MOT_R];
    mbot_cmd.drive_mode = MODE_MOTOR_PWM;
}

int mbot_ros_comms_add_to_executor(rclc_executor_t *executor) {
    rcl_ret_t ret;

    // Add subscribers
    ret = rclc_executor_add_subscription(executor, &cmd_vel_subscriber, &cmd_vel_msg_buffer,
                                        &cmd_vel_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_executor_add_subscription(executor, &motor_vel_cmd_subscriber, &motor_vel_cmd_msg_buffer,
                                       &motor_vel_cmd_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_executor_add_subscription(executor, &motor_pwm_cmd_subscriber, &motor_pwm_cmd_msg_buffer,
                                       &motor_pwm_cmd_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    // Add services
    ret = rclc_executor_add_service(executor, &reset_odometry_service, &reset_odom_req,
                                   &reset_odom_res, &reset_odometry_callback);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Failed to add reset_odometry_service to executor: %d\n", ret);
        return MBOT_ERROR;
    }

    ret = rclc_executor_add_service(executor, &lidar_power_service, &lidar_power_req,
                                   &lidar_power_res, &lidar_power_callback);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Failed to add lidar_power_service to executor: %d\n", ret);
        return MBOT_ERROR;
    }

    return MBOT_OK;
}