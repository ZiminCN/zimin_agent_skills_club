
#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>

#include "wrapper.h"
#include "api_protocol.h"

enum robot_state_t {
    /** Power on default state */
    ROBOT_SELFTEST_STATE,
    /** Pass self-test ready for locomotion */
    ROBOT_READY_STATE,
    /** Legacy locomotion mode, run in mcu */
    ROBOT_LOCOMOTION_STATE,
    /** Direct API control, advanced locomotion */
    ROBOT_DIRECT_STATE,
    /** Robot out of control, e-stop */
    ROBOT_FAULT_STATE,
    /** Robot try to reocvery */
    ROBOT_RECOVERY_STATE,
    /** Deep sleep, power-off motor */
    ROBOT_SLEEP_STATE,
    /** IMU calibration */
    ROBOT_IMU_CALIBRATION_STATE,
    /** Motor calibration */
    ROBOT_MOTOR_CALIBRATION_STATE,
    /** Motor ota */
    ROBOT_MOTOR_OTA_STATE,
    /** boot recovery mode */
    ROBOT_BOOT_RECOVERY_MODE_STATE,

    ROBOT_STATE_COUNT
};

enum ready_next_t {
    READY_WAITING,
    AUTO_LOCOMOTION,
    FORCE_LOCOMOTION,
    FORCE_DIRECT,
    MOTOR_ZERO_CAL,
    BOOT_RECOVERY_MODE,
    ESTOP_MODE,
};

/** robto state machine */
typedef struct robot_sm_t {
    struct smf_ctx ctx;
    struct robot_t *robot; 
} robot_sm_t;

/** robot public data */
typedef struct robot_t {
    struct robot_sm_t sm;

    float dt; //25us - 100ms
    uint32_t frequency; //10Hz - 4kHz
    uint32_t timestamp; //accuracy 1us

    // uint64_t cyclestamp; //
    uint32_t last_hw_cycle, interval_hw_cycle, overflow_count;

    uint32_t cycle_count;

    uint32_t estimator_freq;
    uint32_t estimator_updated;

    // motor data
    // canfd/usbacm data
    // sensor data

    struct imu_config_t imu_cfg;
    float accel[3];
    float gyro[3];
    float mag[3];
    float temperature;

    // battery voltage
    battery_info_t  batt_info[3];

    void *ahrs;
    float quaternion[4];

    uint8_t hwinfo_id[DEVICE_ID_BIN_MAX_SIZE];

    enum ready_next_t ready_next;

    uint8_t rc_connected;
    struct input_t input;

    // api_data
    struct api_ops_t *api_ops;
    struct api_state_t api_state;
    void *acm_data;

    // 4 wheel-leg mode
    uint8_t motor_index_offset;

    // motor zero
    uint8_t motor_zero;

    // canfd_8m_mode
    uint8_t canfd_8mbps_mode;

#if 0
    uint8_t lock_count;
#endif
    
    // legacy wrapper
    struct wrapper_robot_t *priv;
} robot_t;


struct api_ops_t {
    void (*upload_status_data)(struct robot_t *robot, struct api_state_t *api_state, const float dt);
    void (*upload_imu_data)(struct robot_t *robot, struct api_state_t *api_state, const float dt);
    void (*upload_motor_data)(struct robot_t *robot, struct api_state_t *api_state, const float dt);
    void (*swap_motor_data)(struct robot_t *robot, struct api_state_t *api_state, const float dt);
    void (*report_motor_status)(struct robot_t *robot, struct api_state_t *api_state, const float dt);
};

inline void api_upload_status_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
    if (robot->api_ops && robot->api_ops->upload_status_data)
        robot->api_ops->upload_status_data(robot, api_state, dt);
}

inline void api_upload_imu_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
    if (robot->api_ops && robot->api_ops->upload_imu_data)
        robot->api_ops->upload_imu_data(robot, api_state, dt);
}

inline void api_upload_motor_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
    if (robot->api_ops && robot->api_ops->upload_motor_data)
        robot->api_ops->upload_motor_data(robot, api_state, dt);
}

inline void api_swap_motor_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
    if (robot->api_ops && robot->api_ops->swap_motor_data)
        robot->api_ops->swap_motor_data(robot, api_state, dt);
}

inline void api_update_input(struct api_state_t *api_state, struct input_t *input)
{
    // only copy api_state to input
    input->timestamp = api_state->channel_input.timestamp;
    input->forward = api_state->channel_input.forward;
    input->yaw = api_state->channel_input.yaw;
    input->pitch = api_state->channel_input.pitch;
    input->roll = api_state->channel_input.roll;
    input->height = api_state->channel_input.height;
    input->split = api_state->channel_input.split;
    input->tilt = api_state->channel_input.tilt;

    input->forward_accel = api_state->channel_input.forward_accel;
    input->yaw_accel = api_state->channel_input.yaw_accel;
}


void robot_set_state(struct robot_t *robot, const enum robot_state_t state);
enum robot_state_t robot_get_state(struct robot_t *robot);

int robot_init(struct wrapper_robot_t *wrapper);
void robot_set_frequency(struct robot_t *robot, uint32_t frequency);

#ifdef __cplusplus
}
#endif

#endif 
