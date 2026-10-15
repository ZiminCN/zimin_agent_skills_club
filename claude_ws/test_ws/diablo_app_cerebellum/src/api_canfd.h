#ifndef API_CANFD_H
#define API_CANFD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "robot.h"
#include "api_protocol.h"

// 16 device, 127 service id

#define CAN_ID_BASE 0x100

#define CAN_ID_STATE 0x102
#define CAN_ID_LOCOMOTION_STATE 0x103

#define CAN_ID_IMU1 0x118
#define CAN_ID_IMU2 0x119
#define CAN_ID_SBUS 0x11E

#define CAN_ID_MOTOR_IN  0x108
#define CAN_ID_MOTOR_FIXED_IN 0x10C
#define CAN_ID_MOTOR_OUT 0x110
#define CAN_ID_MOTOR_SWAP 0x120


#define TYPE_ROBOT 0x00
#define TYPE_DIABLO 0x01
#define TYPE_TITA 0x02
#define MODEL_WHEEL_LEGGED 0x01
#define MODEL_QUADRUPEDAL 0x02

struct robot_info_t {
    uint8_t type;
    uint8_t hw_ver;
    uint8_t sw_ver;
    uint8_t model;
    uint8_t dof;
};

// get robot info
// return robot/motor/sensor info

int api_canfd_init(struct robot_t *robot);

/** upload(send) imu/ahrs data */
void api_canfd_upload_status_data(struct robot_t *robot, struct api_state_t *api_state, const float dt);

/** upload(send) imu/ahrs data */
void api_canfd_upload_imu_data(struct robot_t *robot, struct api_state_t *api_state, const float dt);

/** upload(send) motor out/in data */
void api_canfd_upload_motor_data(struct robot_t *robot, struct api_state_t *api_state, const float dt);

/** swap motor in/out data, recv motor out(desired) data, send motor in(real) data */
void api_canfd_swap_motor_data(struct robot_t *robot, struct api_state_t *api_state, const float dt);

/** report motor state */
void api_canfd_report_motor_state(struct robot_t *robot, struct api_state_t *api_state, const float dt);

#ifdef __cplusplus
}
#endif

#endif