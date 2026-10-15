

#ifndef API_ACM_H
#define API_ACM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "robot.h"
#include "api_protocol.h"

#define ACM_HDR_LEN 8
#define ACM_CAN_HDR_LEN 8
#define ACM_CAN_DLEN 64

#define ACM_TAG 0xAA55
#define ACM_TYPE_CAM 0x10
#define ACM_TYPE_CAM_BUS0 0x10
#define ACM_TYPE_CAM_BUS1 0x11
#define ACM_TYPE_CAM_BUS2 0x12
#define ACM_TYPE_ROBOT 0x2F


typedef struct acm_pkt_t {
    uint16_t tag;
    uint16_t length;
    uint16_t checksum;
    uint8_t type;
    uint8_t res0;
} acm_pkt_t;

typedef struct acm_can_pkt_t {
    uint32_t id;
    uint8_t dlc;
    uint8_t flags;
    uint8_t padding[2];
    uint8_t data[ACM_CAN_DLEN];
} acm_can_pkt_t;

int api_acm_init(struct robot_t *robot);

/** upload(send) imu/ahrs data */
void api_acm_upload_imu_data(struct robot_t *robot, struct api_state_t *api_state, const float dt);

/** upload(send) motor out/in data */
void api_acm_upload_motor_data(struct robot_t *robot, struct api_state_t *api_state, const float dt);

/** swap motor in/out data, recv motor out(desired) data, send motor in(real) data */
void api_acm_swap_motor_data(struct robot_t *robot, struct api_state_t *api_state, const float dt);

#ifdef __cplusplus
}
#endif

#endif
