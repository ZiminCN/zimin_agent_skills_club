
#ifndef API_PROTOCOL_H
#define API_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PACKED __attribute__((__packed__))

#define DEVICE_ID_BIN_MAX_SIZE 8
#define DEVICE_ID_HEX_MAX_SIZE ((DEVICE_ID_BIN_MAX_SIZE * 2) + 1)

#if 0
/* 40 bytes */
struct api_rc_data_t {
    uint32_t timestamp;
    float axes[2];
    uint8_t keys[4];
    float axes_ex[2];
    uint8_t keys_ex[4];
    float axes_ex2[2];
    uint8_t keys_ex2[4];
} PACKED;
#endif

/* 28 bytes, only for ros2 */
struct api_twist_t {
    uint32_t timestamp;
    float linear[3];
    float angular[3];
#if 0
    float height;
    uint8_t mode;
    uint8_t status;
#endif
} PACKED;

/* 32 bytes */
struct api_channel_input_t {
    uint32_t timestamp;
    float forward;
    float yaw;
    float pitch;
    float roll;
    float height;
    float split;
    float tilt;
    float forward_accel;
    float yaw_accel;
} PACKED;

#if 0
struct api_jump_cmd_t {
    uint32_t timestamp;
    uint8_t state;
    float length;
    float hight;
} PACKED;

struct api_stand_cmd_t {
    uint32_t timestamp;
    uint8_t mode;
} PACKED;

struct api_head_cmd_t {
    uint32_t timestamp;
    uint8_t mode;
} PACKED;

struct api_input_cmd_t {
    uint32_t timestamp;
    uint8_t mode;
} PACKED;
#endif

/* 36 bytes */
struct api_locomotion_state_t {
    uint32_t timestamp;
    float forward;
    float yaw;
    float pitch;
    float roll;
    float height;
    float split;
    float tilt;
    uint8_t input_mode;
    uint8_t stand_mode;
    uint8_t head_mode;
    uint8_t jump_mode;
} PACKED;

struct api_imu_data_u16_t {
    uint32_t timestamp;
    uint16_t accel[3];
    uint16_t gyro[3];
    uint16_t quaternion[4];
} PACKED;

/* 44 bytes */
struct api_imu_data_t {
    uint32_t timestamp;
    float accel[3];
    float gyro[3];
    float quaternion[4];
    float temperature;
} PACKED;

struct imu_fixed_data_t{
    uint32_t timestamp;       
    int16_t accel[3];         
    int16_t gyro[3];          
    int16_t quaternion[4];    
    int16_t temperature;      
} PACKED;

/* 14 bytes */
struct api_motor_out_16_t {
    uint32_t timestamp;
    int16_t position;
    int16_t kp;
    int16_t velocity;
    int16_t kd;
    int16_t torque;
} PACKED;

/* 10 bytes */
struct api_motor_in_16_t {
    uint32_t timestamp;
    int16_t position;
    int16_t velocity;
    int16_t torque;
} PACKED;

/* 14 bytes */
struct api_motor_out_u16_t {
    uint32_t timestamp;
    uint16_t position;
    uint16_t kp;
    uint16_t velocity;
    uint16_t kd;
    uint16_t torque;
} PACKED;

/* 10 bytes */
struct api_motor_in_u16_t {
    uint32_t timestamp;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
} PACKED;

/* 24 bytes */ 
struct api_motor_out_t {
    uint32_t timestamp;
    float position;
    float kp;
    float velocity;
    float kd;
    float torque;
} PACKED;

/* 16 bytes */
struct api_motor_in_t {
    uint32_t timestamp;
    float position;
    float velocity;
    float torque;
} PACKED;

/* 48 bytes */
struct api_motor_group_out_t {
    struct api_motor_out_t out[2];
} PACKED;

/* 32/64 bytes */
struct api_motor_group_in_t {
    struct api_motor_in_t in[2];
} PACKED;

/* extend mode */
/* 8 bytes echo, 64 bytes for 8 motor */ 
struct api_motor_torque_out_t {
    uint32_t timestamp;
    float torque;
} PACKED;


enum api_rpc_key_t {
    RPC_UNDEFINED = 0x000,
    GET_MODEL_INFO = 0x100,
    GET_SERIAL_NUMBER = 0x101,
    SET_READY_NEXT = 0x200,
    SET_BOARDCAST = 0x201,
    SET_INPUT_MODE = 0x221,
    SET_STAND_MODE = 0x222,
    SET_HEAD_MODE = 0x223,
    SET_JUMP = 0x231,
    SET_MOTOR_ZERO= 0x280,
};

enum api_stand_mode_t {
    STAND_IDLE = 0x00,
    STAND_CRAW = 0x01,
    STAND_STAND = 0x02,
    STAND_TRANSFORM_UP = 0x03,
    STAND_TRANSFORM_DOWN = 0x04,
};

enum api_jump_state_t {
    JUMP_IDLE = 0x00,
    JUMP_CHARGE = 0x01,
    JUMP_JUMP = 0x02,
};

struct api_rpc_request_t {
    uint32_t timestamp;
    uint16_t key;
    union {
      uint8_t u8;
      uint8_t u8x4[4];
      uint16_t u16;
      uint16_t u16x4[4];
      uint32_t u32;
      uint32_t u32x4[4];
      float f32;
      float f32x4[4];
    } value;
} PACKED;

struct api_rpc_response_t {
    uint32_t timestamp;
    uint16_t key;
    uint32_t value;
} PACKED;

enum api_status_key_t {
    MOTOR_STATE = 0x100,
};

struct api_motor_status_t {
    uint32_t timestamp;
    uint16_t key;
    uint8_t value[8];
} PACKED;

struct api_locomotion_status_t {
    uint32_t timestamp;
    uint16_t key;
    union {
      uint8_t u8;
      uint8_t u8x4[4];
      uint16_t u16;
      uint16_t u16x4[4];
      uint32_t u32;
      uint32_t u32x4[4];
      float f32;
      float f32x4[4];
      float f32x9[9];
    } value;
} PACKED;

#if 0
// TODO: support locomotion debug
struct api_locomotion_state_sheet1_t {
    float cmd_forward_vel;
    float cmd_yaw_vel;
    float cmd_lateral_vel;
    float abad_angle;
    float roll_angle;
    float height;
    float imu_bias_y;
    float imu_bias_x;
} PACKED;

struct api_locomotion_state_sheet2_t {
    float leg_left_load;
    float leg_right_load;
    float leg_left_len;
    float leg_right_len;
    float forward_vel;
    float yaw_vel;
    float tilt;
    float d_tilt;
    float pitch;
    float d_pitch;
    float height;
} PACKED;

struct api_locomotion_state_sheet3_t {
    float az;
    float ax;
    float ddz;
    float ddx;
    float forward_err;
    float d_forward_err;
    float set_pitch;
    float set_d_pitch;
    float set_tilt;
    float set_d_tilt;
    float com[3];
    float angular_moment_y;
    float angular_moment_x;
} PACKED;
#endif

#define API_MAX_MOTORS 8

struct api_state_t {
    struct api_motor_out_t out[API_MAX_MOTORS];
    struct api_motor_in_t in[API_MAX_MOTORS];
    struct api_motor_in_16_t in_16[API_MAX_MOTORS];
    struct api_motor_out_16_t out_16[API_MAX_MOTORS];
    uint32_t api_motor_out_count;
    uint32_t api_motor_in_count;
#ifdef API_RC_DATA
    struct api_rc_data_t rc_data;
#endif
#if 0
    struct api_twist_t twist;
#endif
    struct api_channel_input_t channel_input;
    struct api_locomotion_status_t locomotion_status[8];

    uint32_t send_drop_count;
    uint32_t send_error_count;
    uint32_t status_offset;
};

//uint16_t float_to_uint16(float in, float min, float max);
//float uint16_to_float(uint16_t in, float min, float max);
static uint8_t calculate_optimal_q(float min, float max, uint8_t max_q);
int16_t float_to_int16(float in, float min, float max);
float int16_to_float(int16_t in, float min, float max);

#ifdef __cplusplus
}
#endif

#endif
