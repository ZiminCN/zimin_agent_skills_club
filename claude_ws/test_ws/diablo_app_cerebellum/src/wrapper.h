
#ifndef WRAPPER_H
#define WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "api_protocol.h"
#include "gpio_ctrl.h"


#if defined(CONFIG_DIABLO_EIGHT_DOF)
#define STUB_MOTOR_NUM 4
#define STUB_LEG_NUM 2
enum joint_type_t {
    M_ABAD = 0,
    M_HIP = 1,
    M_KNEE = 2,
    M_ANKLE = 3,
};
#elif defined(CONFIG_DIABLO_SIX_DOF)
#define STUB_MOTOR_NUM 3
#define STUB_LEG_NUM 2
enum joint_type_t {
    M_ABAD = 0,
    M_HIP = 0,
    M_KNEE = 1,
    M_ANKLE = 2,
};
#else
#error "Diablo DoF model not specified"
#endif 

enum leg_type_t {
    LEFT = 0,
    RIGHT = 1,
};

enum motor_control_mode_t {
    VOLTAGE,
    POSITION,
    VELOCITY,
    CURRENT,
    LOCOMOTION
};

enum motor_direction_t {
    FORWARD = 0,
    REVERSE = 1,
};

struct motor_out_t {
    uint32_t timestamp;
#ifndef TORQUE_ONLY
    float position;  // rad
    float velocity;  // rad/s
    float torque;    // Nm
    float kp;
    float kd;
#else
    float torque;  // Nm
#endif
};

struct motor_in_t {
    uint32_t timestamp;
    float position;  // rad
    float velocity;  // rad/s
    float torque;    // Nm
};

struct motor_in_ex_t {
    float voltage;
    uint8_t temp_drive;
    uint8_t temp_winding;
};

enum motor_type_t {
    MT_UNKNOWN = 0,
    MT_M1505 = 1,
    MT_M1502 = 2,
    MT_P1010 = 3,
    MT_MG10015 = 4,
    MT_RS1 = 5,
    MT_Z1 = 6,
    MT_DM8006 = 7,
};

enum motor_model_t {
    MODEL_UNKNOWN = 0,
    MODEL_P1010_M1502 = 1,
    MODEL_P1010_M1505 = 2,
    MODEL_MG10015_M1502 = 3,
    MODEL_MG10015_M1505 = 4,
    MODEL_M1502_M1502 = 5,
    MODEL_P1010_RS1 = 6,
    MODEL_Z1_M1505 = 7,
    MODEL_Z1_RS1 = 8,
    MODEL_P1010_DM8006 = 9,
    MODEL_P1010_P1010 = 10,
};

struct motor_t {
    struct motor_out_t out;
    struct motor_in_t in;
    struct motor_in_ex_t in_ex;
#if 0
    struct motor_in_t ref_in;
#endif
    enum motor_type_t type;
    enum motor_control_mode_t mode;
    enum motor_direction_t direction;
    float zero_angle_bias;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t status;
#if 0
    uint8_t mcu_id[8];
    uint8_t ota_recv;
    uint8_t ota_state;
    uint8_t ota_status;
    uint16_t ota_offset;
    uint16_t ota_segments;
#endif
};

struct motor_group_t {
    struct motor_t motors[STUB_MOTOR_NUM];
    enum leg_type_t type;
    const struct device *bus;
    uint32_t bus_tx_count;
    uint32_t bus_rx_count;
    uint32_t bus_tx_error;
    uint32_t bus_rx_error;
    int motor_status;
};

typedef enum imu_axis_type_t {
    IMU_AXIS_X = 0,
    IMU_AXIS_Y = 1,
    IMU_AXIS_Z = 2,
} imu_axis_type_t;

typedef enum imu_axis_direction_t {
    IMU_AXIS_DIR_FR = 1,
    IMU_AXIS_DIR_REV = -1,
} imu_axis_direction_t;

typedef struct imu_config_t {
    uint8_t x_map;
    uint8_t y_map;
    uint8_t z_map;
    int8_t  x_dir;
    int8_t  y_dir;
    int8_t  z_dir;
} imu_config_t;

typedef struct input_t {
    float forward;
    float yaw;
    float pitch;
    float roll;
    float height;
    float split;
    float tilt;

    float forward_accel;
    float yaw_accel;

    uint8_t input_mode;
    uint8_t head_mode;
    uint8_t stand_mode;
    uint8_t jump_state;

    // bool lost;
    uint32_t timestamp;
    uint32_t update_count;
} input_t;

typedef struct battery_info_t {
    uint16_t voltage;
} battery_info_t;

struct wrapper_robot_t;

struct wrapper_robot_t *new_wrapper();

void wrap_robot_deinit(struct wrapper_robot_t *wrapper);

void wrap_robot_init(struct wrapper_robot_t *wrapper);

#if 0

void wrap_plan_start(const struct wrapper_robot_t *wrapper);

void wrap_plan_update(const struct wrapper_robot_t *wrapper, const float dt);

void wrap_ctrl_start(const struct wrapper_robot_t *wrapper);

void wrap_ctrl_update(const struct wrapper_robot_t *wrapper, const float dt);

#else

void wrap_ctrl_new(struct wrapper_robot_t *wrapper, const uint32_t frequency);
void wrap_ctrl_loop(struct wrapper_robot_t *wrapper, const float dt);
void wrap_update_estimator(struct wrapper_robot_t *wrapper);

#endif

void wrap_update_battery_info(const struct wrapper_robot_t *wrapper, struct battery_info_t *batt_info, int count, const float dt);

void wrap_update_status(const struct wrapper_robot_t *wrapper, struct api_locomotion_status_t status[], const float dt);

void wrap_update_input(const struct wrapper_robot_t *wrapper, struct input_t *input, const float dt);

int wrap_update_imu_data(struct wrapper_robot_t *wrapper, const float accel[3], const float gyro[3], const float q[4], const float dt);

struct motor_group_t *wrap_get_motor_group(struct wrapper_robot_t *wrapper, int i);

enum motor_model_t wrap_get_motor_model(struct wrapper_robot_t *wrapper);

void wrap_drvie_set_zero(struct wrapper_robot_t *wrapper, int i);

int wrap_drvie_setup(struct wrapper_robot_t *wrapper);

void wrap_update_motor_stub(struct wrapper_robot_t *wrapper);

void wrap_motor_stub_zero(struct wrapper_robot_t *wrapper);

void wrap_motor_stub_fresh(struct wrapper_robot_t *wrapper);

int wrap_drive_reconfigure(struct wrapper_robot_t *wrapper, uint32_t frequency);

int wrap_drive_fresh(struct wrapper_robot_t *wrapper);

int wrap_drvie_close(const struct wrapper_robot_t *wrapper);

int wrap_drive_reset_count(struct wrapper_robot_t *wrapper);

int wrap_drive_reset_status(struct wrapper_robot_t *wrapper);

int wrap_drive_check(const struct wrapper_robot_t *wrapper, int state[2]);

struct imu_config_t wrap_get_imu_config(struct wrapper_robot_t *wrapper, int index);

#ifdef __cplusplus
}
#endif

#endif
