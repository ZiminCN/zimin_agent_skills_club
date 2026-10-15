

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include "wrapper.h"


LOG_MODULE_REGISTER(motor_athena, LOG_LEVEL_INF);


const struct device *const can0 = DEVICE_DT_GET(DT_ALIAS(canbus0));
const struct device *const can1 = DEVICE_DT_GET(DT_ALIAS(canbus1));

// Torque Constant 1502D 0.8 Nm/A, 1502C 0.43 Nm/A
// Phase Resistance 0.7 Ohm
// Back EMF Constant 0.85
// value -16383 ~ 16383 == ~55A ~ 55A

static inline float m15_torque_to_current(struct motor_t *motor)
{
    float current;

    current = (motor->out.torque / 0.8) * (32767 / 55.0);

    if (motor->direction == REVERSE)
        current = -current;

    return current;
}

static inline float m15_torque_to_voltage(struct motor_t *motor)
{
    float current, voltage;

    current = (motor->out.torque / 0.8) * (32767 / 55.0);
    voltage = (current * 0.7 + motor->in.velocity * 0.85) * (32767 / 28.0);

    if (motor->direction == REVERSE)
        voltage = -voltage;

    return voltage;
}

static void m15_stuff_broadcast(struct can_frame *frame, struct motor_group_t *group)
{
    uint8_t *data = frame->data;
    struct motor_t *motor = 0;
    int16_t out = 0;


    frame->flags = 0;
    frame->id = 0x32;
    frame->dlc = 8;

    memset(data, 0, 8);

    motor = &group->motors[M_HIP];
    // TODO: support torque output
    // out = (int16_t)m15_torque_to_voltage(motor);
    out = (int16_t)m15_torque_to_current(motor);

    data[0] = (out & 0xFF00) >> 8;
    data[1] = (out & 0x00FF);

    motor = &group->motors[M_KNEE];
    // TODO: support torque output
    // out = (int16_t)m15_torque_to_voltage(motor);
    out = (int16_t)m15_torque_to_current(motor);

    data[2] = (out & 0xFF00) >> 8;
    data[3] = (out & 0x00FF);

    motor = &group->motors[M_ANKLE];
    out = (int16_t)m15_torque_to_current(motor);

    data[4] = (out & 0xFF00) >> 8;
    data[5] = (out & 0x00FF);
}

static void m15_tx_callback(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(error);
    ARG_UNUSED(user_data);
}

static void any_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{

}

static void m15_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct motor_group_t *group = (struct motor_group_t *)user_data;
    struct motor_t *motor = 0;
    float dir_c = 1.0;

    uint8_t id = frame->id - 0x96;
    uint8_t idx = id - 1;
    uint8_t error = frame->data[6];
    uint8_t mode = frame->data[7];
    int16_t value;

    float iq;
    float speed;
    float encoder;

    value = frame->data[0];
    value = value << 8 | frame->data[1];
    speed = (float)value / 21000 * 210;

    value = frame->data[2];
    value = value << 8 | frame->data[3];
    iq = (float)value / 32767 * 55;

    value = frame->data[4];
    value = value << 8 | frame->data[5];
    value = value & 0x7FFF;
    encoder = (float)value / 32767 * 360;

    // encoder = encoder > 180.0 ? encoder - 360.0 : encoder;

    motor = &group->motors[idx];

    if (motor->direction == REVERSE) dir_c = -1.0;

    motor->in.torque = dir_c * iq * 0.8;               // 1502D 0.8 Nm/A
    motor->in.velocity = dir_c * speed * 0.1047198;    // rpm to rad/s
    motor->in.position = dir_c * encoder * 0.0174533;  // deg to rad

    motor->in.rx_count += 1;

#if 0
    if (motor->in.rx_count % 1000 == 1) {
#if 0
        LOG_INF("ID: 0x%03X [0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]",
                frame->id,
                frame->data[0], frame->data[1], frame->data[2], frame->data[3],
                frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
#endif

#if 1
        LOG_INF("id: %d mode: %d error: %d iq: %+0.3fA speed: %+0.2frpm encoder: %+0.2fdeg",
                id, mode, error, iq, speed, encoder);
#endif
    }
#endif
}

static void m15_motor_config(struct wrapper_robot_t *wrapper)
{
    struct motor_group_t *group = 0;

    group = wrap_get_motor_group(wrapper, LEFT);
    group->type = LEFT;
    group->motors[M_HIP].direction = REVERSE;
    group->motors[M_KNEE].direction = REVERSE;
    group->motors[M_ANKLE].direction = FORWARD;

    group->motors[M_HIP].zero_angle_bias = 0;
    group->motors[M_KNEE].zero_angle_bias = 0;
    group->motors[M_ANKLE].zero_angle_bias = 0;

    group = wrap_get_motor_group(wrapper, RIGHT);
    group->type = RIGHT;
    group->motors[M_HIP].direction = FORWARD;
    group->motors[M_KNEE].direction = FORWARD;
    group->motors[M_ANKLE].direction = REVERSE;

    group->motors[M_HIP].zero_angle_bias = 0;
    group->motors[M_KNEE].zero_angle_bias = 0;
    group->motors[M_ANKLE].zero_angle_bias = 0;
}

static void m15_motor_init(struct wrapper_robot_t *wrapper)
{
    int ret;
    struct can_frame frame;
    uint8_t *data = frame.data;
    struct motor_group_t *group = 0;

    frame.flags = 0;
    frame.id = 0x105;
    frame.dlc = 8;

    memset(frame.data, 0, 8);

    frame.data[0] = 0x01;
    frame.data[1] = 0x01;
    frame.data[2] = 0x01;
    frame.data[3] = 0x01;

    group = wrap_get_motor_group(wrapper, LEFT);

    ret = can_send(can1, &frame, K_NO_WAIT, m15_tx_callback, group);
    if (ret != 0) {
        LOG_INF("can1 send failed");
        return ret;
    }

    group = wrap_get_motor_group(wrapper, RIGHT);

    ret = can_send(can0, &frame, K_NO_WAIT, m15_tx_callback, group);
    if (ret != 0) {
        LOG_INF("can0 send failed");
        return ret;
    }
}

int wrap_drvie_setup(struct wrapper_robot_t *wrapper)
{
    int ret = 0;
    struct motor_group_t *group = 0;

    m15_motor_config(wrapper);

    const struct can_filter any_filter = {
        .id = 0x000, .mask = 0x000, .flags = CAN_FILTER_DATA};

    const struct can_filter m15_filter = {
        .id = 0x090, .mask = CAN_STD_ID_MASK - 0x00F, .flags = CAN_FILTER_DATA};


    ret = can_start(can1);
    if (ret != 0) {

    }

    group = wrap_get_motor_group(wrapper, LEFT);

    can_add_rx_filter(can1, m15_rx_callback, (void *)group, &m15_filter);

    can_add_rx_filter(can1, any_rx_callback, (void *)group, &any_filter);


    ret = can_start(can0);
    if (ret != 0) {

    }

    group = wrap_get_motor_group(wrapper, RIGHT);

    can_add_rx_filter(can0, m15_rx_callback, (void *)group, &m15_filter);

    can_add_rx_filter(can0, any_rx_callback, (void *)group, &any_filter);

    m15_motor_init(wrapper);

    return ret;
}

int wrap_drive_fresh(struct wrapper_robot_t *wrapper)
{
    int ret;
    struct can_frame frame;
    struct motor_group_t *group = 0;

    group = wrap_get_motor_group(wrapper, LEFT);

    m15_stuff_broadcast(&frame, group);

    ret = can_send(can1, &frame, K_NO_WAIT, m15_tx_callback, group);
    if (ret != 0) {
        // LOG_INF("can1 send failed");
        return ret;
    }

    group = wrap_get_motor_group(wrapper, RIGHT);

    m15_stuff_broadcast(&frame, group);

    ret = can_send(can0, &frame, K_NO_WAIT, m15_tx_callback, group);
    if (ret != 0) {
        // LOG_INF("can0 send failed");
        return ret;
    }

    return 0;
}

int wrap_drvie_close(const struct wrapper_robot_t *wrapper)
{
    int ret;

    ret = can_stop(can1);
    if (ret != 0) {

    }

    ret = can_stop(can0);
    if (ret != 0) {

    }

    return 0;
}
