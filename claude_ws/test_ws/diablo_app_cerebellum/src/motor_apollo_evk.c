

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include <math.h>
#include "wrapper.h"
#ifdef CONFIG_APP_ABUP_FOTA	
#include "upgrade_motor.h"
#endif

LOG_MODULE_REGISTER(motor_apollo, LOG_LEVEL_INF);


static const struct device *const can0 = DEVICE_DT_GET(DT_ALIAS(canbus0));
static const struct device *const can1 = DEVICE_DT_GET(DT_ALIAS(canbus1));

// Torque Constant 1502D 0.8 Nm/A, 1502C 0.43 Nm/A
// Phase Resistance 0.7 Ohm
// Back EMF Constant 0.85
// value -16383 ~ 16383 == ~55A ~ 55A

#define M1505A_TORQUE_CONSTANT 0.63
#define P1010A_TORQUE_CONSTANT 2.5
// #define P1010A_TORQUE_CONSTANT 1.85 // P8
#define MG10015_TORQUE_CONSTANT 1.4

static void motor_group_reset_count(struct motor_group_t *group)
{
    for (int i = M_ABAD; i <= M_ANKLE; i++) {
        group->motors[i].rx_count = 0;
        group->motors[i].tx_count = 0;
    }
}

static inline float m1505a_torque_to_current(struct motor_t *motor)
{
    float current;

    // -32767 ~ 32767, -55 ~ 55
    // Torque Constant 1505A 0.63 Nm/A
    float torque = motor->out.torque +
                motor->out.kp * (motor->out.position - motor->in.position) +
                motor->out.kd * (motor->out.velocity - motor->in.velocity);
#define M1505A_MAX_TORQUE 16.0
    if (torque > M1505A_MAX_TORQUE)
        torque = M1505A_MAX_TORQUE;
    if (torque < -M1505A_MAX_TORQUE)
        torque = -M1505A_MAX_TORQUE;

#if 0
    static float m15_step = 2.0; // one step 2Nm
    // 0 - 16Nm
    static float m15_input[] = {0.0, 1650.0, 4300.0, 6650.0, 8300, 8300 + 1 * 1650, 8300 + 2 * 1650, 8300 + 3 * 1650, 8300 + 4 * 1650};

    float input = torque < 0 ? -torque : torque;
    int i = (int)(input / m15_step);
    float base = m15_input[i];
    float fraction = (m15_input[i + 1] - m15_input[i]) * (input - i * m15_step);
    current = (base + fraction);
    current = torque < 0 ? -current : current;
#else
    current = (torque / M1505A_TORQUE_CONSTANT) / 55.0f * INT16_MAX;
#endif


    if (motor->direction == REVERSE)
        current = -current;

    return current;
}

static inline float m15_torque_to_voltage(struct motor_t *motor)
{
    float current, voltage;

    current = (motor->out.torque / 0.8) * (INT16_MAX / 55.0);
    voltage = (current * 0.7 + motor->in.velocity * 0.85) * (INT16_MAX / 28.0);

    if (motor->direction == REVERSE)
        voltage = -voltage;

    return voltage;
}

static inline float p1010a_torque_to_current(struct motor_t *motor)
{
    float current;

    // Torque Constant 3 Nm/A (Gear box output torque)
    // value -3000 ~ 3000 == -30A ~ 30A
    // float error_position = motor->out.position - motor->in.position;
    // while (error_position > M_PI)
    //     error_position -= 2 * M_PI;
    // while (error_position < -M_PI)
    //     error_position += 2 * M_PI;
    
    float torque = motor->out.torque +
                    motor->out.kp * (motor->out.position - motor->in.position) +
                    motor->out.kd * (motor->out.velocity - motor->in.velocity);
#define P10_MAX_TORQUE 90.0
    if (torque > P10_MAX_TORQUE)
        torque = P10_MAX_TORQUE;
    if (torque < -P10_MAX_TORQUE)
        torque = -P10_MAX_TORQUE;
#if 0
    static float p10_step = 10; // one step 10Nm
    // 0 - 100Nm
    static float p10_input[] = {0.0, 625.0, 1250.0, 1800.0, 2525.0, 3250.0, 3900, 3900 + 1 * 650, 3900 + 2 * 650, 3900 + 3 * 650, 3900 + 4 * 650};

    float input = torque < 0 ? -torque : torque;
    int i = (int)(input / p10_step);
    float base = p10_input[i];
    float fraction = (p10_input[i + 1] - p10_input[i]) * (input - i * p10_step);
    current = (base + fraction);
    current = torque < 0 ? -current : current;
#else
    current = (torque / P1010A_TORQUE_CONSTANT) * 100.0f;
#endif

    if (motor->direction == REVERSE)
        current = -current;

    return current;
}

static void lk_stuff_broadcast(struct can_frame *frame, struct motor_group_t *group)
{
    uint8_t *data = frame->data;
    struct motor_t *motor = 0;
    int16_t current = 0;

    frame->flags = 0;
    frame->id = 0x280;
    frame->dlc = 8;

    memset(frame->data, 0, 8);

    for (int i = M_ABAD; i < M_ANKLE; i++) {
        motor = &group->motors[i];
        // Torque Constant 1.4 Nm/A, Gear box 10:1
        // value -2000 ~ 2000 == -32A ~ 32A
        current = (motor->out.torque / 1.4) * (2000 / 32.0);

        if (motor->direction == REVERSE)
            current = -current;

        data[i * 2 + 0] = (current & 0x00FF);
        data[i * 2 + 1] = (current & 0xFF00) >> 8;
    }
}

static void p1010a_stuff_broadcast(struct can_frame *frame, struct motor_group_t *group)
{
    uint8_t *data = frame->data;
    struct motor_t *motor = 0;
    int16_t out = 0;


    frame->flags = 0;
    frame->id = 0x32;
    frame->dlc = 8;

    memset(data, 0, 8);

    for (int i = M_ABAD; i <= M_ANKLE; i++) {
        motor = &group->motors[i];

        out = (int16_t)p1010a_torque_to_current(motor);

        data[i * 2 + 0] = (out & 0xFF00) >> 8;
        data[i * 2 + 1] = (out & 0x00FF);
    }
}

static void p1010a_m1505a_stuff_broadcast(struct can_frame *frame, struct motor_group_t *group)
{
    uint8_t *data = frame->data;
    struct motor_t *motor = 0;
    int16_t out = 0;


    frame->flags = 0;
    frame->id = 0x32;
    frame->dlc = 8;

    memset(data, 0, 8);

    for (int i = M_ABAD; i <= M_ANKLE; i++) {
        motor = &group->motors[i];

        if (i != M_ANKLE) // P1010A
            out = (int16_t)p1010a_torque_to_current(motor);
        else // M1505A
            out = (int16_t)m1505a_torque_to_current(motor);

        data[i * 2 + 0] = (out & 0xFF00) >> 8;
        data[i * 2 + 1] = (out & 0x00FF);
    }
}

static void m1505a_stuff_broadcast(struct can_frame *frame, struct motor_group_t *group)
{
    uint8_t *data = frame->data;
    struct motor_t *motor = 0;
    int16_t out = 0;


    frame->flags = 0;
    frame->id = 0x32;
    frame->dlc = 8;

    memset(data, 0, 8);

    motor = &group->motors[M_ANKLE]; // hardcode
    out = (int16_t)m1505a_torque_to_current(motor);

    data[6] = (out & 0xFF00) >> 8;
    data[7] = (out & 0x00FF);
}

static void lk_tx_callback(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(error);
    ARG_UNUSED(user_data);
}

static void any_tx_callback(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);

    struct motor_group_t *group = (struct motor_group_t *)user_data;

    if (error == 0) {
        group->bus_tx_count += 1;
    } else if (error == -EAGAIN) {
        group->bus_tx_error += 1;
        LOG_WRN("CAN TX DROP: %d, %d", group->type, group->bus_tx_error);
    } else {
        group->bus_tx_error += 1;
        LOG_ERR("CAN TX ERROR: %d, %d, %d", error, group->type, group->bus_tx_error );
    }
}

static void dm_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data);

static void any_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    int id = 0;
    int idx = -1;
    uint32_t u32val = 0;
    uint8_t mode = 0;
    uint8_t cmd = 0;

    struct motor_group_t *group = (struct motor_group_t *)user_data;
    struct motor_t *motor = 0;


    // LOG_ERR("%03x", frame->id);
    // LOG_HEXDUMP_INF(frame->data, frame->dlc, "RX RAW");

    if (frame->id == 0 && (frame->data[0] & 0x0F) == 0x01) {
        // maybe DaMaio Motor
        dm_rx_callback(dev, frame, user_data);

        return;
    }

    switch (frame->id)
    {
    case 0x204:
        // M15 current mode ACK
        id = frame->id - 0x200;
        idx = id - 1;
        motor = &group->motors[idx];
        motor->type = MT_M1505;
        break;

    case 0x0A1:
    case 0x0A2:
    case 0x0A3:
    case 0x0A4:
        // P1010A enable/disable
        id = frame->id - 0xA0;
        idx = id - 1;
        mode = frame->data[1];
        cmd = frame->data[2];
        LOG_DBG("P10 %d set enable, mode %d, cmd %d", id, mode, cmd);
        break;

    case 0X0B1:
    case 0X0B2:
    case 0X0B3:
    case 0X0B4:
        // P1010A save param/zero
        LOG_DBG("P10 %d set zero %d", id);
        break;

    case 0x081:
    case 0x082:
    case 0x083:
    case 0x084:
        // P1010A set param
        id = frame->id - 0x80;
        idx = id - 1;
        cmd = frame->data[1];
        u32val = *(uint32_t *)(frame->data + 2);
        motor = &group->motors[idx];
        motor->type = MT_P1010;
        LOG_DBG("P10 %d set param %d to 0x%02x", id, cmd, u32val);
        break;
    
    case 0x090 + 1:
    case 0x090 + 2:
    case 0x090 + 3:
    case 0x090 + 4:
        // P1010A get param
        id = frame->id - 0x90;
        idx = id - 1;
        cmd = frame->data[1];
        u32val = *(uint32_t *)(frame->data + 2);
        LOG_DBG("P10 %d get param %d to 0x%02x", id, cmd, u32val);
        break;

    case 0X32C + 1:
    case 0X32C + 2:
    case 0X32C + 3:
    case 0X32C + 4:
        LOG_DBG("M15 set time out: 0x%x", frame->id);
        break;

    case 0x451:
    case 0x452:
    case 0x453:
    case 0x454:
        // M1505A OTA Notify
        break;
#ifdef CONFIG_APP_ABUP_FOTA
    case 0x301:
    case 0x302:
    case 0x303:
    case 0x304:
    case 0x201:
    case 0x202:
    case 0x203:
        upgradeMotor_rxCallback(dev,frame,user_data);
        break;
#endif
    
    default:
        LOG_DBG("RECV 0x%03X", frame->id);
        LOG_HEXDUMP_DBG(frame->data, frame->dlc, "RX RAW");
        break;
    }
}

static void lk_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct motor_group_t *group = (struct motor_group_t *)user_data;
    struct motor_t *motor = 0;
    float dir_c = 1.0;

    uint8_t id = frame->id - 0x140;
    uint8_t idx = id - 1;
    uint8_t cmd;
    uint8_t temp;
    float iq;
    int16_t iq_val;
    int16_t speed;
    uint16_t encoder;

    // uint8 1C/LSB
    temp = frame->data[1];

    // int16 -2048 ~ 2048 == -33A ~ 33A
    iq_val = frame->data[3];
    iq_val = iq_val << 8 | frame->data[2];
    iq = (float)iq_val / 2048 * 33;

    // int16 1dps/LSB
    speed = frame->data[5];
    speed = speed << 8 | frame->data[4];

    // uint16 14bit 0~16383
    encoder = frame->data[7];
    encoder = encoder << 8 | frame->data[6];
    encoder = encoder;  // & 0x3FFF;

    motor = &group->motors[id - 1];
    motor->rx_count += 1;

    if (motor->direction == FORWARD) {
      dir_c = 1.0;
      motor->in.position = (float)encoder / UINT16_MAX * 2 * M_PI;
    } else {  // REVERSE
      dir_c = -1.0;
      motor->in.position = (float)(UINT16_MAX - encoder) / UINT16_MAX * 2 * M_PI;
    }

    motor->in.torque = dir_c * iq * 1.4f; // 1.4 Nm/A
    motor->in.velocity = dir_c * speed * 0.017453f / 10.0f; // deg/s to rad/s, 10:1 gear rate
    // motor->in.position = dir_c * encoder / UINT16_MAX * 2 * M_PI; // 14bit encoder to rad
    motor->in.position += motor->zero_angle_bias;

    // trans 0 ~ 2PI to -PI ~ +PI
    if (motor->in.position > M_PI)
        motor->in.position -= 2 * M_PI;

#if 0
    if (motor->rx_count % 1000 == 1 && id == 3 && group->type == LEFT) {
#if 0
        LOG_INF("ID: 0x%03X [0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]",
                frame->id,
                frame->data[0], frame->data[1], frame->data[2], frame->data[3],
                frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
#endif

#if 0
        char type = group->type == LEFT ? 'L' : 'R';
        LOG_INF("%c id: %d cmd: 0x%02x temp: %dC iq: %+0.3fA speed: %ddps encoder: %u",
                type, id, cmd, temp, iq, speed, encoder);
#endif
  }
#endif
}

static void p1010a_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct motor_group_t *group = (struct motor_group_t *)user_data;
    struct motor_t *motor = 0;
    float dir_c = 1.0;

    uint8_t id = frame->id - 0x50;
    uint8_t idx = id - 1;
    // uint8_t error = frame->data[6];
    // uint8_t mode = frame->data[7];
    int16_t value;

    float iq;
    float speed;
    uint16_t encoder;
    float voltage;

    value = frame->data[0];
    value = value << 8 | frame->data[1];
    speed = (float)value / 10;

    value = frame->data[2];
    value = value << 8 | frame->data[3];
    iq = (float)value / 100;

    value = frame->data[4];
    value = value << 8 | frame->data[5];
    encoder = value & 0x7FFF;

    value = frame->data[6];
    value = value << 8 | frame->data[7];
    voltage = (float)value / 10;

    motor = &group->motors[idx];
    motor->rx_count += 1;

    if (motor->direction == REVERSE) {
        dir_c = -1.0;
        motor->in.position = (float)(INT16_MAX - encoder) / INT16_MAX * 2 * M_PI;
    } else {
        dir_c = 1.0;
        motor->in.position = (float)(encoder) / INT16_MAX * 2 * M_PI;
    }

    motor->in.torque = dir_c * iq * P1010A_TORQUE_CONSTANT; // gearbox output torque
    motor->in.velocity = dir_c * speed * 0.1047198; // rpm to rad/s

    motor->in.position += motor->zero_angle_bias;

    // trans 0 ~ 2PI to -PI ~ +PI
    while (motor->in.position > M_PI)
        motor->in.position -= 2 * M_PI;

    while (motor->in.position < -M_PI)
        motor->in.position += 2 * M_PI;

    // make RL happy
    if (idx == M_HIP && motor->in.position < (-135.0 / 180.0) * M_PI)
        motor->in.position += 2 * M_PI;

    motor->in_ex.voltage = voltage;

#if 0
    if (motor->rx_count % 1000 == 1) {
#if 0
        LOG_INF("ID: 0x%03X [0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]",
                frame->id,
                frame->data[0], frame->data[1], frame->data[2], frame->data[3],
                frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
#endif

#if 0
        LOG_INF("id: %d mode: %d error: %d iq: %+0.3fA speed: %+0.2frpm encoder: %+0.2fdeg",
                id, 0, 0, iq, speed, encoder);
#endif
    }
#endif
}

static void m1505a_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
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
    // -5200 ~ 5200, -520rpm ~ 520rpm
    speed = (float)value / 10.0f;

    value = frame->data[2];
    value = value << 8 | frame->data[3];
    // -32767 ~ 32767, -55A ~ 55A
    iq = (float)value / INT16_MAX * 55.0f;

    value = frame->data[4];
    value = value << 8 | frame->data[5];
    value = value & 0x7FFF;
    // encoder = (float)value / INT16_MAX * 360.0;

    // encoder = encoder > 180.0 ? encoder - 360.0 : encoder;

    // motor = &group->motors[idx];
    motor = &group->motors[M_ANKLE]; // hardcode
    motor->rx_count += 1;

    if (motor->direction == REVERSE) {
        dir_c = -1.0;
        motor->in.position = (float)(INT16_MAX - value) / INT16_MAX * 2 * M_PI;
    } else {
        dir_c = 1.0;
        motor->in.position = (float)(value) / INT16_MAX * 2 * M_PI;
    }

    value = frame->data[6];
    if (value != 0x3C) // ignore timeout
        motor->status = value;

    motor->in.torque = dir_c * iq * M1505A_TORQUE_CONSTANT;
    motor->in.velocity = dir_c * speed * 0.1047198;    // rpm to rad/s
    // motor->in.position = dir_c * encoder * 0.0174533;  // deg to rad

    // trans 0 ~ 2PI to -PI ~ +PI
    if (motor->in.position > M_PI)
        motor->in.position -= 2 * M_PI;

#if 1
    if (motor->rx_count % 1000 == 1) {
#if 0
        LOG_INF("ID: 0x%03X [0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]",
                frame->id,
                frame->data[0], frame->data[1], frame->data[2], frame->data[3],
                frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
#endif

#if 0
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
    group->motors[M_ANKLE].direction = FORWARD;

    group->motors[M_ABAD].type = MT_MG10015;
    group->motors[M_HIP].type = MT_MG10015;
    group->motors[M_KNEE].type = MT_MG10015;

    group->motors[M_ABAD].zero_angle_bias = +(45.0f / 360.0f) * 2 * M_PI;
    group->motors[M_HIP].zero_angle_bias = +(180.0f / 360.0f) * 2 * M_PI;
    group->motors[M_KNEE].zero_angle_bias = -(49.73f / 360.0f)  * 2 * M_PI;
    group->motors[M_ANKLE].zero_angle_bias = 0;

    group = wrap_get_motor_group(wrapper, RIGHT);

    group->type = RIGHT;
    group->motors[M_ANKLE].direction = REVERSE;

    group->motors[M_ABAD].type = MT_MG10015;
    group->motors[M_HIP].type = MT_MG10015;
    group->motors[M_KNEE].type = MT_MG10015;

    group->motors[M_ABAD].zero_angle_bias = -(45.0f / 360.0f) * 2 * M_PI;
    group->motors[M_HIP].zero_angle_bias = +(180.0f / 360.0f) * 2 * M_PI;
    group->motors[M_KNEE].zero_angle_bias = -(49.73f / 360.0f)  * 2 * M_PI;
    group->motors[M_ANKLE].zero_angle_bias = 0;
}

static void m15_set_param(struct motor_group_t *group, uint8_t motor_id, uint32_t param_id, uint16_t value)
{
    int ret;
    struct can_frame frame;

    frame.flags = 0;
    frame.id = param_id; 
    frame.dlc = 8;

    memset(frame.data, 0, 8);

    frame.data[0] = motor_id;
    frame.data[1] = 0x10; // set value
    frame.data[2] = 0x01; // write
    frame.data[3] = 0x0000FF00U & value >> 8;
    frame.data[4] = 0x000000FFU & value;

#if 0
    LOG_INF("set param: %02X %02X %08X ", motor_id, param_id, value);
    LOG_HEXDUMP_INF(frame.data, frame.dlc, "TX RAW");
#endif

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("M15 set param failed: %d, motor id: %d param id: %d value: %d",
                ret, motor_id, param_id, value);
    }
}

static void m15_set_mode(struct motor_group_t *group, uint8_t mode)
{
    int ret;
    struct can_frame frame;
    uint8_t *data = frame.data;

    frame.flags = 0;
    frame.id = 0x105;
    frame.dlc = 8;

    memset(frame.data, 0, 8);

    frame.data[0] = mode;
    frame.data[1] = mode;
    frame.data[2] = mode;
    frame.data[3] = mode;

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("M15 set mode failed: %d, group id: %d mode: %d",
                ret, group->type, mode);
    }
}

static void m15_set_data_frequency(struct motor_group_t *group, uint32_t frequency)
{
    int ret;
    struct can_frame frame;
    uint8_t *data = frame.data;
    uint8_t delay_ms = 0x7F & (1000 / frequency);

    frame.flags = 0;
    frame.id = 0x106;
    frame.dlc = 8;

    memset(frame.data, 0, 8);

    frame.data[0] = delay_ms;
    frame.data[1] = delay_ms;
    frame.data[2] = delay_ms;
    frame.data[3] = delay_ms;

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("M15 set data frequency failed: %d, group id: %d mode: %d",
                ret, group->type, frequency);
    }  
}

#define M15_TIMEOUT_KEY 0x10B

static void m15_motor_init(struct wrapper_robot_t *wrapper)
{
    int ret = 0;
    struct motor_group_t *group = 0;


    group = wrap_get_motor_group(wrapper, LEFT);

    m15_set_mode(group, 1);
    m15_set_param(group, 4, M15_TIMEOUT_KEY, 100);

    group = wrap_get_motor_group(wrapper, RIGHT);

    m15_set_mode(group, 1);
    m15_set_param(group, 4, M15_TIMEOUT_KEY, 100);

    return ret;
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static inline uint32_t rs1_encode_id(uint8_t cmd, uint16_t val16, uint8_t val8)
{
    uint32_t id = 0;

    id = cmd & 0x1F;
    id = (id << 16) | val16;
    id = (id << 8) | val8;

    return id;
}


#define RS1_KP_MIN 0.0f
#define RS1_KP_MAX 500.0f
#define RS1_KD_MIN 0.0f
#define RS1_KD_MAX 5.0f

#define RS1_P_MIN -12.56f
#define RS1_P_MAX 12.56f
#define RS1_V_MIN -60.00f
#define RS1_V_MAX 60.00f
#define RS1_T_MIN -12.00f
#define RS1_T_MAX 12.00f

#define Z1_KP_MIN 0.0f
#define Z1_KP_MAX 5000.0f
#define Z1_KD_MIN 0.0f
#define Z1_KD_MAX 100.0f

#define Z1_P_MIN -12.56f
#define Z1_P_MAX 12.56f
#define Z1_V_MIN -15.00f
#define Z1_V_MAX 15.00f
#define Z1_T_MIN -120.00f
#define Z1_T_MAX 120.00f

static void z1_rs1_stuff_broadcast(struct can_frame *frame, struct motor_group_t *group, const int idx)
{
    uint8_t *data = frame->data;
    struct motor_t *motor = 0;
    uint16_t p_out, v_out, t_out, kp_out, kd_out;
    uint8_t motor_id = idx == M_ANKLE ? 0x7F : idx + 1;
    uint8_t cmd = 0x01; // locomotion mode
    uint8_t zero[] = {0x7F, 0xFF, 0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00};

    float kp_max, kd_max;
    float p_max, p_min, v_max, v_min, t_max, t_min;
    float position, velocity, torque;
    p_min = Z1_P_MIN, p_max = Z1_P_MAX;
    v_min = Z1_V_MIN, v_max = Z1_V_MAX;
    t_min = Z1_T_MIN, t_max = Z1_T_MAX;
    kp_max = Z1_KP_MAX, kd_max = Z1_KD_MAX;

    if (motor_id == 0x7F) {
        p_min = RS1_P_MIN, p_max = RS1_P_MAX;
        v_min = RS1_V_MIN, v_max = RS1_V_MAX;
        t_min = RS1_T_MIN, t_max = RS1_T_MAX;
        kp_max = RS1_KP_MAX, kd_max = RS1_KD_MAX;
    }

    motor = &group->motors[idx];
    position = motor->out.position;
    velocity = motor->out.velocity;

#if 0
    float torque_max = 35.0;
    if (motor->out.torque > torque_max)
        motor->out.torque = torque_max;
    if (motor->out.torque < -torque_max)
        motor->out.torque = -torque_max;
#endif

    torque = motor->out.torque;

#if 1
    // torque filter, eliminate torque jitter
    // but i dont know why
    
    float x = 0.333;
    torque = torque * x + motor->in.torque * (1.0 - x);
#endif

    // trans to 0PI ~ 2PI
    if (position < 0)
        position += 2 * M_PI;

    position -= motor->zero_angle_bias;

    // wrapping overflow, range 0 ~ 2PI
    while (position > 2 * M_PI)
        position -= 2 * M_PI;

    while (position < 0)
        position += 2 * M_PI;

    // mirror motor direction
    if (motor->direction == REVERSE) {
        position = 2 * M_PI - position;
        velocity = -velocity;
        torque = -torque;
    }
    
    p_out = float_to_uint(position, p_min, p_max, 16);
    v_out = float_to_uint(velocity, v_min, v_max, 16);
    t_out = float_to_uint(torque, t_min, t_max, 16);

    kp_out = float_to_uint(motor->out.kp, 0.0f, kp_max, 16);
    kd_out = float_to_uint(motor->out.kd, 0.0f, kd_max, 16);

    frame->flags = CAN_FILTER_IDE;
    frame->id = rs1_encode_id(cmd, t_out, motor_id);
    frame->dlc = 8;

    // memcpy((void *)frame->data, zero, 8);

    frame->data[0] = (0xFF00 & p_out) >> 8;
    frame->data[1] = 0x00FF & p_out;
    frame->data[2] = (0xFF00 & v_out) >> 8;
    frame->data[3] = 0x00FF & v_out;
    frame->data[4] = (0xFF00 & kp_out) >> 8;
    frame->data[5] = 0x00FF & kp_out;
    frame->data[6] = (0xFF00 & kd_out) >> 8;
    frame->data[7] = 0x00FF & kd_out;
}

static void z1_rs1_rx_ota_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct motor_group_t *group = (struct motor_group_t *)user_data;
    struct motor_t *motor = 0;
    float position, velocity, torque;

    // decode can id
    uint8_t cmd = (frame->id & 0x1F000000) >> 24;
    uint8_t status = (frame->id & 0x00FF0000) >> 16;
    uint8_t motor_id = (frame->id & 0x0000FF00) >> 8;
    uint8_t host_id = (frame->id & 0x000000FF) >> 0;

    int idx = motor_id == 0x7F ? motor_id = 3 : motor_id - 1;

    motor = &group->motors[idx];

    LOG_INF("%d %d %d", cmd, status, motor_id);

#if 0
    if (cmd == 0) {
        LOG_DBG("motor: %02x mcu-id: %02x %02x %02x %02x %02x %02x %02x %02x",
                motor_id,
                frame->data[0], frame->data[1], frame->data[2], frame->data[3],
                frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
        memcpy(motor->mcu_id, frame->data, 8);
    } else if (cmd == 11 && status == 0x0) {
        LOG_INF("ota state: %d status: %d", motor->ota_state, motor->ota_status);
        // start ota ack
        // start ota success, next to send firmware info
        motor->ota_state = 12;

    } else if (cmd == 12 && status == 0x0) {
        // send firmware info ack
        // send firmware info success, next to send firmware data
        motor->ota_state = 13;
        motor->ota_offset = 0;

    } else if ((cmd == 13 || cmd == 14) && status == 0x0) {
        // send firmware data/end ack
        // send firmware data success, continue send firmware data or finish ota
        motor->ota_state = 13;
        motor->ota_offset += 1;

    } else if (status != 0x0 ) {
        motor->ota_status = status;
    }

#endif
    return;
}

static void z1_rs1_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct motor_group_t *group = (struct motor_group_t *)user_data;
    struct motor_t *motor = 0;
    float position, velocity, torque;

    uint16_t value;

    // decode can id
    uint8_t cmd = (frame->id & 0x1F000000) >> 24;
    uint8_t status = (frame->id & 0x00FF0000) >> 16;
    uint8_t motor_id = (frame->id & 0x0000FF00) >> 8;
    uint8_t host_id = (frame->id & 0x000000FF) >> 0;

    // gear ratio 7.75

    int idx = motor_id == 0x7F ? motor_id = 3 : motor_id - 1;

    motor = &group->motors[idx];
    motor->rx_count += 1;

    if (cmd != 2) {
        z1_rs1_rx_ota_callback(dev, frame, user_data);
        return;
    }

    float p_min, p_max, v_min, v_max, t_min, t_max;
    // position, -4PI ~ 4PI
    p_min = Z1_P_MIN, p_max = Z1_P_MAX;
    // velocity, -15rad/s ~ 15rad/s
    v_min = Z1_V_MIN, v_max = Z1_V_MAX;
    // torque, -120Nm ~ 120Nm
    t_min = Z1_T_MIN, t_max = Z1_T_MAX;

    if (motor_id == 0x7F) {
        // position, -4PI ~ 4PI
        p_min = RS1_P_MIN, p_max = RS1_P_MAX;
        // velocity, -60rad/s ~ 60rad/s
        v_min = RS1_V_MIN, v_max = RS1_V_MAX;
        // torque, -12Nm ~ 12Nm
        t_min = RS1_T_MIN, t_max = RS1_T_MAX;
    }


    value = frame->data[0];
    value = value << 8 | frame->data[1];
    position = uint_to_float(value, p_min, p_max, 16);

    value = frame->data[2];
    value = value << 8 | frame->data[3];
    velocity = uint_to_float(value, v_min, v_max, 16);

    value = frame->data[4];
    value = value << 8 | frame->data[5];
    torque = uint_to_float(value, t_min, t_max, 16);

    motor->type = motor_id == 0x7F ? MT_RS1 : MT_Z1;

    // position input is 0PI ~ 2PI

    // mirror motor direction
    if (motor->direction == REVERSE) {
        position = 2 * M_PI - position;
        velocity = -velocity;
        torque = -torque;
    }

    // zero position offset
    position += motor->zero_angle_bias;

    // wrapping overflow, range -PI ~ +PI
    while (position > M_PI)
        position -= 2 * M_PI;

    while (position < -M_PI)
        position += 2 * M_PI;

#if 1
    motor->in.position = position;
    motor->in.velocity = velocity;
    motor->in.torque = torque;

#else

    float position_x = 0.500;
    float velocity_x = 0.500;
    float torque_x = 1.000;

    motor->in.position = motor->ref_in.position * (1.0 - position_x) + position * position_x;
    motor->in.velocity = motor->ref_in.velocity * (1.0 - velocity_x) + velocity * velocity_x;
    motor->in.torque = motor->ref_in.torque * (1.0 - torque_x) + torque * torque_x;

    // motor->ref_in.position = position;
    // motor->ref_in.velocity = velocity;
    // motor->ref_in.torque = torque;

    motor->ref_in.position = motor->in.position;
    motor->ref_in.velocity = motor->in.velocity;
    motor->ref_in.torque = motor->in.torque;
#endif

    motor->status = status;
}

static void z1_rs1_set_param(struct motor_group_t *group, uint16_t index, void *value)
{
    int ret;
    struct can_frame frame;
    uint8_t cmd = 18; // set_param
    uint8_t ids[4] = {0x01, 0x02, 0x03, 0x7F};

    for (int i = 0; i < sizeof(ids); i++) {
    frame.flags = CAN_FRAME_IDE;
    frame.id = rs1_encode_id(cmd, 0, ids[i]);
    frame.dlc = 8;

    frame.data[2] = 0;
    frame.data[3] = 0;
    memcpy(frame.data, &index, 2);
    memcpy(frame.data + 4, value, 4);

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("z1/rs1 set param failed: %d, group id: %d, motor id: %d", ret, group->type, i);
    }
    k_msleep(1);
    }
}

static void rs1_get_param(struct motor_group_t *group, uint16_t index)
{
    int ret;
    struct can_frame frame;
    uint8_t cmd = 17; // get_param

    frame.flags = CAN_FRAME_IDE;
    frame.id = rs1_encode_id(cmd, 0, 0x7F);
    frame.dlc = 8;

    memset(frame.data, 0, 8);
    memcpy(frame.data, &index, 2);

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("rs1 get param failed: %d, group id: %d", ret, group->type);
    }
}

static void z1_rs1_set_zero(struct motor_group_t *group, uint16_t id)
{
    int ret;
    struct can_frame frame;
    uint8_t cmd = 6; // set zero

    frame.flags = CAN_FRAME_IDE;
    frame.id = rs1_encode_id(cmd, 0, id);
    frame.dlc = 8;

    memset(frame.data, 0, 8);
    frame.data[0] = 1;

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("rs1 set zero failed: %d, group id: %d", ret, group->type);
    }

    k_sleep(K_MSEC(2));
}

static void z1_rs1_get_mcu_id(struct motor_group_t *group, uint16_t id)
{
    int ret;
    struct can_frame frame;
    uint8_t cmd = 0; // get mcu id

    frame.flags = CAN_FRAME_IDE;
    frame.id = rs1_encode_id(cmd, 0, id);
    frame.dlc = 8;

    memset(frame.data, 0, 8);

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("rs1 get mcu id failed: %d, group id: %d", ret, group->type);
    }

    k_sleep(K_MSEC(2));
}

static void z1_rs1_start_ota(struct motor_group_t *group, uint16_t idx)
{
    int ret;
    struct can_frame frame;
    struct motor_t *motor = &group->motors[idx];
    uint8_t cmd = 11; // start ota

    frame.flags = CAN_FRAME_IDE;
    frame.id = rs1_encode_id(cmd, 0, idx + 1);
    frame.dlc = 8;

#if 0
    memset(frame.data, 0, 8);
    memcpy(frame.data, motor->mcu_id, 8);

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("rs1 start ota failed: %d, group id: %d", ret, group->type);
    }
#endif
}

static void z1_rs1_send_firmware_info(struct motor_group_t *group, uint16_t id)
{
}

static void z1_rs1_send_firmware_data(struct motor_group_t *group, uint16_t id)
{
}

static void z1_rs1_set_enable(struct motor_group_t *group, bool enabled)
{
    int ret;
    struct can_frame frame;
    uint8_t cmd = 4; // diable
    uint8_t ids[4] = {0x01, 0x02, 0x03, 0x7F};

    if (enabled)
        cmd = 3; // enable

    for (int i = 0; i < sizeof(ids); i++) {
    frame.flags = CAN_FRAME_IDE;
    frame.id = rs1_encode_id(cmd, 0, ids[i]);
    frame.dlc = 8;

    memset(frame.data, 0, 8);

    if (enabled == false)
        frame.data[0] = 1;

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("z1/rs1 set enable failed: %d, group id: %d, motor id: %d", ret, group->type, i);
    }
    k_msleep(1);
    }
}

static void z1_rs1_motor_set_zero(struct wrapper_robot_t *wrapper, int i)
{
    int ret;
    struct motor_group_t *group = 0;
    uint8_t ids[4] = {0x01, 0x02, 0x03, 0x7F};

    group = wrap_get_motor_group(wrapper, i);

    z1_rs1_set_enable(group, false);
    
    for (int i = 0; i < sizeof(ids); i++) {
        z1_rs1_set_zero(group, ids[i]);
    }
}

static void z1_rs1_motor_get_mcu_id(struct wrapper_robot_t *wrapper, int i)
{
    int ret;
    struct motor_group_t *group = 0;
    uint8_t ids[4] = {0x01, 0x02, 0x03, 0x7F};

    group = wrap_get_motor_group(wrapper, i);

    z1_rs1_set_enable(group, false);
    
    for (int i = 0; i < sizeof(ids); i++) {
        z1_rs1_get_mcu_id(group, ids[i]);
    } 
}



#define RS1_CAN_TIMEOUT 0x200C
#define RS1_SDP_FILT_GAIN 0x2017
#define Z1_CURRENT_KP_KEY 0x7010
#define Z1_CURRENT_KI_KEY 0x7011
#define Z1_CURRENT_FILT_GAIN 0x7014
#define Z1_CURRENT_FILT_GAIN 0x7014
#define Z1_SPEED_FILT_GAIN 0x7021

static void z1_rs1_motor_init(struct wrapper_robot_t *wrapper)
{
    int ret = 0;
    struct motor_group_t *group = 0;

    z1_rs1_motor_get_mcu_id(wrapper, LEFT);
    z1_rs1_motor_get_mcu_id(wrapper, RIGHT);

    // const uint32_t can_timeout = 12 * 1000u; // 12k = 1s
    // const float speed_filt_gain = 0.01f;
    const float current_kp = 0.050;
    const float current_ki = 0.040;
    const float current_filt_gain = 0.060;
    const float speed_filt_gain = 0.030;

    group = wrap_get_motor_group(wrapper, LEFT);

    z1_rs1_set_enable(group, 0);
    z1_rs1_set_param(group, Z1_CURRENT_KP_KEY, (void *)&current_kp);
    z1_rs1_set_param(group, Z1_CURRENT_KI_KEY, (void *)&current_ki);
    z1_rs1_set_param(group, Z1_CURRENT_FILT_GAIN, (void *)&current_filt_gain);
    z1_rs1_set_param(group, Z1_SPEED_FILT_GAIN, (void *)&speed_filt_gain);

    z1_rs1_set_enable(group, 1);

    group = wrap_get_motor_group(wrapper, RIGHT);

    z1_rs1_set_enable(group, 0);
    z1_rs1_set_param(group, Z1_CURRENT_KP_KEY, (void *)&current_kp);
    z1_rs1_set_param(group, Z1_CURRENT_KI_KEY, (void *)&current_ki);
    z1_rs1_set_param(group, Z1_CURRENT_FILT_GAIN, (void *)&current_filt_gain);
    z1_rs1_set_param(group, Z1_SPEED_FILT_GAIN, (void *)&speed_filt_gain);

    z1_rs1_set_enable(group, 1);

    return ret;
}

#define DM80_KP_MIN 0.0f
#define DM80_KP_MAX 500.0f
#define DM80_KD_MIN 0.0f
#define DM80_KD_MAX 5.0f

#define DM80_P_MIN -12.50f
#define DM80_P_MAX 12.50f
#define DM80_V_MIN -45.00f
#define DM80_V_MAX 45.00f
#define DM80_T_MIN -20.00f
#define DM80_T_MAX 20.00f

static void dm_stuff_broadcast(struct can_frame *frame, struct motor_group_t *group, const int idx)
{
    uint8_t *data = frame->data;
    struct motor_t *motor = 0;
    uint16_t p_out, v_out, t_out, kp_out, kd_out;
    uint8_t motor_id = idx == M_ANKLE ? 0x7F : idx + 1;
    uint8_t cmd = 0x01; // locomotion mode

    float kp_max, kd_max;
    float p_max, p_min, v_max, v_min, t_max, t_min;
    float position, velocity, torque;

    p_min = DM80_P_MIN, p_max = DM80_P_MAX;
    v_min = DM80_V_MIN, v_max = DM80_V_MAX;
    t_min = DM80_T_MIN, t_max = DM80_T_MAX;
    kp_max = DM80_KP_MAX, kd_max = DM80_KD_MAX;

    motor = &group->motors[idx];
    position = motor->out.position;
    velocity = motor->out.velocity;
    torque = motor->out.torque;

    // trans to 0PI ~ 2PI
    if (position < 0)
        position += 2 * M_PI;

    position -= motor->zero_angle_bias;

    // wrapping overflow, range 0 ~ 2PI
    while (position > 2 * M_PI)
        position -= 2 * M_PI;

    while (position < 0)
        position += 2 * M_PI;

    // mirror motor direction
    if (motor->direction == REVERSE) {
        position = 2 * M_PI - position;
        velocity = -velocity;
        torque = -torque;
    }

    p_out = float_to_uint(position, p_min, p_max, 16);
    v_out = float_to_uint(velocity, v_min, v_max, 12);
    t_out = float_to_uint(torque, t_min, t_max, 12);

    kp_out = float_to_uint(motor->out.kp, 0.0f, kp_max, 12);
    kd_out = float_to_uint(motor->out.kd, 0.0f, kd_max, 12);


    frame->flags = 0;
    frame->id = 1;
    frame->dlc = 8;

    frame->data[0] = (0xFF00 & p_out) >> 8;
    frame->data[1] = 0x00FF & p_out;
    frame->data[2] = (0x0FF0 & v_out) >> 4;
    frame->data[3] = (0x000F & v_out) << 4 | (0x0F00 & kp_out) >> 8;
    frame->data[4] = 0x00FF & kp_out;
    frame->data[5] = (0x0FF0 & kd_out) >> 4;
    frame->data[6] = (0x000F & kd_out) << 4 | (0x0F00 & t_out) >> 8;
    frame->data[7] = 0x00FF & t_out;
}


static void dm_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct motor_group_t *group = (struct motor_group_t *)user_data;
    struct motor_t *motor = 0;
    float position, velocity, torque;

    uint16_t value;

    uint8_t id = frame->data[0] & 0x0F;
    uint8_t err = (frame->data[0] & 0xF0) >> 4;

    // only support dm8006 as wheel
    if (err == 1 && id == 1) {
        motor = &group->motors[3];
        motor->rx_count += 1;
        motor->type = MT_DM8006;
        motor->status = err != 1 ? err : 0;
    } else {
        return;
    }

    float p_min, p_max, v_min, v_max, t_min, t_max;
    p_min = DM80_P_MIN, p_max = DM80_P_MAX;
    v_min = DM80_V_MIN, v_max = DM80_V_MAX;
    t_min = DM80_T_MIN, t_max = DM80_T_MAX;

    value = frame->data[1];
    value = value << 8 | frame->data[2];
    position = uint_to_float(value, p_min, p_max, 16);

    value = frame->data[3];
    value = value << 8 | frame->data[4];
    value = value >> 4;
    velocity = uint_to_float(value, v_min, v_max, 12);

    value = frame->data[4];
    value = value << 8 | frame->data[5];
    value = value & 0x0FFF;
    torque = uint_to_float(value, t_min, t_max, 12);

    motor->in_ex.temp_drive = frame->data[6];
    motor->in_ex.temp_winding = frame->data[7];

    // mirror motor direction
    if (motor->direction == REVERSE) {
        position = 2 * M_PI - position;
        velocity = -velocity;
        torque = -torque;
    }

    // zero position offset
    position += motor->zero_angle_bias;

    // wrapping overflow, range -PI ~ +PI
    while (position > M_PI)
        position -= 2 * M_PI;

    while (position < -M_PI)
        position += 2 * M_PI;

    motor->in.position = position;
    motor->in.velocity = velocity;
    motor->in.torque = torque;

    return;
}

static void dm_set_enable(struct motor_group_t *group, bool enabled)
{
    int ret;
    struct can_frame frame;
    uint8_t cmd = 0xFD; // diable
    uint8_t ids[1] = {0x01};

    if (enabled)
        cmd = 0xFC; // enable

    for (int i = 0; i < sizeof(ids); i++) {
        frame.flags = 0;
        frame.id = ids[i];
        frame.dlc = 8;

        memset(frame.data, 0xFF, 8);

        frame.data[7] = cmd;

        ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
        if (ret != 0) {
            group->bus_tx_error += 1;
            LOG_ERR("dm_motor set enable failed: %d, group id: %d, motor id: %d", ret, group->type, i);
        }

        k_msleep(1);
    }
}

static void dm_motor_init(struct wrapper_robot_t *wrapper)
{
    struct motor_group_t *group = 0;

    group = wrap_get_motor_group(wrapper, LEFT);

    dm_set_enable(group, 0);
    dm_set_enable(group, 1);

    group = wrap_get_motor_group(wrapper, RIGHT);

    dm_set_enable(group, 0);
    dm_set_enable(group, 1);

    return;
}

static void p1010a_set_enable(struct motor_group_t *group, bool enabled)
{
    int ret;
    struct can_frame frame;
    uint8_t val = 0x01; // diable

    frame.flags = 0;
    frame.id = 0x38; 
    frame.dlc = 8;
    
    if (enabled)
        val = 0x02; // enable

    memset(frame.data, val, 8);

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("P10 set enable failed: %d, group id: %d", ret, group->type);
    }

    k_sleep(K_MSEC(2));
}

static void p1010a_set_zero(struct motor_group_t *group)
{
    int ret;
    struct can_frame frame;

    frame.flags = 0;
    frame.id = 0x39; 
    frame.dlc = 8;

    memset(frame.data, 0, 8);

    frame.data[1] = 1; // set zero

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("P10 set zero failed: %d, group id: %d", ret, group->type);
    }

    k_sleep(K_MSEC(2));
}

static void p1010a_set_param(struct motor_group_t *group, uint8_t motor_id, uint8_t param_id, uint32_t value)
{
    int ret;
    struct can_frame frame;

    frame.flags = 0;
    frame.id = 0x36; 
    frame.dlc = 8;

    memset(frame.data, 0, 8);

    frame.data[0] = motor_id;
    frame.data[1] = param_id;
    frame.data[2] = (0x000000FFU & value);
    frame.data[3] = (0x0000FF00U & value) >> 8;
    frame.data[4] = (0x00FF0000U & value) >> 16;
    frame.data[5] = (0xFF000000U & value) >> 24;

#if 0
    LOG_INF("set param: %02X %02X %08X ", motor_id, param_id, value);
    LOG_HEXDUMP_INF(frame.data, frame.dlc, "TX RAW");
#endif

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("P10 set param failed: %d, motor id: %d param key: %d value: %d",
                ret, motor_id, param_id, value);
    }

    k_sleep(K_MSEC(1));
}

static void p1010a_get_param(struct motor_group_t *group, uint8_t motor_id, uint8_t param_id)
{
    int ret;
    struct can_frame frame;

    frame.flags = 0;
    frame.id = 0x37U;
    frame.dlc = 8;

    memset(frame.data, 0, 8);

    frame.data[0] = motor_id;
    frame.data[1] = param_id;

#if 0
    LOG_INF("get param: %02X %02X ", motor_id, param_id);
    LOG_HEXDUMP_INF(frame.data, frame.dlc, "TX RAW");
#endif    

    ret = can_send(group->bus, &frame, K_MSEC(10), any_tx_callback, (void *)group);
    if (ret != 0) {
        group->bus_tx_error += 1;
        LOG_ERR("P10 get param failed: %d, motor id: %d param key: %d",
                ret, motor_id, param_id);
    }

    k_sleep(K_MSEC(1));
}

#define P10_MODE_KEY 28U
#define P10_MODE_CURRENT 2U
#define P10_BUS_FAULT_KEY 22U
#define P10_TIME_OUT_KEY 47U
#define P10_CURRENT_ACCEL_KEY 85U
#define P10_OVERVOLTAGE_KEY 116U
#define P10_UNDERVOLTAGE_KEY 117U
#define P10_PHASE_RESISTANCE_KEY 118U
#define P10_PHASE_INDUCTANCE_KEY 119U
#define P10_CURRENT_BANDWIDTH_KEY 120U


static void p1010a_motor_init(struct wrapper_robot_t *wrapper)
{
    int ret;
    struct motor_group_t *group = 0;

    group = wrap_get_motor_group(wrapper, LEFT);

    p1010a_set_enable(group, false);

    p1010a_set_param(group, 1, 106, 4);
    p1010a_set_param(group, 2, 106, 4);
    p1010a_set_param(group, 3, 106, 4);
    p1010a_set_param(group, 4, 106, 4);

    p1010a_set_param(group, 1, P10_CURRENT_BANDWIDTH_KEY, 1600);
    p1010a_set_param(group, 2, P10_CURRENT_BANDWIDTH_KEY, 1600);
    p1010a_set_param(group, 3, P10_CURRENT_BANDWIDTH_KEY, 1600);

    // set current mode
    p1010a_set_param(group, 1, P10_MODE_KEY, P10_MODE_CURRENT);
    p1010a_set_param(group, 2, P10_MODE_KEY, P10_MODE_CURRENT);
    p1010a_set_param(group, 3, P10_MODE_KEY, P10_MODE_CURRENT);

    p1010a_set_param(group, 1, P10_CURRENT_ACCEL_KEY, 6000);
    p1010a_set_param(group, 2, P10_CURRENT_ACCEL_KEY, 6000);
    p1010a_set_param(group, 3, P10_CURRENT_ACCEL_KEY, 6000);

    p1010a_set_param(group, 1, P10_PHASE_RESISTANCE_KEY, 155);
    p1010a_set_param(group, 2, P10_PHASE_RESISTANCE_KEY, 155);
    p1010a_set_param(group, 3, P10_PHASE_RESISTANCE_KEY, 155);

    p1010a_set_param(group, 1, P10_PHASE_INDUCTANCE_KEY, 170);
    p1010a_set_param(group, 2, P10_PHASE_INDUCTANCE_KEY, 170);
    p1010a_set_param(group, 3, P10_PHASE_INDUCTANCE_KEY, 170);

    // under voltage 33V, battery cell 2.75V
    p1010a_set_param(group, 1, P10_UNDERVOLTAGE_KEY, 33U);
    p1010a_set_param(group, 2, P10_UNDERVOLTAGE_KEY, 33U); 
    p1010a_set_param(group, 3, P10_UNDERVOLTAGE_KEY, 33U);

    // set time out
    p1010a_set_param(group, 1, P10_TIME_OUT_KEY, 1000U);
    p1010a_set_param(group, 2, P10_TIME_OUT_KEY, 1000U); 
    p1010a_set_param(group, 3, P10_TIME_OUT_KEY, 1000U);

    // set bus fault
    p1010a_set_param(group, 1, P10_BUS_FAULT_KEY, 0U);
    p1010a_set_param(group, 2, P10_BUS_FAULT_KEY, 0U);
    p1010a_set_param(group, 3, P10_BUS_FAULT_KEY, 0U);

    p1010a_set_enable(group, true);


    group = wrap_get_motor_group(wrapper, RIGHT);

    p1010a_set_enable(group, false);

    p1010a_set_param(group, 1, 106, 4);
    p1010a_set_param(group, 2, 106, 4);
    p1010a_set_param(group, 3, 106, 4);
    p1010a_set_param(group, 4, 106, 4);

    p1010a_set_param(group, 1, P10_CURRENT_BANDWIDTH_KEY, 1600);
    p1010a_set_param(group, 2, P10_CURRENT_BANDWIDTH_KEY, 1600);
    p1010a_set_param(group, 3, P10_CURRENT_BANDWIDTH_KEY, 1600);
    
    // set current mode
    p1010a_set_param(group, 1, P10_MODE_KEY, P10_MODE_CURRENT);
    p1010a_set_param(group, 2, P10_MODE_KEY, P10_MODE_CURRENT);
    p1010a_set_param(group, 3, P10_MODE_KEY, P10_MODE_CURRENT);

    p1010a_set_param(group, 1, P10_CURRENT_ACCEL_KEY, 6000);
    p1010a_set_param(group, 2, P10_CURRENT_ACCEL_KEY, 6000);
    p1010a_set_param(group, 3, P10_CURRENT_ACCEL_KEY, 6000);

    p1010a_set_param(group, 1, P10_PHASE_RESISTANCE_KEY, 155);
    p1010a_set_param(group, 2, P10_PHASE_RESISTANCE_KEY, 155);
    p1010a_set_param(group, 3, P10_PHASE_RESISTANCE_KEY, 155);

    p1010a_set_param(group, 1, P10_PHASE_INDUCTANCE_KEY, 170);
    p1010a_set_param(group, 2, P10_PHASE_INDUCTANCE_KEY, 170);
    p1010a_set_param(group, 3, P10_PHASE_INDUCTANCE_KEY, 170);

    // under voltage 33V, battery cell 2.75V
    p1010a_set_param(group, 1, P10_UNDERVOLTAGE_KEY, 33U); 
    p1010a_set_param(group, 2, P10_UNDERVOLTAGE_KEY, 33U);
    p1010a_set_param(group, 3, P10_UNDERVOLTAGE_KEY, 33U);

    // set time out
    p1010a_set_param(group, 1, P10_TIME_OUT_KEY, 1000U);
    p1010a_set_param(group, 2, P10_TIME_OUT_KEY, 1000U); 
    p1010a_set_param(group, 3, P10_TIME_OUT_KEY, 1000U);

    // set bus fault
    p1010a_set_param(group, 1, P10_BUS_FAULT_KEY, 0U);
    p1010a_set_param(group, 2, P10_BUS_FAULT_KEY, 0U);
    p1010a_set_param(group, 3, P10_BUS_FAULT_KEY, 0U);

    p1010a_set_enable(group, true);
}

static void p1010a_motor_set_zero(struct wrapper_robot_t *wrapper, int i)
{
    int ret;
    struct motor_group_t *group = 0;

    group = wrap_get_motor_group(wrapper, i);

    p1010a_set_enable(group, false);
    p1010a_set_zero(group);
}

int wrap_drive_ota_upgrade(struct wrapper_robot_t *wrapper)
{
    struct motor_t * motor = 0;
    struct motor_group_t *group = 0;
    

#if 0
    // if (wrap_get_motor_model(wrapper) != MODEL_Z1_M1505) {
    //     return;
    // }

    group = wrap_get_motor_group(wrapper, RIGHT);

    z1_rs1_set_enable(group, false);

    motor = &group->motors[0];

    if (motor->ota_status != 0/* || motor->ota_recv == 0*/) {
        return motor->ota_status;
    }


    if (motor->ota_state == 0) {
        // z1_rs1_set_enable(group, false);
        z1_rs1_start_ota(group, 0);
        motor->ota_state = 11; return 0;

    } else if (motor->ota_state == 11) {
        LOG_INF("send ota start");

        z1_rs1_start_ota(group, 0);
        motor->ota_state = 12; return 0;
        
    } else if (motor->ota_state == 12) {
        LOG_INF("send firmware info");
    } else if (motor->ota_state == 13 || motor->ota_state == 14) {
        LOG_INF("send firmware data");
    }

#endif
    return 0;
}

void wrap_drvie_set_zero(struct wrapper_robot_t *wrapper, int i)
{
    struct motor_group_t *group = 0;

    // only support P1010
    if (wrap_get_motor_model(wrapper) == MODEL_P1010_M1505) {
        p1010a_motor_set_zero(wrapper, i);
    } else if (wrap_get_motor_model(wrapper) == MODEL_Z1_M1505) {
        z1_rs1_motor_set_zero(wrapper, i);
    }
}

int wrap_drvie_setup(struct wrapper_robot_t *wrapper)
{
    int ret = 0;
    struct motor_group_t *group = 0;

    m15_motor_config(wrapper);

    const struct can_filter any_filter = {
        .id = 0x000, .mask = 0x000, .flags = CAN_FILTER_DATA};

    // 0x050 ~ 0x05F
    const struct can_filter p1010a_filter = {
        .id = 0x050, .mask = CAN_STD_ID_MASK - 0x00F, .flags = CAN_FILTER_DATA};

    // 0x090 ~ 0x09F
    const struct can_filter m1505a_filter = {
        .id = 0x090, .mask = CAN_STD_ID_MASK - 0x00F, .flags = CAN_FILTER_DATA};

    // 0x140 ~ 0x143
    const struct can_filter lk_filter = {
        .id = 0x140, .mask = CAN_STD_ID_MASK - 0x003, .flags = CAN_FILTER_DATA};

     // CMD = 2 && CAN_ID = 0
    const struct can_filter rs1_filter = {
        .id = 0x02000000, .mask = 0x1F0000FF, .flags = CAN_FILTER_DATA | CAN_FILTER_IDE};


    group = wrap_get_motor_group(wrapper, LEFT);

    group->bus = can1;

    ret = can_start(group->bus);
    if (ret != 0) {
        return ret;
    }

    can_add_rx_filter(group->bus, lk_rx_callback, (void *)group, &lk_filter);

    can_add_rx_filter(group->bus, p1010a_rx_callback, (void *)group, &p1010a_filter);

    can_add_rx_filter(group->bus, m1505a_rx_callback, (void *)group, &m1505a_filter);

    // can_add_rx_filter(group->bus, z1_rs1_rx_callback, (void *)group, &rs1_filter);

    // DaMiao default ID is 0, shared filter with any_filter
    // can_add_rx_filter(group->bus, dm_rx_callback, (void *)group, &dm_filter);

    can_add_rx_filter(group->bus, any_rx_callback, (void *)group, &any_filter);


    group = wrap_get_motor_group(wrapper, RIGHT);

    group->bus = can0;

    ret = can_start(group->bus);
    if (ret != 0) {
        return ret;
    }

    can_add_rx_filter(group->bus, lk_rx_callback, (void *)group, &lk_filter);

    can_add_rx_filter(group->bus, p1010a_rx_callback, (void *)group, &p1010a_filter);

    can_add_rx_filter(group->bus, m1505a_rx_callback, (void *)group, &m1505a_filter);

    // can_add_rx_filter(group->bus, z1_rs1_rx_callback, (void *)group, &rs1_filter);

    // DaMiao default ID is 0, shared filter with any_filter
    // can_add_rx_filter(group->bus, dm_rx_callback, (void *)group, &dm_filter);

    can_add_rx_filter(group->bus, any_rx_callback, (void *)group, &any_filter);


    m15_motor_init(wrapper);

    // z1_rs1_motor_init(wrapper);

    // dm_motor_init(wrapper);

    p1010a_motor_init(wrapper);

    return ret;
}

int wrap_drive_reconfigure(struct wrapper_robot_t *wrapper, uint32_t frequency)
{
    int ret = 0;
    struct motor_group_t *group = 0;

    group = wrap_get_motor_group(wrapper, LEFT);

    m15_set_data_frequency(group, frequency);

    group = wrap_get_motor_group(wrapper, RIGHT);

    m15_set_data_frequency(group, frequency);

    return ret;
}

int wrap_drive_fresh(struct wrapper_robot_t *wrapper)
{
#ifdef CONFIG_APP_ABUP_FOTA	
    if(upgradeMotor_getState() != UPGRADE_STATE_INITIAL)
    {
        return 0;
    }
#endif
    int ret;
    struct can_frame frame;
    struct motor_group_t *group = 0;
    int motor_status = 0;
    int error_count = 0;

    for (int i = LEFT; i <= RIGHT; i++) {
        group = wrap_get_motor_group(wrapper, i);

        for (int j = M_ABAD; j <= M_ANKLE; j++) {
            error_count = group->motors[j].tx_count - group->motors[j].rx_count;
            if (error_count > 500) { // 1505 defualt ignore first 10 frames
                group->motor_status |= BIT(j * 8);
            }

            // Check Z1 status
            if (j != M_ANKLE && wrap_get_motor_model(wrapper) == MODEL_Z1_M1505) {
                if (group->motors[j].status != 0x80) {
                    group->motor_status |= BIT(j * 8);
                }
            }
            
            if (j == M_ANKLE && group->motors[j].type == MT_M1505) {
                if (group->motors[j].status != 0x00) {
                    group->motor_status |= (group->motors[j].status << 24) & 0xFF000000;
                }
            }
             
        }

        motor_status |=  group->motor_status;
    }

    if (motor_status != 0) {
        // force zero output
        wrap_motor_stub_zero(wrapper);
    }

    for (int i = LEFT; i <= RIGHT; i++) {
        group = wrap_get_motor_group(wrapper, i);


        if (wrap_get_motor_model(wrapper) == MODEL_MG10015_M1505) {
            lk_stuff_broadcast(&frame, group);

            // LOG_HEXDUMP_INF(frame.data, 8, "TX RAW");

            ret = can_send(group->bus, &frame, K_NO_WAIT, any_tx_callback, group);
            if (ret == 0) {
                group->motors[M_ABAD].tx_count += 1;
                group->motors[M_HIP].tx_count += 1;
                group->motors[M_KNEE].tx_count += 1;
            } else {
                group->bus_tx_error += 1;
            }

            m1505a_stuff_broadcast(&frame, group);

            ret = can_send(group->bus, &frame, K_NO_WAIT, any_tx_callback, group);
            if (ret == 0) {
                group->motors[M_ANKLE].tx_count += 1;
            } else {
                group->bus_tx_error += 1;
            }
        } else if (wrap_get_motor_model(wrapper) == MODEL_P1010_M1505) {
            
            p1010a_m1505a_stuff_broadcast(&frame, group);

            ret = can_send(group->bus, &frame, K_NO_WAIT, any_tx_callback, group);
            if (ret == 0) {
                group->motors[M_ABAD].tx_count += 1;
                group->motors[M_HIP].tx_count += 1;
                group->motors[M_KNEE].tx_count += 1;
                // for point foot
                // group->motors[M_ANKLE].tx_count += 1;
            } else {
                group->bus_tx_error += 1;
            }
        } else if (wrap_get_motor_model(wrapper) == MODEL_P1010_RS1) {
            
            p1010a_m1505a_stuff_broadcast(&frame, group);

            ret = can_send(group->bus, &frame, K_NO_WAIT, any_tx_callback, group);
            if (ret == 0) {
                group->motors[M_ABAD].tx_count += 1;
                group->motors[M_HIP].tx_count += 1;
                group->motors[M_KNEE].tx_count += 1;
            } else {
                group->bus_tx_error += 1;
            }

            z1_rs1_stuff_broadcast(&frame, group, M_ANKLE);

            ret = can_send(group->bus, &frame, K_NO_WAIT, any_tx_callback, group);
            if (ret == 0) {
                group->motors[M_ANKLE].tx_count += 1;
            } else {
                group->bus_tx_error += 1;
            }
        } else if (wrap_get_motor_model(wrapper) == MODEL_Z1_M1505) {
            for (int i = M_ABAD; i <= M_KNEE; i++) {

                z1_rs1_stuff_broadcast(&frame, group, i);

                ret = can_send(group->bus, &frame, K_NO_WAIT, any_tx_callback, group);
                if (ret == 0) {
                    group->motors[i].tx_count += 1;
                } else {
                    group->bus_tx_error += 1;
                }
            }

            m1505a_stuff_broadcast(&frame, group);

            ret = can_send(group->bus, &frame, K_NO_WAIT, any_tx_callback, group);
            if (ret == 0) {
                group->motors[M_ANKLE].tx_count += 1;
            } else {
                group->bus_tx_error += 1;
            }
        } else if (wrap_get_motor_model(wrapper) == MODEL_Z1_RS1) {
            for (int i = M_ABAD; i <= M_ANKLE; i++) {

                z1_rs1_stuff_broadcast(&frame, group, i);
                
                ret = can_send(group->bus, &frame, K_NO_WAIT, any_tx_callback, group);
                if (ret == 0) {
                    group->motors[i].tx_count += 1;
                } else {
                    group->bus_tx_error += 1;
                }
            }
        } else if (wrap_get_motor_model(wrapper) == MODEL_P1010_DM8006) {
            p1010a_m1505a_stuff_broadcast(&frame, group);

            ret = can_send(group->bus, &frame, K_NO_WAIT, any_tx_callback, group);
            if (ret == 0) {
                group->motors[M_ABAD].tx_count += 1;
                group->motors[M_HIP].tx_count += 1;
                group->motors[M_KNEE].tx_count += 1;
            } else {
                group->bus_tx_error += 1;
            }

            dm_stuff_broadcast(&frame, group, M_ANKLE);

            ret = can_send(group->bus, &frame, K_NO_WAIT, any_tx_callback, group);
            if (ret == 0) {
                group->motors[M_ANKLE].tx_count += 1;
            } else {
                group->bus_tx_error += 1;
            }
        } else if (wrap_get_motor_model(wrapper) == MODEL_P1010_P1010) {
            p1010a_stuff_broadcast(&frame, group);

            ret = can_send(group->bus, &frame, K_NO_WAIT, any_tx_callback, group);
            if (ret == 0) {
                group->motors[M_ABAD].tx_count += 1;
                group->motors[M_HIP].tx_count += 1;
                group->motors[M_KNEE].tx_count += 1;
                group->motors[M_ANKLE].tx_count += 1;
            } else {
                group->bus_tx_error += 1;
            }
        }
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

int wrap_drive_reset_count(struct wrapper_robot_t *wrapper)
{
    struct motor_group_t *group = 0;

    for (int i = LEFT; i <= RIGHT; i++) {
        group = wrap_get_motor_group(wrapper, i);

        for (int j = M_ABAD; j <= M_ANKLE; j++) {
            group->motors[j].tx_count = 0;
            group->motors[j].rx_count = 0;
        }
    }

    return 0;
}

int wrap_drive_reset_status(struct wrapper_robot_t *wrapper)
{
    struct motor_group_t *group = 0;

    for (int i = LEFT; i <= RIGHT; i++) {
        group = wrap_get_motor_group(wrapper, i);
        group->motor_status = 0;
    }

    return 0;
}

int wrap_drive_check(const struct wrapper_robot_t *wrapper, int status[2])
{
    struct motor_group_t *group = 0;
    int motor_status = 0;

    for (int i = LEFT; i <= RIGHT; i++) {
        group = wrap_get_motor_group(wrapper, i);

        if (status != NULL)
            status[i] = group->motor_status;

        motor_status |= group->motor_status;
        // todo: more details
        motor_status |= (group->bus_rx_error > 100 || group->bus_tx_error > 100);
    }

    return motor_status;
}

