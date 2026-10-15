

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include "robot.h"
#include "wrapper.h"
#include "api_canfd.h"
#ifdef CONFIG_APP_ABUP_FOTA	
#include "abup_fota.h"
#endif

LOG_MODULE_REGISTER(api_canfd, LOG_LEVEL_INF);

static const struct device *const canfd = DEVICE_DT_GET(DT_CHOSEN(zephyr_api_canfd));

static struct api_ops_t api_canfd_ops;

static uint8_t calculate_optimal_q(float min, float max, uint8_t max_q)
{
    if (max_q > 15)
        max_q = 15;

    float range = fmaxf(fabsf(min), fabsf(max));

    for (int8_t q = max_q; q >= 0; q--)
    {
        float scale = (float)(1 << q);
        float scaled_min = min * scale;
        float scaled_max = max * scale;

        if (scaled_min >= -32768.0f && scaled_max <= 32767.0f)
        {
            return q;
        }
    }

    return 0;
}

int16_t float_to_int16(float in, float min, float max)
{
    uint8_t q = calculate_optimal_q(min, max, 15);

    float scale = (float)(1 << q);
    float scaled = in * scale;

    if (scaled > 32767.0f)
    {
        return 32767;
    }
    else if (scaled < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

float int16_to_float(int16_t in, float min, float max)
{
    uint8_t q = calculate_optimal_q(min, max, 15);

    float scale = (float)(1 << q);

    return (float)in / scale;
}

static void motor_out_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct robot_t *robot = (struct api_state_t *)user_data;
    struct api_state_t *state = &robot->api_state;

    uint8_t data_len = can_dlc_to_bytes(frame->dlc);

    #ifdef CONFIG_DDT_CANFD_USE_FIXED_POINT
    int base_idx;
    int max_motors_per_frame;

    #define FIXED_MOTOR_ID_BASE_FRONT  0x128
    #define FIXED_MOTOR_ID_BASE_REAR   0x12A

    if (robot->motor_index_offset > 0) {
        // 后机: 0x12A-0x12B
        if (frame->id < FIXED_MOTOR_ID_BASE_REAR || frame->id > FIXED_MOTOR_ID_BASE_REAR + 1) {
            return;
        }
        base_idx = (frame->id - FIXED_MOTOR_ID_BASE_REAR) * 4;
    } else {
        // 前机: 0x128-0x129
        if (frame->id < FIXED_MOTOR_ID_BASE_FRONT || frame->id > FIXED_MOTOR_ID_BASE_FRONT + 1) {
            return;
        }
        base_idx = (frame->id - FIXED_MOTOR_ID_BASE_FRONT) * 4;
    }

    max_motors_per_frame = data_len / sizeof(struct api_motor_out_16_t);
    if (max_motors_per_frame > 4) {
        max_motors_per_frame = 4;
    }

    for (int i = 0; i < max_motors_per_frame; i++)
    {
        struct api_motor_out_16_t *out_16 = &state->out_16[base_idx + i];
        memcpy((void *)out_16, frame->data + i * sizeof(struct api_motor_out_16_t), sizeof(struct api_motor_out_16_t));
        state->out[base_idx + i].timestamp = state->out_16[base_idx + i].timestamp;
        state->out[base_idx + i].position = int16_to_float(out_16->position, -25, 25);
        state->out[base_idx + i].kp = int16_to_float(out_16->kp, -32768, 32768);
        state->out[base_idx + i].velocity = int16_to_float(out_16->velocity, -100, 100);
        state->out[base_idx + i].kd = int16_to_float(out_16->kd, -32768, 32768);
        state->out[base_idx + i].torque = int16_to_float(out_16->torque, -100, 100);
    }
    #else
    int base_idx = (frame->id - CAN_ID_MOTOR_SWAP) * 2;

    if (robot->motor_index_offset > 0 && base_idx > 7) {
        // rear
        base_idx -= 8;
    } else if (robot->motor_index_offset == 0 && base_idx < 8) {
        // front
    } else {
        // other
        return;
    }

    // maybe only one motor data
    for (int i = 0; i < data_len / sizeof(struct api_motor_out_t); i++) {
        struct api_motor_out_t *out = &state->out[base_idx + i];

        memcpy((void *)out, frame->data + i * sizeof(struct api_motor_out_t), sizeof(struct api_motor_out_t));
    }

    // LOG_INF("MotorOut 0x%03X LEN %d", frame->id, data_len);
    // LOG_HEXDUMP_INF(frame->data, data_len, "RX RAW");
    #endif
}

#if 0
static void sensor_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct robot_t *robot = (struct api_state_t *)user_data;
    struct api_state_t *state = &robot->api_state;

    uint8_t data_len = can_dlc_to_bytes(frame->dlc);

    if (frame->id == 0x12D) {
        struct api_channel_input_t *input = (struct api_channel_input_t *)frame->data;
        memcpy((void *)(&state->channel_input), (void *)input, sizeof(struct api_channel_input_t));
    } else if (frame->id == 0x12E) {
        struct api_twist_t *twist = (struct api_twist_t *)frame->data;
        memset((void *)(&state->channel_input), 0, sizeof(struct api_channel_input_t));
    } else {
        LOG_ERR("Not Support device type: %d", frame->id);
    }
}

static void any_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct robot_t *robot = (struct api_state_t *)user_data;
    struct api_state_t *state = &robot->api_state;

    uint8_t len = can_dlc_to_bytes(frame->dlc);

    if (frame->id != 0x170) {
        LOG_ERR("Not Support request: %d", frame->id);
        return;
    }

    struct api_rpc_request_t *request = (struct api_rpc_request_t *)frame->data;

    switch(request->key) {
        case SET_STAND_MODE:
            robot->input.stand_mode = request->value.u8;
            break;
        case SET_READY_NEXT:
            robot->ready_next = request->value.u8;
            break;
        case SET_JUMP:
            robot->input.jump_state = request->value.u8;
            break;
        default:
        /* nothing */
    }
}

#else

static inline void rpc_rx_callback(struct can_frame *frame, struct robot_t *robot)
{
    struct api_rpc_request_t *request = (struct api_rpc_request_t *)frame->data;

    switch(request->key) {
        case SET_STAND_MODE:
            robot->input.stand_mode = request->value.u8;
            break;
        case SET_READY_NEXT:
            LOG_DBG("ready_next: %d", robot->ready_next);
            robot->ready_next = request->value.u8;
            break;
        case SET_JUMP:
            // TODO: more safe
            robot->input.jump_state = request->value.u8;
            break;
        case SET_MOTOR_ZERO:
            robot->motor_zero = request->value.u8;
            break;
        default:
        /* nothing */
    }
}

static inline void sensor_rx_callback(struct can_frame *frame, struct robot_t *robot)
{
    static uint32_t pseudo_ts = 0;
    struct api_state_t *state = &robot->api_state;

    struct api_channel_input_t *input = (struct api_channel_input_t *)frame->data;

    size_t size = can_dlc_to_bytes(frame->dlc);
    size = size < sizeof(struct api_channel_input_t) ? size : sizeof(struct api_channel_input_t);
    memset((void *)(&state->channel_input), 0, sizeof(struct api_channel_input_t));
    memcpy((void *)(&state->channel_input), (void *)input, size);

    // pseudo timestamp for api timeout check
    if (input->timestamp == 0) {
        state->channel_input.timestamp = pseudo_ts++;
    }
}

static inline void canfd_reconfigure_rate(struct can_frame *frame, struct robot_t *robot)
{
    robot->canfd_8mbps_mode = 1;
}

static inline void canfd_reconfigure_motor_offset(struct can_frame *frame, struct robot_t *robot)
{
    if (frame->dlc != 8)
        return;

    if (frame->data[0] == 0xFF && frame->data[1] == 0xFF)
        robot->motor_index_offset = 8;
    else if (frame->data[0] == 0x00 && frame->data[1] == 0x00)
        robot->motor_index_offset = 0;
}

/** do not change bitrate in callback */
void canfd_reconfigure_rate_real(void)
{
    int ret;
    struct can_timing timing, timing_data;

    ret = can_stop(canfd);
    if (ret != 0) {
        LOG_ERR("can_stop failed, retval = %d", ret);
    }

    timing.sjw = CAN_SJW_NO_CHANGE;
    timing_data.sjw = CAN_SJW_NO_CHANGE;

    ret = can_calc_timing(canfd, &timing, 1000000, 800);
    if (ret != 0) {
        LOG_ERR("can_calc_timing failed, retval = %d", ret);
    }
    ret = can_calc_timing_data(canfd, &timing_data, 8000000, 800);
    if (ret != 0) {
        LOG_ERR("can_calc_timing_data failed, retval = %d", ret);
    }

    ret = can_set_timing(canfd, &timing);
    if (ret != 0) {
        LOG_ERR("can_set_timing failed, retval = %d", ret);
    }
    ret = can_set_timing_data(canfd, &timing_data);
    if (ret != 0) {
        LOG_ERR("can_set_timing_data failed, retval = %d", ret);
    }

    ret = can_start(canfd);
    if (ret != 0) {
        LOG_ERR("can_start failed, retval = %d", ret);
    }
}

static inline void pms_rx_callback(struct can_frame *frame, struct robot_t *robot)
{
    int i = 0;

    switch (frame->id)
    {
    case 0x81: // left battery
    case 0x82: // right battery
    case 0x83: // supercap battery
        uint32_t val = *((uint32_t *)(frame->data + 4));
        uint32_t online = val & 0x80000000 ? 1 : 0;
        uint32_t voltage = val & 0x7FFFFFFF;
        i = frame->id - 0x81;
        robot->batt_info[i].voltage = online ? voltage : 0;
        break;
    case 0x9F:
        canfd_reconfigure_rate(frame, robot);
        break;
    default:
        // LOG_INF("%03x", frame->id);
        // LOG_HEXDUMP_INF(frame->data, can_dlc_to_bytes(frame->dlc), "pms");
        break;
    }
}

static void any_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
    struct robot_t *robot = (struct api_state_t *)user_data;
    struct api_state_t *state = &robot->api_state;

    if ((frame->id & 0x780) == 0x080) {
        pms_rx_callback(frame, robot);
        return;
    }

    switch (frame->id) {
        case 0x170:
            rpc_rx_callback(frame, robot);
            break;
        case 0x12D:
            sensor_rx_callback(frame, robot);
            break;
        case 0x381:
            canfd_reconfigure_rate(frame, robot);
            break;
        case 0x385:
            canfd_reconfigure_motor_offset(frame, robot);
            break;
        default:
            break;
    }
}

#endif

static void any_tx_callback(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(error);
    ARG_UNUSED(user_data);
}

static void state_change_callback(const struct device *dev, enum can_state state, struct can_bus_err_cnt err_cnt, void *user_data)
{
#if 0 // TODO: slow reboot canfd 
    LOG_WRN("CANFD STATE: %d", state);
#endif
}

int api_canfd_init(struct robot_t *robot)
{
    int ret;
    uint32_t rate;
    struct can_timing timing_data;

#if 0
    const struct can_filter sensor_filter = {
        .id = 0x128, .mask = 0x7F8, .flags = CAN_FILTER_DATA | CAN_FILTER_FDF};
#endif

    const struct can_filter motor_out_filter = {
        .id = 0x120, .mask = 0x7F8, .flags = CAN_FILTER_DATA | CAN_FILTER_FDF};

        // 定点版电机指令过滤器: 0x128-0x12B
    const struct can_filter motor_out_fixed_filter = {
        .id = 0x128, .mask = 0x7FC, .flags = CAN_FILTER_DATA | CAN_FILTER_FDF};

    // filter 0b0001/0b0010/0b0011
    const struct can_filter any_filter = {
        .id = 0x000, .mask = 0x000, .flags = CAN_FILTER_DATA | CAN_FILTER_FDF};

    if (!device_is_ready(canfd)) {
        return -ENOTSUP;
    }

#if 0 // for debug
    can_get_core_clock(canfd, &rate);
    LOG_DBG("canfd core clock: %lu", rate);
#endif

    robot->api_ops = &api_canfd_ops;

    can_set_mode(canfd, CAN_MODE_FD);

#if 0
    can_add_rx_filter(canfd, sensor_rx_callback, (void *)robot, &sensor_filter);
#endif

    can_add_rx_filter(canfd, motor_out_rx_callback, (void *)robot, &motor_out_filter);
    can_add_rx_filter(canfd, motor_out_rx_callback, (void *)robot, &motor_out_fixed_filter);

    can_add_rx_filter(canfd, any_rx_callback, (void *)robot, &any_filter);

    can_set_state_change_callback(canfd, state_change_callback, 0);

    /** dts is set to a maximum of 8 Mbps
     *  and for compatibility is set to 5 Mbps */
    timing_data.sjw = CAN_SJW_NO_CHANGE;

    ret = can_calc_timing_data(canfd, &timing_data, 5000000, 750);
    if (ret != 0) {
        LOG_ERR("can_calc_timing_data failed, retval = %d", ret);
    }

    ret = can_set_timing_data(canfd, &timing_data);
    if (ret != 0) {
        LOG_ERR("can_set_timing_data failed, retval = %d", ret);
    }

    ret = can_start(canfd);
    if (ret != 0) {
        LOG_ERR("can_start failed, retval = %d", ret);
    }

    return 0;
}

void api_canfd_upload_status_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
    int ret = 0;
    struct api_locomotion_status_t *api_locomotion_status;
    struct can_frame frame;

    memset(frame.data, 0, sizeof(frame.data));
    api_locomotion_status = (struct api_locomotion_status_t *)(&frame.data);

    api_locomotion_status->timestamp = robot->timestamp;
    api_locomotion_status->key = api_state->locomotion_status[api_state->status_offset].key;
    api_locomotion_status->value = api_state->locomotion_status[api_state->status_offset].value;

    frame.id = CAN_ID_LOCOMOTION_STATE;
    frame.dlc = can_bytes_to_dlc(sizeof(struct api_locomotion_status_t));
    frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

    ret = can_send(canfd, &frame, K_NO_WAIT, any_tx_callback, 0);
    if (ret != 0) {
        if (ret == -EAGAIN || ret == -ENETUNREACH) {
            // time out, not free mailbox, drop frame
            // ignore error message 
        } else {
            LOG_ERR("api canfd upload imu date error: %d", ret);
        }
    }

    while (api_state->status_offset++) {
        if (api_state->status_offset > 7)
            api_state->status_offset = 0;

        if (api_state->locomotion_status[api_state->status_offset].key != 0)
            break;
    }
}

void api_canfd_lock_connector(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
    int ret = 0;
    struct can_frame frame;

    frame.id = 0x89;
    frame.dlc = 8;
    frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

    memset(frame.data, 0, sizeof(frame.data));

    // // Lock 1000mA
    // frame.data[0] = 0xFC;
    // frame.data[1] = 0x17;

    // Lock 2000mA
    frame.data[0] = 0xF8;
    frame.data[1] = 0x2F;

    ret = can_send(canfd, &frame, K_MSEC(2), any_tx_callback, 0);
    if (ret != 0) {
        LOG_ERR("api canfd lock connector error: %d", ret);
    }
}

void api_canfd_stop_connector(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
    int ret = 0;
    struct can_frame frame;

    frame.id = 0x89;
    frame.dlc = 8;
    frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

    memset(frame.data, 0, sizeof(frame.data));

    ret = can_send(canfd, &frame, K_MSEC(2), any_tx_callback, 0);
    if (ret != 0) {
        LOG_ERR("api canfd stop connector error: %d", ret);
    }
}

void api_canfd_upload_imu_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
#ifdef CONFIG_APP_ABUP_FOTA	
    if(abup_get_fota_state() != ABUP_FOTA_STATE_NO_UPGRADE)
    {
        return;
    } 
#endif

    // if (robot->motor_index_offset > 0)
    //     return; // ignore imu2

    int ret = 0;
    struct api_imu_data_t *api_imu_data;
    struct can_frame frame;

    api_imu_data = (struct api_imu_data_t *)(&frame.data);

    api_imu_data->timestamp = robot->timestamp;
    api_imu_data->accel[0] = robot->accel[0];
    api_imu_data->accel[1] = robot->accel[1];
    api_imu_data->accel[2] = robot->accel[2];
    api_imu_data->gyro[0] = robot->gyro[0];
    api_imu_data->gyro[1] = robot->gyro[1];
    api_imu_data->gyro[2] = robot->gyro[2];
    api_imu_data->quaternion[0] = robot->quaternion[0];
    api_imu_data->quaternion[1] = robot->quaternion[1];
    api_imu_data->quaternion[2] = robot->quaternion[2];
    api_imu_data->quaternion[3] = robot->quaternion[3];
    api_imu_data->temperature = robot->temperature;

    frame.id = robot->motor_index_offset > 0 ? CAN_ID_IMU2 : CAN_ID_IMU1;
    frame.dlc = can_bytes_to_dlc(sizeof(struct api_imu_data_t));
    frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

    ret = can_send(canfd, &frame, K_NO_WAIT, any_tx_callback, 0);
    if (ret != 0) {
        if (ret == -EAGAIN || ret == -ENETUNREACH) {
            // time out, not free mailbox, drop frame
            // ignore error message 
        } else {
            LOG_ERR("api canfd upload imu date error: %d", ret);
        }
    }

    return 0;
}

void api_canfd_upload_motor_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
#ifdef CONFIG_APP_ABUP_FOTA	
    if(abup_get_fota_state() != ABUP_FOTA_STATE_NO_UPGRADE)
    {
        return;
    } 
#endif

    // copy wrapper data to api_state
    struct motor_group_t *group = 0;

    for (int i = LEFT; i <= RIGHT; i++) {
        int idx = i * 4;
        struct motor_t *motor = 0;
        struct api_motor_in_t *api_in = 0;
        struct api_motor_out_t *api_out = 0;

        const struct wrapper_robot_t *wrapper = robot->priv;

        group = wrap_get_motor_group(wrapper, i);

        for (int i = M_ABAD; i <= M_ANKLE; i++) {
            motor = &group->motors[i];
            api_in = &api_state->in[idx + i];
            api_out = &api_state->out[idx + i];

            api_in->timestamp = motor->in.timestamp;
            api_in->position = motor->in.position;
            api_in->velocity = motor->in.velocity;
            api_in->torque = motor->in.torque;

            // TODO: more fine timestamp
            api_in->timestamp = robot->timestamp;

            api_out->timestamp = motor->out.timestamp;
            api_out->position = motor->out.position;
            api_out->kp = motor->out.kp;
            api_out->velocity = motor->out.velocity;
            api_out->kd = motor->out.kd;
            api_out->torque = motor->out.torque;
        }
    }

    // send api_state to bus
    int ret = 0;
    size_t dsize = 0;
    struct can_frame frame;
    
    dsize = sizeof(struct api_motor_in_t) * 4;

    for (int i = 0; i < API_MAX_MOTORS / 4; i++) {
        frame.id = CAN_ID_MOTOR_IN + i;
        frame.dlc = can_bytes_to_dlc(dsize);
        frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

        memcpy(frame.data, api_state->in + i * 4, dsize);

        ret = can_send(canfd, &frame, K_NO_WAIT, any_tx_callback, 0);
        if (ret != 0) {
            if (ret == -EAGAIN || ret == -ENETUNREACH) {
              // time out, not free mailbox, drop frame
              // ignore error message
              api_state->send_drop_count += 1;
            } else {
              LOG_ERR("api canfd upload motor date error: %d", ret);
              api_state->send_error_count += 1;
            }
        }
    }

    dsize = sizeof(struct api_motor_out_t) * 2;

    for (int i = 0; i < API_MAX_MOTORS / 2; i++) {
        frame.id = CAN_ID_MOTOR_OUT + i;
        frame.dlc = can_bytes_to_dlc(dsize);
        frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

        memcpy(frame.data, api_state->out + i * 2, dsize);

        ret = can_send(canfd, &frame, K_NO_WAIT, any_tx_callback, 0);
        if (ret != 0) {
            if (ret == -EAGAIN || ret == -ENETUNREACH) {
              // time out, not free mailbox, drop frame
              // ignore error message
              api_state->send_drop_count += 1;
            } else {
              LOG_ERR("api canfd upload motor date error: %d", ret);
              api_state->send_error_count += 1;
            }
        } 
    }
}

#ifdef CONFIG_DDT_CANFD_USE_FIXED_POINT
void api_canfd_swap_motor_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
    #ifdef CONFIG_APP_ABUP_FOTA
    if (abup_get_fota_state() != ABUP_FOTA_STATE_NO_UPGRADE)
    {
        return;
    }
    #endif
    struct motor_group_t *group = 0;

    for (int i = LEFT; i <= RIGHT; i++)
    {
        int idx = i * 4;
        struct motor_t *motor = 0;
        struct api_motor_out_t *api_out = 0;
        struct api_motor_in_16_t *api_in_16 = 0;

        const struct wrapper_robot_t *wrapper = robot->priv;

        group = wrap_get_motor_group(wrapper, i);

        for (int i = M_ABAD; i <= M_ANKLE; i++)
        {
            motor = &group->motors[i];
            api_out = &api_state->out[idx + i];

            api_in_16 = &api_state->in_16[idx + i];

            api_in_16->timestamp = motor->in.timestamp;
            api_in_16->position = float_to_int16(motor->in.position, -25, 25);
            api_in_16->velocity = float_to_int16(motor->in.velocity, -100, 100);
            api_in_16->torque = float_to_int16(motor->in.torque, -100, 100);

            api_in_16->timestamp = robot->timestamp;

            motor->out.timestamp = api_out->timestamp;
            motor->out.position = api_out->position;
            motor->out.kp = api_out->kp;
            motor->out.velocity = api_out->velocity;
            motor->out.kd = api_out->kd;
            motor->out.torque = api_out->torque;
        }
    }

    int ret = 0;
    size_t dsize = 0;
    struct can_frame frame;
    dsize = sizeof(struct api_motor_in_16_t) * 4;

    for (int i = 0; i < API_MAX_MOTORS / 4; i++)
    {
        frame.id = CAN_ID_MOTOR_FIXED_IN + i;
        frame.dlc = can_bytes_to_dlc(dsize);
        frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

        if (robot->motor_index_offset > 0)
        {
            frame.id += robot->motor_index_offset / 4;
        }

        memcpy(frame.data, api_state->in_16 + i * 4, dsize);

        ret = can_send(canfd, &frame, K_NO_WAIT, any_tx_callback, 0);
        if (ret != 0)
        {
            if (ret == -EAGAIN || ret == -ENETUNREACH)
            {
                // time out, not free mailbox, drop frame
                // ignore error message
            }
            else
            {
                LOG_ERR("api canfd upload motor date error: %d", ret);
            }
        }
    }

    api_state->api_motor_out_count += 1;
}
#else
void api_canfd_swap_motor_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
#ifdef CONFIG_APP_ABUP_FOTA	
    if(abup_get_fota_state() != ABUP_FOTA_STATE_NO_UPGRADE)
    {
        return;
    }
#endif 
    struct motor_group_t *group = 0;

    for (int i = LEFT; i <= RIGHT; i++) {
        int idx = i * 4;
        struct motor_t *motor = 0;
        struct api_motor_in_t *api_in = 0;
        struct api_motor_out_t *api_out = 0;

        const struct wrapper_robot_t *wrapper = robot->priv;

        group = wrap_get_motor_group(wrapper, i);

        for (int i = M_ABAD; i <= M_ANKLE; i++) {
            motor = &group->motors[i];
            api_in = &api_state->in[idx + i];
            api_out = &api_state->out[idx + i];

            api_in->timestamp = motor->in.timestamp;
            api_in->position = motor->in.position;
            api_in->velocity = motor->in.velocity;
            api_in->torque = motor->in.torque;

            // TODO: more fine timestamp
            api_in->timestamp = robot->timestamp;

            motor->out.timestamp = api_out->timestamp;
            motor->out.position = api_out->position;
            motor->out.kp = api_out->kp;
            motor->out.velocity = api_out->velocity;
            motor->out.kd = api_out->kd;
            motor->out.torque = api_out->torque;
        }
    }

    int ret = 0;
    struct can_frame frame;
    size_t dsize = sizeof(struct api_motor_in_t) * 4;

    for (int i = 0; i < API_MAX_MOTORS / 4; i++) {
        frame.id = CAN_ID_MOTOR_IN + i;
        frame.dlc = can_bytes_to_dlc(dsize);
        frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

        if (robot->motor_index_offset > 0) {
            frame.id += robot->motor_index_offset / 4;
        }

        memcpy(frame.data, api_state->in + i * 4, dsize);

        ret = can_send(canfd, &frame, K_NO_WAIT, any_tx_callback, 0);
        if (ret != 0) {
            if (ret == -EAGAIN || ret == -ENETUNREACH) {
              // time out, not free mailbox, drop frame
              // ignore error message
            } else {
              LOG_ERR("api canfd upload motor date error: %d", ret);
            }
        }
    }

    api_state->api_motor_out_count += 1;
}
#endif

/** report motor state */
void api_canfd_report_motor_status(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
    int ret = 0;
    struct can_frame frame;

    frame.id = CAN_ID_STATE;
    frame.dlc = can_bytes_to_dlc(sizeof(struct api_motor_status_t));
    frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

    struct api_motor_status_t *motor_status = (struct api_motor_status_t *)frame.data;
    memset((void *)motor_status, 0, sizeof(struct api_motor_status_t));

    motor_status->timestamp = 0;
    motor_status->key = MOTOR_STATE;

    int status[2];
    wrap_drive_check(robot->priv, status);

    motor_status->value[0] = (status[0] >> 0) & 0xFF;
    motor_status->value[1] = (status[0] >> 8) & 0xFF;
    motor_status->value[2] = (status[0] >> 16) & 0xFF;
    motor_status->value[3] = (status[0] >> 24) & 0xFF;

    motor_status->value[4] = (status[1] >> 0) & 0xFF;
    motor_status->value[5] = (status[1] >> 8) & 0xFF;
    motor_status->value[6] = (status[1] >> 16) & 0xFF;
    motor_status->value[7] = (status[1] >> 24) & 0xFF;    

    ret = can_send(canfd, &frame, K_NO_WAIT, any_tx_callback, 0);
    if (ret != 0) {
        if (ret == -EAGAIN) {
          // time out, not free mailbox, drop frame
          // ignore error message
        } else {
          LOG_ERR("api canfd report motor status error: %d", ret);
        }
    }
}

static struct api_ops_t api_canfd_ops = {
    .upload_status_data = api_canfd_upload_status_data,
    .upload_imu_data = api_canfd_upload_imu_data,
    .upload_motor_data = api_canfd_upload_motor_data,
    .swap_motor_data = api_canfd_swap_motor_data,
    .report_motor_status = api_canfd_report_motor_status,
};
