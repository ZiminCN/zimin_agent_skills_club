
#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>

#include "led_gpio.h"
#include "gpio_ctrl.h"
#include "api_acm.h"
#include "api_canfd.h"
#include "rc_sbus.h"
#include "sensors.h"
#include "wrapper.h"
#include "robot.h"
#include "ahrs.h"

LOG_MODULE_REGISTER(robot, LOG_LEVEL_INF);

#define DEFAULT_RATE 10
#define DIRECT_RATE 400
#define LOCOMOTION_RATE 500
#define SELFTEST_RATE 100
#define READY_RATE 10
#define FAULT_RATE 5
#define MOTOR_CAL_RATE 1

/**
 * Locomotion State Machine
*/

static void robot_selftest_state_entry(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;

    LOG_DBG("ts: %08u %s", robot->timestamp, __func__);

    robot_set_frequency(robot, SELFTEST_RATE);
}

static void robot_selftest_state_run(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;
    struct wrapper_robot_t *wrapper = robot->priv;

    LOG_DBG("ts: %08u %s", robot->timestamp, __func__);

    if (robot->cycle_count > SELFTEST_RATE * 10) {
        LOG_ERR("Self-Test Failed");
        robot_set_state(robot, ROBOT_FAULT_STATE);
        return;
    }

    // todo check power
    
    if (wrap_get_motor_model(wrapper) == MODEL_UNKNOWN) {
        return;
    }

    // wait 1 second, check motor online
    if (robot->cycle_count > SELFTEST_RATE * 1 && wrap_drive_check(wrapper, NULL) == 0) {
#if 0
        // check imu temp
        if (robot->temp > 00.0 && robot->temp < 60.0) {
            // check imu pose, about < 5 dagree
            if (robot->accel[0] >= 0.0 && robot->accel[0] < 0.1 
                    && robot->accel[1] >= 0.0 && robot->accel[1] < 0.1
                    && robot->accel[2] > 0.90 && robot->accel[2] <= 1.00) {
                // check imu pose again, quaternion w
                if (robot->quaternion[3] > 0.99 && robot->quaternion[3] < 1.01) {
                    LOG_INF("Self-Test Pass");
                    robot_set_state(robot, ROBOT_READY_STATE);
                    return;
                }
            }
        }
#else
        LOG_INF("Self-Test Pass");
        robot_set_state(robot, ROBOT_READY_STATE);
        return;
#endif
    }

    // send zero torque
    wrap_motor_stub_zero(wrapper);

    wrap_drive_fresh(wrapper);

    return;
}

static void robot_selftest_state_exit(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;
}

static void robot_ready_state_entry(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;

    LOG_DBG("ts: %08u %s", robot->timestamp, __func__);
    LOG_INF("Robot is Ready");

    robot_set_frequency(robot, READY_RATE);

    if (robot->canfd_8mbps_mode) {
        LOG_INF("Reconfigure CAN-FD bitrate");
        canfd_reconfigure_rate_real();
    }
}

static void robot_ready_state_run(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;

    int cycle_per_second = 100;
    int ready_next_state = 0;

#if 0
    if (robot->cycle_count > 10) {
        robot_set_state(robot, ROBOT_LOCOMOTION_STATE);
    }
#endif

    if (robot->rc_connected || robot->ready_next == FORCE_LOCOMOTION ||
        robot->ready_next == AUTO_LOCOMOTION) {
        robot_set_state(robot, ROBOT_LOCOMOTION_STATE);
    } else if (robot->ready_next == FORCE_DIRECT) {
        robot_set_state(robot, ROBOT_DIRECT_STATE);
    } else if (robot->ready_next == MOTOR_ZERO_CAL) {
        robot_set_state(robot, ROBOT_MOTOR_CALIBRATION_STATE);
    } else if (robot->ready_next == BOOT_RECOVERY_MODE) {
        robot_set_state(robot, ROBOT_BOOT_RECOVERY_MODE_STATE);
    } else {
        // notify waiting state update
    }
    
    if (ROBOT_MODEL_REAR == gpio_ctrl_update()) {
        robot->motor_index_offset = 8;
    } else {
        robot->motor_index_offset = 0;
    }

    // read setting, rc_auto_robot, wait_protocol, auto_robot, auto_direct
    // if sbus online, default entry robot
    // else auto loco
    // else auto direct
    // wait protocol mode

    // configure cycle timer
}

static void robot_locomotion_state_entry(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;
    struct wrapper_robot_t *wrapper = robot->priv;

    LOG_DBG("ts: %08u %s", robot->timestamp, __func__);
    LOG_INF("Entry Locomotion State");

    // get locomotion period
    led_gpio_set_indicator(LED_LOCOMOTION);

    uint locomotion_rate = LOCOMOTION_RATE;

    if (locomotion_rate > 500 &&
            (wrap_get_motor_model(wrapper) == MODEL_Z1_M1505 ||
             wrap_get_motor_model(wrapper) == MODEL_Z1_RS1)) {
        // CAN2.0 bandwidth overload
        locomotion_rate = 500;
    }

    robot_set_frequency(robot, locomotion_rate);

    wrap_ctrl_new(wrapper, locomotion_rate);
}

static void robot_locomotion_state_run(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;
    struct wrapper_robot_t *wrapper = robot->priv;

    float dt = robot->dt;
    uint32_t timestamp = robot->timestamp;

    /** check motor status */
    if (wrap_drive_check(wrapper, NULL) != 0) {
        robot_set_state(robot, ROBOT_FAULT_STATE);
    }

    /** check input, timeout force stand down */
    static uint32_t last_input_ts = 0;
    static uint32_t input_ts_count = 0;

    if (last_input_ts != robot->input.timestamp) {
        // check mono increment
        last_input_ts = robot->input.timestamp;
        input_ts_count = 0;
    } else {
        input_ts_count += 1;
    }
    if (input_ts_count > robot->frequency * 1) {
        robot->input.forward = 0;
        robot->input.yaw = 0;
        robot->input.pitch = 0;
        robot->input.roll = 0;
        robot->input.stand_mode = 1;
    }
    
    /** update status */
    wrap_update_status(wrapper, robot->api_state.locomotion_status, dt);

    // robot connector unstable
    // api_upload_imu_data(robot, &robot->api_state, robot->dt);

    if (robot->cycle_count % (robot->frequency / 100) == 0) {
        api_upload_status_data(robot, &robot->api_state, robot->dt);
    }

    /** update battery info */
    wrap_update_battery_info(wrapper, robot->batt_info, 3, dt);

    /** update input */
    wrap_update_input(wrapper, &robot->input, dt);

    /** update control loop */
    wrap_update_imu_data(wrapper, robot->accel, robot->gyro, robot->quaternion, dt);

    wrap_update_motor_stub(wrapper);

    wrap_ctrl_loop(wrapper, dt);

    wrap_motor_stub_fresh(wrapper);

    wrap_drive_fresh(wrapper);

    // robot connector unstable
    // api_upload_motor_data(robot, &robot->api_state, robot->dt);
}

static void robot_locomotion_state_exit(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;
    struct wrapper_robot_t *wrapper = robot->priv;

    wrap_motor_stub_zero(wrapper);

    wrap_drive_fresh(wrapper);
}

static void robot_direct_state_entry(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;

    LOG_DBG("ts: %08u %s", robot->timestamp, __func__);
    LOG_INF("Entry Direct State");

    // get locomotion period
    led_gpio_set_indicator(LED_LOCOMOTION);

    robot_set_frequency(robot, DIRECT_RATE);
}

static void robot_direct_state_run(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;
    struct wrapper_robot_t *wrapper = robot->priv;

    float dt = robot->dt;
    uint32_t timestamp= robot->timestamp;

    /** check motor status */
    if (wrap_drive_check(wrapper, NULL) != 0) {
        robot_set_state(robot, ROBOT_FAULT_STATE);
    }

    api_swap_motor_data(robot, &robot->api_state, robot->dt);

    wrap_drive_fresh(wrapper);

    api_upload_imu_data(robot, &robot->api_state, robot->dt);
}

static void robot_direct_state_exit(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;
    struct wrapper_robot_t *wrapper = robot->priv;
    
    wrap_motor_stub_zero(wrapper);

    wrap_drive_fresh(wrapper);
}

static void robot_fault_state_entry(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;

    LOG_DBG("ts: %08u %s", robot->timestamp, __func__);
    LOG_INF("Entry Robot Fault State");

    led_gpio_set_indicator(LED_FAULT);

    robot_set_frequency(robot, FAULT_RATE);
}

static void robot_fault_state_run(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;
    struct wrapper_robot_t *wrapper = robot->priv;

    wrap_motor_stub_zero(wrapper);

    wrap_drive_fresh(wrapper);

    robot->api_ops->report_motor_status(robot, &robot->api_state, 0);

    /** auto recovery ? */
}

static void robot_recovery_state_run(void *obj)
{

}

static void robot_sleep_state_run(void *obj)
{

}

static void robot_imu_calibration_state_run(void *obj)
{

}

static void robot_motor_calibration_state_entry(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;

    LOG_DBG("ts: %08u %s", robot->timestamp, __func__);
    LOG_INF("Entry Motor Calibration State");

    robot_set_frequency(robot, MOTOR_CAL_RATE);
}

static void robot_motor_calibration_state_run(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;
    struct wrapper_robot_t *wrapper = robot->priv;

    if (robot->motor_zero & 0x01) {
        robot->motor_zero &= ~0x01;
        wrap_drvie_set_zero(wrapper, LEFT);
        LOG_INF("Set Left Zero");
    }

    if (robot->motor_zero & 0x02) {
        robot->motor_zero &= ~0x02;
        wrap_drvie_set_zero(wrapper, RIGHT);
        LOG_INF("Set Right Zero");
    }
}

static void robot_motor_ota_state_entry(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;

    LOG_DBG("ts: %08u %s", robot->timestamp, __func__);
    LOG_INF("Entry Motor OTA State");

    robot_set_frequency(robot, 1);
}

static void robot_motor_ota_state_run(void *obj)
{
    struct robot_sm_t *sm = (struct robot_sm_t *)obj;
    struct robot_t *robot = sm->robot;
    struct wrapper_robot_t *wrapper = robot->priv;

    int ret;

    ret = wrap_drive_ota_upgrade(wrapper);
    if (ret != 0) {
        LOG_INF("%d", ret);
        robot_set_state(robot, ROBOT_FAULT_STATE);
    }
}

static void robot_boot_recovery_mode_state_run(void *obj)
{
}

static void robot_boot_recovery_mode_state_entry(void *obj)
{
    // wait robot is down
    // disable all motors
    // check battery status
}

static const struct smf_state robot_states[] = {
    [ROBOT_SELFTEST_STATE] = SMF_CREATE_STATE(
        robot_selftest_state_entry,
        robot_selftest_state_run,
        robot_selftest_state_exit,
        NULL),
    [ROBOT_READY_STATE] = SMF_CREATE_STATE(
        robot_ready_state_entry,
        robot_ready_state_run,
        NULL,
        NULL),
    [ROBOT_LOCOMOTION_STATE] = SMF_CREATE_STATE(
        robot_locomotion_state_entry,
        robot_locomotion_state_run,
        robot_locomotion_state_exit,
        NULL),
    [ROBOT_DIRECT_STATE] = SMF_CREATE_STATE(
        robot_direct_state_entry,
        robot_direct_state_run,
        robot_direct_state_exit,
        NULL),
    [ROBOT_FAULT_STATE] = SMF_CREATE_STATE(
        robot_fault_state_entry,
        robot_fault_state_run,
        NULL,
        NULL),
    [ROBOT_RECOVERY_STATE] = SMF_CREATE_STATE(
        NULL,
        robot_recovery_state_run,
        NULL,
        NULL),
    [ROBOT_SLEEP_STATE] = SMF_CREATE_STATE(
        NULL,
        robot_sleep_state_run,
        NULL,
        NULL),
    [ROBOT_IMU_CALIBRATION_STATE] = SMF_CREATE_STATE(
        NULL,
        robot_imu_calibration_state_run,
        NULL,
        NULL),
    [ROBOT_MOTOR_CALIBRATION_STATE] = SMF_CREATE_STATE(
        robot_motor_calibration_state_entry,
        robot_motor_calibration_state_run,
        NULL,
        NULL),
    [ROBOT_MOTOR_OTA_STATE] = SMF_CREATE_STATE(
        robot_motor_ota_state_entry,
        robot_motor_ota_state_run,
        NULL,
        NULL),
    [ROBOT_BOOT_RECOVERY_MODE_STATE] = SMF_CREATE_STATE(
        robot_boot_recovery_mode_state_entry,
        robot_boot_recovery_mode_state_run,
        NULL,
        NULL),
};
BUILD_ASSERT(ARRAY_SIZE(robot_states) == ROBOT_STATE_COUNT);

void robot_set_state(struct robot_t *robot, const enum robot_state_t state)
{
    __ASSERT(state < ARRAY_SIZE(robot_states), "invalid robot_state %d", state);
    smf_set_state(SMF_CTX(&robot->sm), &robot_states[state]);
}

enum robot_state_t robot_get_state(struct robot_t *robot)
{
    return robot->sm.ctx.current - &robot_states[0];
}

static void robot_sm_init(struct robot_t *robot, const enum robot_state_t state)
{
    smf_set_initial(SMF_CTX(&robot->sm), &robot_states[state]);
}


/**
 * Locotion Runtime
*/

robot_t *debug_robot;

#define CYCLES_PER_SEC (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC)


static void robot_handler(struct k_work *work);
static void estimator_handler(struct k_work *work);


static void robot_timer_callback(struct k_timer *timer_id);

K_TIMER_DEFINE(robot_timer, robot_timer_callback, NULL);


#define ESTIMATOR_STACK_SIZE 4096
#define ESTIMATOR_WORKQ_PRIORITY 10

K_THREAD_STACK_DEFINE(estimator_stack_area, ESTIMATOR_STACK_SIZE);

struct k_work_q estimator_work_q;

struct estimator_work_t {
    struct k_work work;

    struct robot_t *robot;
};


#define ROBOT_STACK_SIZE 4096
#define ROBOT_WORKQ_PRIORITY -10

K_THREAD_STACK_DEFINE(robot_stack_area, ROBOT_STACK_SIZE);

struct k_work_q robot_work_q;

struct robot_work_t {
    struct k_work work;

    struct wrapper_robot_t *wrapper;

    struct robot_t robot;
};

static void robot_timer_callback(struct k_timer *timer_id)
{
    void *data = k_timer_user_data_get(timer_id);
    struct robot_work_t *robot_work = (struct robot_work_t *)data;
    
    k_work_submit_to_queue(&robot_work_q, &robot_work->work);
}

static void robot_handler(struct k_work *work)
{
    struct robot_work_t *p = CONTAINER_OF(work, struct robot_work_t, work);
    struct wrapper_robot_t *wrapper = p->wrapper;
    struct robot_t *robot = &p->robot;
    struct robot_sm_t *sm = &robot->sm;

    int ret = 0;
    uint32_t now, interval;

    now = sys_clock_cycle_get_32();

    if (robot->last_hw_cycle == 0) {
        // init state
        interval = (float)CYCLES_PER_SEC / robot->frequency;
    } else if (robot->last_hw_cycle > now) {
        // overflow
        interval = (UINT32_MAX - robot->last_hw_cycle) + now;
        robot->overflow_count += 1;
    } else {
        interval = now - robot->last_hw_cycle;
    }

    robot->last_hw_cycle = now;

    robot->dt = interval / (float)CYCLES_PER_SEC;

    float jitter =
        (int)(interval - CYCLES_PER_SEC / robot->frequency) /
        ((float)CYCLES_PER_SEC / robot->frequency) * 100;

#if 0
    if (update_count % 500 == 1) {
        LOG_INF("dt: %+0.6f jitter: %+0.6f%%", dt, jitter);
    }
#else
    if (jitter > 10.0f) { // jitter > 10%, warning
        LOG_ERR("dt: %+0.6f jitter: %+0.6f%%", robot->dt, jitter);
    }
#endif

    uint64_t cycle64 = robot->overflow_count;
    cycle64 = cycle64 << 32 | now;
    robot->timestamp = cycle64 / (CYCLES_PER_SEC / 1000 / 1000);

#if 0
    LOG_INF("%d now: %f %f %f", robot->cycle_count, now / (float)CYCLES_PER_SEC,
            robot->dt, robot->timestamp / 1000000.0);
#endif

    /** update imu raw data */
    ret = sensors_fresh(robot);
    if (ret == 0) {
        /** update ahrs */
        ahrs_update(robot->ahrs, robot->accel, robot->gyro, 0, robot->dt);
        ahrs_get_quaternion(robot->ahrs, robot->quaternion);

#if 0
        if (robot->cycle_count % (robot->frequency / 2) == 1) {
            LOG_INF("DT: %+0.6f Q: %+0.6f %+0.6f %+0.6f %+0.6f", robot->dt,
                    robot->quaternion[0],robot->quaternion[1], robot->quaternion[2], robot->quaternion[3]);
        }
#endif
    } else {
        // TODO: aysnc imu data update
        // LOG_WRN("IMU update failed: %d", ret);
    }

    /** update input */
    if (robot->rc_connected) {
        rc_update_input(&robot->input);
    } else {
        api_update_input(&robot->api_state, &robot->input);
    }

    /** update async estimator */
    if (robot->frequency > robot->estimator_freq &&
        robot->cycle_count % (robot->frequency / robot->estimator_freq) == 0) {
        robot_update_estimator(robot);
    }

    /** check return ready state */
    if ((SMF_CTX(sm)->current == &robot_states[ROBOT_LOCOMOTION_STATE] ||
         SMF_CTX(sm)->current == &robot_states[ROBOT_DIRECT_STATE]) &&
        robot->ready_next == READY_WAITING /* && mode == CRAW */) {
        robot->ready_next = READY_WAITING;
        robot_set_state(robot, ROBOT_READY_STATE);
    }

    /** motor fault or estop mode */
    if ((wrap_drive_check(wrapper, NULL) != 0 ||
         robot->ready_next == ESTOP_MODE) &&
        SMF_CTX(sm)->current != &robot_states[ROBOT_FAULT_STATE]) {
      // force in fault satae
      robot_set_state(robot, ROBOT_FAULT_STATE);
    }

    /** reset motor count */
    if (robot->cycle_count % (robot->frequency * 5) == 0) {
        wrap_drive_reset_count(wrapper);
    }

#if 0 // move to robot_ready_state_run
    if (robot->cycle_count % (robot->frequency / 10) == 0) {
        enum robot_model_t robot_model, old_model;
        old_model = gpio_get_robot_model();
        robot_model = gpio_ctrl_update();
        if (robot_model == ROBOT_MODEL_REAR && old_model == ROBOT_MODEL_FRONT) {
            robot->lock_count = 10;
        }
        
        if (robot_model == ROBOT_MODEL_REAR) {
            robot->motor_index_offset = 8;
        } else {
            robot->motor_index_offset = 0;
        }

        if (robot->lock_count > 0) {
            // LOG_INF("lock %d", robot->lock_count);
            api_canfd_lock_connector(robot, 0, 0);
            robot->lock_count -= 1;
        } else {
            // api_canfd_stop_connector(robot, 0, 0);
        }
    }
#endif

    /** update data from motor */

    /** run state machine */
    smf_run_state(SMF_CTX(sm));

    /** fresh data to motor */

    robot->cycle_count += 1;
}

static void estimator_handler(struct k_work *work)
{
    struct estimator_work_t *p = CONTAINER_OF(work, struct estimator_work_t, work);
    struct robot_t *robot = p->robot;
    struct wrapper_robot_t *wrapper = (struct wrapper_robot_t *)robot->priv;

    wrap_update_estimator(wrapper);

    robot->estimator_updated = 1;
}

int robot_init(struct wrapper_robot_t *wrapper) {
    const struct k_work_queue_config estimator_workq_cfg = {
        .name = "estimatorworkq",
        .no_yield = true,
    };

    k_work_queue_init(&estimator_work_q);

    k_work_queue_start(&estimator_work_q, estimator_stack_area,
                       K_THREAD_STACK_SIZEOF(estimator_stack_area),
                       ESTIMATOR_WORKQ_PRIORITY, &estimator_workq_cfg);

    const struct k_work_queue_config robot_workq_cfg = {
        .name = "robotworkq",
        .no_yield = true,
    };
 
    k_work_queue_init(&robot_work_q);

    k_work_queue_start(&robot_work_q, robot_stack_area,
                       K_THREAD_STACK_SIZEOF(robot_stack_area),
                       ROBOT_WORKQ_PRIORITY, &robot_workq_cfg);

    struct robot_work_t *robot_work =
        (struct robot_work_t *)k_malloc(sizeof(struct robot_work_t));
    memset((void *)robot_work, 0, sizeof(struct robot_work_t));
    robot_work->wrapper = wrapper;

    struct robot_t *robot = &robot_work->robot;
    robot->sm.robot = &robot_work->robot;
    robot->priv = (void *)wrapper;

    // TOOD: move to setting
    enum robot_model_t robot_model = gpio_get_robot_model();
    if (robot_model != ROBOT_MODEL_REAR) {
        robot->motor_index_offset = 0;
    } else {
        robot->motor_index_offset = 8;
    }

    robot->ready_next = FORCE_DIRECT;
    // robot->ready_next = AUTO_LOCOMOTION;
    // robot->canfd_8mbps_mode = 1;
    robot->estimator_freq = 30;

    debug_robot = robot;
    robot->ahrs = ahrs_init(DEFAULT_RATE);
    robot->imu_cfg = wrap_get_imu_config(wrapper, 0);

#if defined(CONFIG_DDT_ROBOT_API_ACM)
    api_acm_init(robot);
#endif

#if defined(CONFIG_DDT_ROBOT_API_CANFD)
    api_canfd_init(robot);
#endif

	hwinfo_get_device_id(robot->hwinfo_id, DEVICE_ID_BIN_MAX_SIZE);

    rc_sbus_init(robot);

    sensors_init(robot);

    k_work_init(&robot_work->work, robot_handler);

    k_timer_user_data_set(&robot_timer, (void *)robot_work);

    robot_set_frequency(robot, DEFAULT_RATE);

    robot_sm_init(robot, ROBOT_SELFTEST_STATE);

    return 0;
}

void robot_update_estimator(struct robot_t *robot)
{
    static struct estimator_work_t *estimator_work = 0;
    
    if (estimator_work == 0) {
        estimator_work = (struct estimator_work_t *)k_malloc(sizeof(struct estimator_work_t));
        memset((void *)estimator_work, 0, sizeof(struct estimator_work_t));

        estimator_work->robot = robot;

        k_work_init(&estimator_work->work, estimator_handler);
    }

    robot->estimator_updated = 0;

    k_work_submit_to_queue(&estimator_work_q, &estimator_work->work);
}

void robot_set_frequency(struct robot_t *robot, uint32_t frequency)
{
    robot->frequency = frequency;
    robot->cycle_count = 0;

    LOG_DBG("ts: %08u %s: %d", robot->timestamp, __func__, frequency);

    ahrs_reset(robot->ahrs, frequency);

    k_timer_start(&robot_timer, K_NO_WAIT, K_USEC(1000000 / robot->frequency));

    wrap_drive_reconfigure(robot->priv, robot->frequency);
}
