

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "wrapper.h"
#include "rc_sbus.h"

#include "motor_stub.hpp"
#include "parameter_zephyr.hpp"


#include <Controller.hpp>
#include <Attitude_Comp.hpp>
#include <Task.hpp>
#include <Cmd.hpp>
#include <imu.h>

#include <SystemFunc.hpp>

LOG_MODULE_REGISTER(diablo_hal, LOG_LEVEL_INF);

struct wrapper_robot_t {
    WB6_Parameter *param;
    WB6_Base *base;
    WB6_Task *task;
    Attitude_Complementary *atti;

    IMUStruct *imu;

    uint32_t started;
    imu_config_t imu_config;

    // motor stub and motor groups
    struct motor_group_t motor_groups[STUB_LEG_NUM];
    motor_stub *motor_stubs[STUB_LEG_NUM * STUB_MOTOR_NUM];
};

void wrap_set_motor_stub(struct wrapper_robot_t *wrapper) {
    WB6_Base *base = wrapper->base;

#if defined(CONFIG_DIABLO_SIX_DOF)
    wrapper->motor_stubs[0] = new motor_stub();
    wrapper->motor_stubs[1] = new motor_stub();
    wrapper->motor_stubs[2] = new motor_stub();
    
    wrapper->motor_stubs[3] = new motor_stub();
    wrapper->motor_stubs[4] = new motor_stub();
    wrapper->motor_stubs[5] = new motor_stub();

    base->legL.hip = wrapper->motor_stubs[0];
    base->legL.knee = wrapper->motor_stubs[1];
    base->legL.wheel = wrapper->motor_stubs[2];

    base->legR.hip = wrapper->motor_stubs[3];
    base->legR.knee = wrapper->motor_stubs[4];
    base->legR.wheel = wrapper->motor_stubs[5];
#else
#error "diablo-control only support SIX dof"
#endif
}

void wrap_fresh_motor_stub(struct wrapper_robot_t *wrapper) {
    struct motor_group_t *group = 0;
    
    group = &wrapper->motor_groups[0];

    wrapper->motor_stubs[0]->refresh(&group->motors[M_HIP].out.torque);
    wrapper->motor_stubs[1]->refresh(&group->motors[M_KNEE].out.torque);
    wrapper->motor_stubs[2]->refresh(&group->motors[M_ANKLE].out.torque);

    group = &wrapper->motor_groups[1];

    wrapper->motor_stubs[3]->refresh(&group->motors[M_HIP].out.torque);
    wrapper->motor_stubs[4]->refresh(&group->motors[M_KNEE].out.torque);
    wrapper->motor_stubs[5]->refresh(&group->motors[M_ANKLE].out.torque);
}

void wrap_update_motor_stub(struct wrapper_robot_t *wrapper) {
    struct motor_group_t *group = 0;
    
    group = &wrapper->motor_groups[LEFT];

    for (int i = 0; i <= M_ANKLE; i++) {
        wrapper->motor_stubs[i]->update(group->motors[i].in.position,
                                        group->motors[i].in.velocity,
                                        group->motors[i].in.torque);
    }

    group = &wrapper->motor_groups[RIGHT];

    for (int i = 0; i <= M_ANKLE; i++) {
        wrapper->motor_stubs[3 + i]->update(group->motors[i].in.position,
                                            group->motors[i].in.velocity,
                                            group->motors[i].in.torque);
    }
}

struct motor_group_t *wrap_get_motor_group(struct wrapper_robot_t *wrapper, int i) {
    return &wrapper->motor_groups[i];
}

int wrap_update_imu_data(struct wrapper_robot_t *wrapper, const float accel[3], const float gyro[3], const float q[4], const float dt)
{
    IMUStruct *imu = wrapper->imu;

    imu->accelData[imu->_config.x_map] = imu->_config.x_dir * (accel[0] - imu->accl_bias[0]) / imu->accl_scale[0];
    imu->accelData[imu->_config.y_map] = imu->_config.y_dir * (accel[1] - imu->accl_bias[1]) / imu->accl_scale[1];
    imu->accelData[imu->_config.z_map] = imu->_config.z_dir * (accel[2] - imu->accl_bias[2]) / imu->accl_scale[2];

    imu->accl_update_flag = true;
    imu->accl_stamp += dt;

    imu->gyroData[imu->_config.x_map] = imu->_config.x_dir * (gyro[0]  - imu->gyro_bias[0]);
    imu->gyroData[imu->_config.y_map] = imu->_config.y_dir * (gyro[1]  - imu->gyro_bias[1]);
    imu->gyroData[imu->_config.z_map] = imu->_config.z_dir * (gyro[2]  - imu->gyro_bias[2]);

    imu->gyro_update_flag = true;
    imu->gyro_stamp += dt;

    imu->update_cnt += 1;
    imu->temperature = 0;


    wrapper->atti->update_accl(imu->accelData, imu->accl_stamp, dt);
    wrapper->atti->update_gyro(imu->gyroData, imu->gyro_stamp, dt);
    wrapper->atti->update_heading();

    // if (wrapper->atti->data.update_cnt % 20) {
    //     wrapper->atti->data.q.normalize();
    // }

#if 0
    if (wrapper->atti->data.update_cnt % 500 == 1) {
        Quaternion *q = wrapper->atti->data.getQuaternion();
        LOG_INF("Quat[%+0.3f %+0.3f %+0.3f %+0.3f]", q->x, q->y, q->y, q->z);
    }
#endif

    return 0;
}

void wrap_update_input(const struct wrapper_robot_t *wrapper, struct input_t *input, const float dt)
{
    RC_Cmd *rc = &wrapper->base->rc;

    rc->move_forward = input->forward;
    rc->move_left = input->yaw;

    rc->head_up = input->pitch;
    rc->tilt_left = input->roll;
}

struct wrapper_robot_t *new_wrapper()
{
    struct wrapper_robot_t *wrapper = (struct wrapper_robot_t *)k_malloc(sizeof(struct wrapper_robot_t));

    memset((void *)wrapper, 0, sizeof(struct wrapper_robot_t));

    return wrapper;
}

void wrap_robot_deinit(struct wrapper_robot_t *wrapper)
{
    if (wrapper->param) {
        delete wrapper->param;
        wrapper->param = 0;
    }

    if (wrapper->base) {
        delete wrapper->base;
        wrapper->base = 0;
    }

    if (wrapper->task) {
        delete wrapper->task;
        wrapper->task = 0;
    }

    if (wrapper->atti) {
        delete wrapper->atti;
        wrapper->atti = 0;
    }
}

void wrap_robot_init(struct wrapper_robot_t *wrapper, enum motor_model_t motor_model)
{
        
    parameter_zephyr *param = new parameter_zephyr(param_core, param_lqr, param_estimation, param_jump);

    WB6_Base *base = new WB6_Base(*param, LEG_CONFIG_BACKWARD);
    param->load(base);

    Attitude_Complementary *atti = new Attitude_Complementary(*base);

    WB6_Task *task = new WB6_Task(*base, *param);
    task->estimator.atti = atti;

    IMUStruct *imu = new IMUStruct();
#if CONFIG_BOARD_DIRECTDRIVETECH_APOLLO || \
    CONFIG_BOARD_DIRECTDRIVETECH_APOLLO_EVK
    imu->_config.x_map = IMU_AXIS_X;
    imu->_config.y_map = IMU_AXIS_Y;
    imu->_config.z_map = IMU_AXIS_Z;
    imu->_config.x_dir = IMU_AXIS_DIR_REV;
    imu->_config.y_dir = IMU_AXIS_DIR_REV;
    imu->_config.z_dir = IMU_AXIS_DIR_FR;
#elif CONFIG_BOARD_DIRECTDRIVETECH_DIABLO_ATHENA
    imu->_config.x_map = IMU_AXIS_Y;
    imu->_config.y_map = IMU_AXIS_Z;
    imu->_config.z_map = IMU_AXIS_X;
    imu->_config.x_dir = IMU_AXIS_DIR_REV;
    imu->_config.y_dir = IMU_AXIS_DIR_REV;
    imu->_config.z_dir = IMU_AXIS_DIR_FR;
#else
#error "Not Support Board"
    imu->_config.x_map = IMU_AXIS_X; 
    imu->_config.y_map = IMU_AXIS_Y;
    imu->_config.z_map = IMU_AXIS_Z;
    imu->_config.x_dir = IMU_AXIS_DIR_FR;
    imu->_config.y_dir = IMU_AXIS_DIR_FR;
    imu->_config.z_dir = IMU_AXIS_DIR_FR;
#endif

    // TODO: read form parameter
    imu->accl_scale[0] = imu->accl_scale[1] = imu->accl_scale[2] = 1.0f;
    imu->accl_bias[0] = imu->accl_bias[1] = imu->accl_bias[2] = 0.0f;
    imu->gyro_bias[0] = imu->gyro_bias[1] = imu->gyro_bias[2] = 0.0f;

    imu->accl_stamp = imu->gyro_stamp = 0;


    // store all in wrapper
    wrapper->param = param;
    wrapper->base = base;
    wrapper->task = task;
    wrapper->atti = atti;
    wrapper->imu = imu;

    wrapper->imu_config.x_map = IMU_AXIS_Z;
    wrapper->imu_config.x_dir = IMU_AXIS_DIR_REV;
    wrapper->imu_config.y_map = IMU_AXIS_X;
    wrapper->imu_config.y_dir = IMU_AXIS_DIR_FR;
    wrapper->imu_config.z_map = IMU_AXIS_Y;
    wrapper->imu_config.z_dir = IMU_AXIS_DIR_REV;

    wrap_set_motor_stub(wrapper);
}

#if 0
void wrap_plan_start(const struct wrapper_robot_t *wrapper)
{
    wrapper->task->plan_start();
}

void wrap_plan_update(const struct wrapper_robot_t *wrapper, const float dt)
{

    wrapper->task->plan_update(dt);
}

void wrap_ctrl_start(const struct wrapper_robot_t *wrapper)
{
    wrapper->task->ctrl_start();
}

void wrap_ctrl_update(const struct wrapper_robot_t *wrapper, const float dt)
{
    wrapper->task->ctrl_update(dt);
}
#else
void wrap_ctrl_loop(struct wrapper_robot_t *wrapper, const float dt)
{
    switch (wrapper->started) {
        case 0:
            wrapper->task->plan_start();
            wrapper->task->ctrl_start();
            wrapper->started = 1;
            break;
    
        case 1:
            wrapper->task->plan_update(dt);
            wrapper->task->ctrl_update(dt);
            break;
        
        default:
            break;
    }
}
#endif

void wrap_motor_stub_fresh(struct wrapper_robot_t *wrapper)
{
}

void wrap_update_twist_input(const struct wrapper_robot_t *wrapper, const struct api_twist_t *twist, const float dt)
{
}

void wrap_update_estimator(struct wrapper_robot_t *wrapper)
{
    // do update estimator in here
}

struct imu_config_t wrap_get_imu_config(struct wrapper_robot_t *wrapper, int index)
{
    return wrapper->imu_config;
}

void System_Lock::on() {}
void System_Lock::off() {}
