

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <math.h>
#include "wrapper.h"

#include "gpio_ctrl.h"

#ifdef CONFIG_DIABLO_CONTROL_A2V2
#include "task/Task.hpp"
#include "middleware/middleware.hpp"
#endif

LOG_MODULE_REGISTER(apollo_wrapper, LOG_LEVEL_INF);

struct wrapper_robot_t {
#ifdef CONFIG_DIABLO_CONTROL_A2V2
    WB6_Task *robot_task;
#endif

    enum motor_model_t motor_model;

    imu_config_t imu_config;

    int loop_count;

    struct motor_group_t motor_groups[STUB_LEG_NUM];
};

void wrap_set_motor_stub(struct wrapper_robot_t *wrapper) {
}

void wrap_update_motor_stub(struct wrapper_robot_t *wrapper)
{
#ifdef CONFIG_DIABLO_CONTROL_A2V2
    struct motor_t *motor = 0;

    for (int id = 0; id < MOTOR_NUM; id++) {
        if (id < 4) {
            motor = &wrapper->motor_groups[LEFT].motors[id];
        } else {
            motor = &wrapper->motor_groups[RIGHT].motors[id - 4];
        }

#if 0
    if (update_count % 1000 == 1) {
        if (id == 0)
            LOG_INF("======================================================");
        if (id == 0 || id == 4)
        LOG_INF("id: %d torque: %+0.3f, velocity: %+0.3f, position: %+0.3f",
            id, in->torque, in->velocity, in->position);
    }
#endif
        Middleware_Class *middle = wrapper->robot_task->get_robot_middleware();

        middle->downbridge.Set_Motor(motor->in.torque, motor->in.velocity, motor->in.position, id);

        wrapper->robot_task->planner->voltage[id] = motor->in_ex.voltage;
    }

    
#endif
}

void wrap_motor_stub_zero(struct wrapper_robot_t *wrapper)
{
    struct motor_out_t *out = 0;

    for (int i = LEFT; i <= RIGHT; i++) {
        for (int j = 0; j < 4; j++) {
            out = &wrapper->motor_groups[i].motors[j].out;

#ifndef TORQUE_ONLY
            out->position = 0.0f;
            out->velocity = 0.0f;
            out->torque = 0.0f;
            out->kp = 0.0f;
            out->kd = 0.0f;
#else
            out->torque = 0.0f;
#endif
        }
    }
}

void wrap_motor_stub_fresh(struct wrapper_robot_t *wrapper)
{
#ifdef CONFIG_DIABLO_CONTROL_A2V2
    struct motor_out_t *out = 0;

    for (int id = 0; id < MOTOR_NUM; id++) {
        if (id < 4) {
            out = &wrapper->motor_groups[LEFT].motors[id].out;
        } else {
            out = &wrapper->motor_groups[RIGHT].motors[id - 4].out;
        }
        Middleware_Class *middle = wrapper->robot_task->get_robot_middleware();

        middle->downbridge.Get_Motor_Torque(out->torque, id);

#if 0
        if (id == 0) {
            middle->downbridge.Get_Abad_Motor_Kp_Kd_Pos_Vel(out->kp, out->kd, out->position, out->velocity, 0);
        } else if (id == 4) {
            middle->downbridge.Get_Abad_Motor_Kp_Kd_Pos_Vel(out->kp, out->kd, out->position, out->velocity, 1);
        }
#endif
    }
#endif
}

void wrap_update_battery_info(const struct wrapper_robot_t *wrapper, struct battery_info_t *batt_info, int count, const float dt)
{
#ifdef CONFIG_DIABLO_CONTROL_A2V2
    wrapper->robot_task->set_robot_battery_status(batt_info[0].voltage > 33000, batt_info[1].voltage > 33000);
#endif   
}

void wrap_update_input(const struct wrapper_robot_t *wrapper, struct input_t *input, const float dt)
{
#ifdef CONFIG_DIABLO_CONTROL_A2V2
    const LWD_RobotModel *param = wrapper->robot_task->get_robot_model();

    if ((input->stand_mode == STAND_IDLE || input->stand_mode == STAND_CRAW) && input->jump_state == JUMP_JUMP) {
        wrapper->robot_task->emergency_braking(true);
    } else {
        wrapper->robot_task->emergency_braking(false);
    }
    if (wrapper->robot_task->get_loco_get_wrapper()->robot_mode == Robot_Class::CRASH) {
        input->stand_mode = STAND_IDLE;
    }
    if(input->stand_mode == STAND_STAND) {
        wrapper->robot_task->set_stand(true);
    } else {
        wrapper->robot_task->set_stand(false);
    }

    // wrapper->robot_task->set_robot_vel(input->forward, input->yaw);
    wrapper->robot_task->planner->get_cmd()->forward.input = input->forward;
    if (input->forward_accel != 0.0f) {
        wrapper->robot_task->planner->get_cmd()->forward.acceleration_switch = true;
        wrapper->robot_task->planner->get_cmd()->forward.acceleration_input = input->forward_accel;
    } else {
        wrapper->robot_task->planner->get_cmd()->forward.acceleration_switch = false;
        wrapper->robot_task->planner->get_cmd()->forward.acceleration_input = 0.0f;
    }
    wrapper->robot_task->planner->get_cmd()->yaw.input = input->yaw;
    if (input->yaw_accel != 0.0f) {
        wrapper->robot_task->planner->get_cmd()->yaw.acceleration_switch = true;
        wrapper->robot_task->planner->get_cmd()->yaw.acceleration_input = input->yaw_accel;
    } else {
        wrapper->robot_task->planner->get_cmd()->yaw.acceleration_switch = false;
        wrapper->robot_task->planner->get_cmd()->yaw.acceleration_input = 0.0f;
    }
    wrapper->robot_task->set_robot_pitch(input->pitch, Command::INPUT_POS);
    wrapper->robot_task->set_roll_angle(input->roll);
    
    float height = input->height;
    // if (height < param->body_height_min)
    //     height = param->body_height_min * 1.1;
    // if (height > param->body_height_max)
    //     height = param->body_height_max * 0.9;
    wrapper->robot_task->set_height(height, Command::INPUT_POS);
    wrapper->robot_task->set_tilt(input->tilt);

    if (input->jump_state == JUMP_JUMP) {
        wrapper->robot_task->set_jump_cmd(true, 0.0, 0, 0);
        // input->jump_state = JUMP_IDLE;
    } else {
        wrapper->robot_task->set_jump_cmd(false, 0.0, 0, 0);
        // input->jump_state = JUMP_IDLE;
    }
#endif
}

void wrap_update_status(const struct wrapper_robot_t *wrapper, struct api_locomotion_status_t status[], const float dt)
{
#ifdef CONFIG_DIABLO_CONTROL_A2V2
    status[0].key = 0x1001;
    status[0].value.u8 = wrapper->robot_task->get_loco_get_wrapper()->robot_mode;
    
    status[1].key = 0x1002;
    status[1].value.u8 = wrapper->robot_task->get_loco_get_wrapper()->jump_status;
    
    // status[1].key = 0x1101;
    // memcpy(status[1].value.f32x9, wrapper->robot_task->get_loco_get_wrapper()->estimator.position, sizeof(status[0].value.f32x9));

    // status[2].key = 0x1102;
    // memcpy(status[2].value.f32x9, wrapper->robot_task->get_loco_get_wrapper()->estimator.velocity, sizeof(status[0].value.f32x9));
#endif
}

struct motor_group_t *wrap_get_motor_group(struct wrapper_robot_t *wrapper, int i)
{
    return &wrapper->motor_groups[i];
}

enum motor_model_t wrap_get_motor_model(struct wrapper_robot_t *wrapper)
{
    int detect_z1 = 1;
    int detect_p1010 = 1;
    int detect_mg10015 = 1;

#ifdef P8
    wrapper->motor_groups[LEFT].motors[M_ABAD].type = MT_P1010;
    wrapper->motor_groups[LEFT].motors[M_HIP].type = MT_P1010;
    wrapper->motor_groups[LEFT].motors[M_KNEE].type = MT_P1010;

    wrapper->motor_groups[RIGHT].motors[M_ABAD].type = MT_P1010;
    wrapper->motor_groups[RIGHT].motors[M_HIP].type = MT_P1010;
    wrapper->motor_groups[RIGHT].motors[M_KNEE].type = MT_P1010;
#endif

    if (wrapper->motor_model == MODEL_UNKNOWN) {
        for (int i = LEFT; i <= RIGHT; i++) {
            for (int j = M_ABAD; j <= M_KNEE; j++) {
                detect_z1 = detect_z1 && wrapper->motor_groups[i].motors[j].type == MT_Z1;
                detect_p1010 = detect_p1010 && wrapper->motor_groups[i].motors[j].type == MT_P1010;
                detect_mg10015 = detect_mg10015 && wrapper->motor_groups[i].motors[j].type == MT_MG10015;
            }
        }

        if (detect_z1 == 1) {

            wrapper->motor_groups[LEFT].motors[M_ABAD].direction = FORWARD;
            wrapper->motor_groups[LEFT].motors[M_HIP].direction = REVERSE;
            wrapper->motor_groups[LEFT].motors[M_KNEE].direction = REVERSE;

            wrapper->motor_groups[RIGHT].motors[M_ABAD].direction = FORWARD;
            wrapper->motor_groups[RIGHT].motors[M_HIP].direction = FORWARD;
            wrapper->motor_groups[RIGHT].motors[M_KNEE].direction = FORWARD;

            wrapper->motor_groups[LEFT].motors[M_ABAD].zero_angle_bias = +(45.0f / 360.0f) * 2 * M_PI;
            wrapper->motor_groups[LEFT].motors[M_HIP].zero_angle_bias = -(160.0f / 360.0f) * 2 * M_PI;
            wrapper->motor_groups[LEFT].motors[M_KNEE].zero_angle_bias = -(40.0f / 360.0f)  * 2 * M_PI;
            
            wrapper->motor_groups[RIGHT].motors[M_ABAD].zero_angle_bias = -(45.0f / 360.0f) * 2 * M_PI;
            wrapper->motor_groups[RIGHT].motors[M_HIP].zero_angle_bias = -(160.0f / 360.0f) * 2 * M_PI;
            wrapper->motor_groups[RIGHT].motors[M_KNEE].zero_angle_bias = -(40.0f / 360.0f)  * 2 * M_PI;

            wrapper->motor_groups[LEFT].motor_status = 0;
            wrapper->motor_groups[RIGHT].motor_status = 0;
            
            if (wrapper->motor_groups[LEFT].motors[M_ANKLE].type == MT_M1505
                    && wrapper->motor_groups[RIGHT].motors[M_ANKLE].type == MT_M1505) {
                wrapper->motor_model = MODEL_Z1_M1505;
                // TODO: set direction
            } else if (wrapper->motor_groups[LEFT].motors[M_ANKLE].type == MT_RS1
                    && wrapper->motor_groups[RIGHT].motors[M_ANKLE].type == MT_RS1) {
                wrapper->motor_model = MODEL_Z1_RS1;
                wrapper->motor_groups[LEFT].motors[M_ANKLE].direction = REVERSE;
                wrapper->motor_groups[RIGHT].motors[M_ANKLE].direction = FORWARD;
            }

        } else if (detect_p1010 == 1) {

            // wrapper->motor_model = MODEL_P1010_M1505;
            wrapper->motor_groups[LEFT].motors[M_ABAD].direction = REVERSE;
            wrapper->motor_groups[LEFT].motors[M_HIP].direction = FORWARD;
            wrapper->motor_groups[LEFT].motors[M_KNEE].direction = FORWARD;
            wrapper->motor_groups[RIGHT].motors[M_ABAD].direction = REVERSE;
            wrapper->motor_groups[RIGHT].motors[M_HIP].direction = REVERSE;
            wrapper->motor_groups[RIGHT].motors[M_KNEE].direction = REVERSE;

#ifdef P8
            wrapper->motor_groups[LEFT].motors[M_ABAD].direction = FORWARD;
            wrapper->motor_groups[LEFT].motors[M_HIP].direction = REVERSE;
            wrapper->motor_groups[LEFT].motors[M_KNEE].direction = REVERSE;
            wrapper->motor_groups[RIGHT].motors[M_ABAD].direction = FORWARD;
            wrapper->motor_groups[RIGHT].motors[M_HIP].direction = FORWARD;
            wrapper->motor_groups[RIGHT].motors[M_KNEE].direction = FORWARD;
#endif
            
            if (gpio_ctrl_dvt3_mark == 1) { // DVT3
                wrapper->motor_groups[LEFT].motors[M_HIP].zero_angle_bias = -((165.0) / 360.0f) * 2 * M_PI;
                wrapper->motor_groups[RIGHT].motors[M_HIP].zero_angle_bias = -((165.0) / 360.0f) * 2 * M_PI;
            } else { // DVT2
                wrapper->motor_groups[LEFT].motors[M_HIP].zero_angle_bias = -((165.0) / 360.0f) * 2 * M_PI;
                wrapper->motor_groups[RIGHT].motors[M_HIP].zero_angle_bias = -((165.0) / 360.0f) * 2 * M_PI;
            }


            wrapper->motor_groups[LEFT].motor_status = 0;
            wrapper->motor_groups[RIGHT].motor_status = 0;

            // for point foot force MT_M1505

            if (wrapper->motor_groups[LEFT].motors[M_ANKLE].type == MT_UNKNOWN
                    && wrapper->motor_groups[RIGHT].motors[M_ANKLE].type == MT_UNKNOWN) {
                wrapper->motor_groups[LEFT].motors[M_ANKLE].type = MT_M1505;
                wrapper->motor_groups[RIGHT].motors[M_ANKLE].type = MT_M1505;
            }

            if (wrapper->motor_groups[LEFT].motors[M_ANKLE].type == MT_M1505
                    && wrapper->motor_groups[RIGHT].motors[M_ANKLE].type == MT_M1505) {
                wrapper->motor_model = MODEL_P1010_M1505;
                // TODO: set direction
            } else if (wrapper->motor_groups[LEFT].motors[M_ANKLE].type == MT_RS1
                    && wrapper->motor_groups[RIGHT].motors[M_ANKLE].type == MT_RS1) {
                wrapper->motor_model = MODEL_P1010_RS1;
                wrapper->motor_groups[LEFT].motors[M_ANKLE].direction = REVERSE;
                wrapper->motor_groups[RIGHT].motors[M_ANKLE].direction = FORWARD;
            } else if (wrapper->motor_groups[LEFT].motors[M_ANKLE].type == MT_DM8006
                    && wrapper->motor_groups[RIGHT].motors[M_ANKLE].type == MT_DM8006) {
                wrapper->motor_model = MODEL_P1010_DM8006;
                wrapper->motor_groups[LEFT].motors[M_ANKLE].direction = FORWARD;
                wrapper->motor_groups[RIGHT].motors[M_ANKLE].direction = REVERSE;
            } else if (wrapper->motor_groups[LEFT].motors[M_ANKLE].type == MT_P1010
                    && wrapper->motor_groups[RIGHT].motors[M_ANKLE].type == MT_P1010) {
                wrapper->motor_model = MODEL_P1010_P1010;
            }

            if (wrapper->motor_model == MODEL_P1010_DM8006) {
                wrapper->motor_groups[LEFT].motors[M_HIP].zero_angle_bias = +((180.0f) / 360.0f) * 2 * M_PI;
                wrapper->motor_groups[RIGHT].motors[M_HIP].zero_angle_bias = +((180.0f) / 360.0f) * 2 * M_PI;
                wrapper->motor_groups[LEFT].motors[M_KNEE].zero_angle_bias = -((49.73f) / 360.0f) * 2 * M_PI;
                wrapper->motor_groups[RIGHT].motors[M_KNEE].zero_angle_bias = -((49.73f) / 360.0f) * 2 * M_PI;
            }

        } else if(detect_mg10015 == 1) {
            // wrapper->motor_model = MODEL_MG10015_M1505;
            wrapper->motor_groups[LEFT].motors[M_ABAD].direction = FORWARD;
            wrapper->motor_groups[LEFT].motors[M_HIP].direction = REVERSE;
            wrapper->motor_groups[LEFT].motors[M_KNEE].direction = REVERSE;
            wrapper->motor_groups[RIGHT].motors[M_ABAD].direction = FORWARD;
            wrapper->motor_groups[RIGHT].motors[M_HIP].direction = FORWARD;
            wrapper->motor_groups[RIGHT].motors[M_KNEE].direction = FORWARD;

            wrapper->motor_groups[LEFT].motors[M_HIP].zero_angle_bias = +(180.0f / 360.0f) * 2 * M_PI;
            wrapper->motor_groups[RIGHT].motors[M_HIP].zero_angle_bias = +(180.0f / 360.0f) * 2 * M_PI;

            wrapper->motor_groups[LEFT].motor_status = 0;
            wrapper->motor_groups[RIGHT].motor_status = 0;

            if (wrapper->motor_groups[RIGHT].motors[M_ANKLE].type == MT_M1505
                    && wrapper->motor_groups[LEFT].motors[M_ANKLE].type == MT_M1505) {
                wrapper->motor_model = MODEL_MG10015_M1505;
            }
        } else {
            wrapper->motor_model = MODEL_UNKNOWN;

            for (int i = LEFT; i <= RIGHT; i++) {
                // for point foot, ignored ankle check
                for (int j = M_ABAD; j < M_ANKLE; j++) {
                    if (wrapper->motor_groups[i].motors[j].type == MT_MG10015
                            || wrapper->motor_groups[i].motors[j].type ==  MT_UNKNOWN)
                        wrapper->motor_groups[i].motor_status |= BIT(j * 8);
                }
            }
        }

        if (wrapper->motor_model == MODEL_P1010_M1505)
            if (gpio_ctrl_dvt3_mark == 1) {
                LOG_INF("Motor Model: [P1010+M1505] [DVT3]");
            } else {
                LOG_INF("Motor Model: [P1010+M1505] [DVT2]");
            }
        else if (wrapper->motor_model == MODEL_MG10015_M1505)
            LOG_INF("Motor Model: [MG10015+M1505]");
        else if (wrapper->motor_model == MODEL_P1010_RS1)
            LOG_INF("Motor Model: [P1010+RS1]");
        else if (wrapper->motor_model == MODEL_Z1_M1505)
            LOG_INF("Motor Model: [Z1+M1505]");
        else if (wrapper->motor_model == MODEL_Z1_RS1)
            LOG_INF("Motor Model: [Z1+RS1]");
        // else
        //     LOG_ERR("Not Suport Motor Model, 0x%02x", wrapper->motor_model);
    }

    return wrapper->motor_model;
}

int wrap_update_imu_data(struct wrapper_robot_t *wrapper, const float accel[3], const float gyro[3], const float q[4], const float dt)
{
#ifdef CONFIG_DIABLO_CONTROL_A2V2
    Middleware_Class *middle = wrapper->robot_task->get_robot_middleware();

    middle->downbridge.Set_Accl(accel[0], accel[1], accel[2]);
    middle->downbridge.Set_Gyro(gyro[0], gyro[1], gyro[2]);
    // xyzw to wxyz
    middle->downbridge.Set_Quaternion(q[3], q[0], q[1], q[2]);
#endif

    return 0;
}

struct wrapper_robot_t *new_wrapper()
{
    struct wrapper_robot_t *wrapper = (struct wrapper_robot_t *)k_malloc(sizeof(struct wrapper_robot_t));

    memset((void *)wrapper, 0, sizeof(struct wrapper_robot_t));

    return wrapper;
}

void wrap_robot_deinit(struct wrapper_robot_t *wrapper)
{
}

void wrap_robot_init(struct wrapper_robot_t *wrapper)
{
    wrapper->loop_count = 0;

#if 1
    wrapper->imu_config.x_map = IMU_AXIS_Y;
    wrapper->imu_config.x_dir = IMU_AXIS_DIR_FR;
    wrapper->imu_config.y_map = IMU_AXIS_X;
    wrapper->imu_config.y_dir = IMU_AXIS_DIR_REV;
    wrapper->imu_config.z_map = IMU_AXIS_Z;
    wrapper->imu_config.z_dir = IMU_AXIS_DIR_FR;
#else
    wrapper->imu_config.x_map = IMU_AXIS_X;
    wrapper->imu_config.x_dir = IMU_AXIS_DIR_FR;
    wrapper->imu_config.y_map = IMU_AXIS_Y;
    wrapper->imu_config.y_dir = IMU_AXIS_DIR_REV;
    wrapper->imu_config.z_map = IMU_AXIS_Z;
    wrapper->imu_config.z_dir = IMU_AXIS_DIR_REV;
#endif

    wrap_set_motor_stub(wrapper);
}

#if 0
void wrap_plan_start(const struct wrapper_robot_t *wrapper)
{
    // wrapper->task->plan_start();
}

void wrap_plan_update(const struct wrapper_robot_t *wrapper, const float dt)
{

    // wrapper->task->plan_update(dt);
}

void wrap_ctrl_start(const struct wrapper_robot_t *wrapper)
{
    // wrapper->task->ctrl_start();
}

void wrap_ctrl_update(const struct wrapper_robot_t *wrapper, const float dt)
{
    // wrapper->task->ctrl_update(dt);
}

#else

void wrap_ctrl_new(struct wrapper_robot_t *wrapper, const uint32_t frequency)
{
#ifdef CONFIG_DIABLO_CONTROL_A2V2
    if (wrapper->robot_task != 0) {
        delete wrapper->robot_task;
        wrapper->robot_task = 0;
    }

    enum robot_ctrl_model_t ctrl_model = ROBOT_P1010_M1505;

    if (wrapper->motor_model == MODEL_MG10015_M1505) {
        ctrl_model = ROBOT_MG10015_M1505;
    }
    
    wrapper->robot_task = new WB6_Task(ctrl_model);
    wrapper->robot_task->set_robot_control_frequency(frequency);
#endif
}

void wrap_ctrl_loop(struct wrapper_robot_t *wrapper, const float dt)
{
#ifdef CONFIG_DIABLO_CONTROL_A2V2
    switch (wrapper->loop_count) {
        case 11:
            wrapper->robot_task->Run_Medium(dt);
            wrapper->robot_task->Run_Fast(dt);
            break;
        default:
            wrapper->loop_count += 1;
            break;
    }
#endif
}

void wrap_update_estimator(struct wrapper_robot_t *wrapper)
{
    // do update estimator in here
#ifdef CONFIG_DIABLO_CONTROL_A2V2
  if (wrapper->robot_task == 0)
    return;
  solver * solver_2 = wrapper->robot_task->get_lqr_k_solver();
  solver_2->updateKlist();
#endif
}

struct imu_config_t wrap_get_imu_config(struct wrapper_robot_t *wrapper, int index)
{
    return wrapper->imu_config;
}


#endif
