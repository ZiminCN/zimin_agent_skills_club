#pragma once

#include <Joint_motor.hpp>

class motor_stub: public Joint_motor
{
public:
    motor_stub(): Joint_motor(10.0f, 1.0f, 1.0f, 1.0f)
    {
        // stub fake init
        this->init = true;
        this->rev = this->update_cnt = 10;

        this->torque_set = this->vq_set = this->iq_set = 0;
        this->round_angle = this->max_torque = this->min_torque = this->max_vel = 0;

        // stub only support torque mode
        this->dir = DIR_CW;
        this->mode = this->mode_set = CTRL_TORQUE;
    }

    void update(const float angle, const float angle_vel, const float torque) {
        this->angle = this->round_angle = angle * this->dir;
        this->angle_vel  = angle_vel * this->dir;
        this->torque     = torque * this->dir;

        this->update_cnt++;
    }

    int refresh(float *torque) {
        *torque = this->torque_set;

        return 0;
    };
};
