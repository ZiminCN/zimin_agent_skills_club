#include "Attitude_Comp.hpp"
#include "Quaternion.hpp"
#include <string.h>

Attitude_Complementary::Attitude_Complementary(
    const float w_accl,
    const float w_gyro,
    const float gyro_bias_max) : w_accl(w_accl), w_gyro(w_gyro), gyro_bias_max(gyro_bias_max),
                                 gyro_bias{0}, init_accl{0},
                                 init_dt_gyro(1e-3), init_dt_accl(1e-3),
                                 q{1, 0, 0, 0}, rotation{0}, accl{0},
                                 linear_accl{0}, init(false), update_cnt(0)
{
    init = false;
}

void Attitude_Complementary::init_state(const float init_accl[3])
{
    float Reb[3][3] = {0};
    memcpy(&(Reb[2][0]), init_accl, 12);
    vector_normalize(Reb[2], 3);

    Reb[0][0] = Reb[2][2];
    Reb[0][1] = 0;
    Reb[0][2] = -Reb[0][2];
    vector_normalize(Reb[0], 3);
    vector3_cross(Reb[2], Reb[0], Reb[1]);

    q = Quaternion(Reb);
    init = true;
}

void Attitude_Complementary::update_gyro(const float gyro_meas[3], const float stamp, float dt)
{
    static float yaw_last_gyro = 0, yaw_gyro = 0, start_init = 1;
    static uint32_t stop_count = 0;

    if (!init)
    {

        if (mabs(gyro_meas[2]) > 0.06)
        {

            memset(gyro_bias, 0, 12);
            memset(init_accl, 0, 12);
            update_cnt = 0;
            average_cnt = 0;
            return;
        }
        if (average_cnt < 1500)
        {
            gyro_bias[0] += gyro_meas[0] / 1500.f;
            gyro_bias[1] += gyro_meas[1] / 1500.f;
            gyro_bias[2] += gyro_meas[2] / 1500.f;
            average_cnt++;
        }

        return;
    }
    //    if(start_init){
    //     yaw_last_gyro = yaw_gyro = data.rotation[0];
    //     start_init = 0;
    //    }

    rotation[0] = gyro_meas[0] - gyro_bias[0];
    rotation[1] = gyro_meas[1] - gyro_bias[1];
    rotation[2] = gyro_meas[2] - gyro_bias[2];

    deadzone(rotation[2], 0.01f);
    //     yaw_gyro = data.rotation[2];
    // if(robot.cmd.yaw.d_val == 0)
    // {
    //     stop_count++;
    // }
    // else
    //      stop_count = 0;

    // if(stop_count>5000){
    //     if(robot.cmd.yaw.d_val == 0 &&yaw_gyro>(yaw_last_gyro+0.01)){
    //        data.rotation[2] = yaw_gyro;
    //        yaw_last_gyro = data.rotation[2];
    //     }
    //     else if(robot.cmd.yaw.d_val == 0 && yaw_gyro<(yaw_last_gyro-0.01)){
    //        data.rotation[2] = yaw_gyro;
    //        yaw_last_gyro = data.rotation[2];
    //     }
    //     else
    //         data.rotation[2] = yaw_last_gyro;
    // }

    if (!(dt > 0))
        dt = init_dt_gyro;

    Quaternion dq(rotation, dt);
    q *= dq;
}

bool Attitude_Complementary::update_accl(const float accl_meas[3], const float stamp, float dt)
{
    memcpy(accl, accl_meas, 12);

    if (!init)
    {
        float norm = vector_norm<float>(accl_meas, 3);
        if (norm > 1.05f || norm < 0.95f)
            return false;

        init_accl[0] += accl_meas[0];
        init_accl[1] += accl_meas[1];
        init_accl[2] += accl_meas[2];
        update_cnt++;

        if (update_cnt < 20)
            return true;

        float Reb[3][3] = {0};
        memcpy(&(Reb[2][0]), init_accl, 12);
        vector_normalize<float>(Reb[2], 3);

        Reb[0][0] = Reb[2][2];
        Reb[0][1] = 0;
        Reb[0][2] = -Reb[0][2];
        vector_normalize<float>(Reb[0], 3);
        vector3_cross<float>(Reb[2], Reb[0], Reb[1]);

        q = Quaternion(Reb);
        if (update_cnt > 2000)
        {
            init = true;
            update_cnt = 2000;
        }

        return true;
    }
    else
    {
        if (!(dt > 0))
            dt = init_dt_accl;

        float corr[3];
        float n_accl = vector_norm<float>(accl_meas, 3);

        float v2[3];
        // Quaternion q = q;
        v2[0] = 2 * (q.x * q.z - q.w * q.y);
        v2[1] = 2 * (q.y * q.z + q.w * q.x);
        v2[2] = 1 - 2 * (q.x * q.x + q.y * q.y);
        for (uint8_t i = 0; i < 3; i++)
            linear_accl[i] = accl_meas[i] - v2[i];

        if (n_accl < 1.05f && n_accl > 0.95f)
        {
            float accel_corr[3], norm_accel[3];
            for (uint8_t i = 0; i < 3; i++)
                norm_accel[i] = accl_meas[i] / n_accl;

            vector3_cross(norm_accel, v2, accel_corr);
            for (uint8_t i = 0; i < 3; i++)
                corr[i] = accel_corr[i] * w_accl;

            if (rotation[0] < 0.1f && rotation[0] > -0.1f &&
                rotation[1] < 0.1f && rotation[1] > -0.1f &&
                rotation[2] < 0.1f && rotation[2] > -0.1f)
                for (uint8_t i = 0; i < 3; i++)
                {
                    gyro_bias[i] -= corr[i] * (w_gyro * dt);
                    bound<float>(gyro_bias[i], gyro_bias_max);
                }
        }
        else
            return false;

        Quaternion dq(corr, dt);
        q *= dq;
        update_cnt++;
        if(update_cnt > 5000)
        {
            update_cnt = 5000;
        }
        return true;
    }
}
