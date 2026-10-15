#pragma once

#include <stdint.h>
#include <string.h>
#include "Quaternion.hpp"

class Attitude_Complementary
{
public:
    Attitude_Complementary(const float w_accl        = 0.6f, 
                           const float w_gyro        = 0.2f, 
                           const float gyro_bias_max = 0.05f);

    void init_state(const float init_accl[3]);
    void update_gyro(const float gyro_meas[3], const float stamp, float dt);
    bool update_accl(const float accl_meas[3], const float stamp, float dt);

private:
    const float w_accl;
    const float w_gyro;
    const float gyro_bias_max;
    int average_cnt = 0;
    
    float gyro_bias[3];
    float init_accl[3];
    
    const float init_dt_gyro;
    const float init_dt_accl;

    
    uint32_t update_cnt;

public:
    Quaternion q;
    bool     init; 
    float rotation[3];
    float accl[3];
    float linear_accl[3];
};
