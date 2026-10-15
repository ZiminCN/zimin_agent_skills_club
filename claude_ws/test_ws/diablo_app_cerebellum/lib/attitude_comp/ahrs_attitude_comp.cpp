#include "ahrs.h"

#include "Attitude_Comp.hpp"

void ahrs_reset(void *ahrs, uint32_t rate)
{
}

void *ahrs_init(uint32_t rate)
{
    Attitude_Complementary *atti = new Attitude_Complementary();

    return (void *)atti;
}

void ahrs_release(void *ahrs)
{
}

int ahrs_update(void *ahrs, const float accel[3], const float gray[3], const float mag[3], float dt)
{
    Attitude_Complementary *atti = (Attitude_Complementary *)ahrs;

    atti->update_accl(accel, 0, dt);
    atti->update_gyro(gray, 0, dt);
    atti->q.normalize();
    return true;
}

int ahrs_get_quaternion(void *ahrs, float quaternion[4])
{

    Attitude_Complementary *atti = (Attitude_Complementary *)ahrs;

    quaternion[0] = atti->q.x; // x
    quaternion[1] = atti->q.y; // y
    quaternion[2] = atti->q.z; // z
    quaternion[3] = atti->q.w; // w

    return 0;
}

int ahrs_get_euler(void *ahrs, float euler[3])
{
    return 0;
}
