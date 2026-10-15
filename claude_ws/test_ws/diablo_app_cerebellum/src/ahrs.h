#ifndef AHRS_H
#define AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void ahrs_reset(void *ahrs, uint32_t rate);

void *ahrs_init(uint32_t rate);
void ahrs_release(void *ahrs);

int ahrs_update(void *ahrs, const float accel[3], const float gray[3], const float mag[3], const float dt);

int ahrs_get_quaternion(void *ahrs, float quaternion[4]);
int ahrs_get_euler(void *ahrs, float euler[3]);

#ifdef __cplusplus
}
#endif

#endif
