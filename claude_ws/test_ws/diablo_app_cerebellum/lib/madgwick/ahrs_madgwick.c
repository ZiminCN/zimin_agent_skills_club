
#include "ahrs.h"

#include "Fusion.h"

struct ahrs_fusion_t {
    /** gyroscope */
    FusionMatrix gyroscopeMisalignment;
    FusionVector gyroscopeSensitivity;
    FusionVector gyroscopeOffset;
    /** accelerometer */
    FusionMatrix accelerometerMisalignment;
    FusionVector accelerometerSensitivity;
    FusionVector accelerometerOffset;
    /** magnetometer */
    FusionMatrix softIronMatrix;
    FusionVector hardIronOffset;
    /** initialise algorithms */
    FusionOffset offset;
    FusionAhrs ahrs;
    /** settings */
    FusionAhrsSettings settings;
    /** sample rate */
    uint32_t rate;
    /** ahrs output */
    FusionQuaternion quaternion;
};

void ahrs_reset(void *ahrs, uint32_t rate)
{
    struct ahrs_fusion_t *fusion = (struct ahrs_fusion_t *)ahrs;

    fusion->gyroscopeMisalignment = FUSION_IDENTITY_MATRIX;
    fusion->gyroscopeSensitivity = FUSION_VECTOR_ONES;
    fusion->gyroscopeOffset = FUSION_VECTOR_ZERO;

    fusion->accelerometerMisalignment = FUSION_IDENTITY_MATRIX;
    fusion->accelerometerSensitivity = FUSION_VECTOR_ONES;
    fusion->accelerometerOffset = FUSION_VECTOR_ZERO;

    fusion->softIronMatrix = FUSION_IDENTITY_MATRIX;
    fusion->hardIronOffset = FUSION_VECTOR_ZERO;

    fusion->rate = rate;

    FusionOffsetInitialise(&fusion->offset, fusion->rate);
    FusionAhrsInitialise(&fusion->ahrs);

    fusion->settings.convention = FusionConventionNwu;
    fusion->settings.gain = 0.5f;
    fusion->settings.accelerationRejection = 10.0f;
    fusion->settings.magneticRejection = 20.0f;
    fusion->settings.rejectionTimeout = 5 * fusion->rate;

    FusionAhrsSetSettings(&fusion->ahrs, &fusion->settings);
}

void *ahrs_init(uint32_t rate)
{
    struct ahrs_fusion_t *fusion = (struct ahrs_fusion_t *)malloc(sizeof(struct ahrs_fusion_t));
    if (fusion == 0) {
        return (void *)0;
    }

    ahrs_reset((void *)fusion, rate);

    return (void *)fusion;
}

void ahrs_release(void *ahrs)
{
    if (ahrs != 0)
        free(ahrs);
}

int ahrs_update(void *ahrs, const float accel[3], const float gray[3], const float mag[3], const float dt)
{
    struct ahrs_fusion_t *fusion = (struct ahrs_fusion_t *)ahrs;

    FusionVector gyroscope = {
        FusionRadiansToDegrees(gray[0]), FusionRadiansToDegrees(gray[1]),
        FusionRadiansToDegrees(gray[2])};  // degrees/s
    FusionVector accelerometer = {accel[0], accel[1], accel[2]};  // g
    FusionVector magnetometer = FUSION_VECTOR_ZERO;

    // apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, fusion->gyroscopeMisalignment, fusion->gyroscopeSensitivity, fusion->gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, fusion->accelerometerMisalignment, fusion->accelerometerSensitivity, fusion->accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(magnetometer, fusion->softIronMatrix, fusion->hardIronOffset);

    // update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&fusion->offset, gyroscope);

    // update gyroscope ahrs algorithm
    FusionAhrsUpdate(&fusion->ahrs, gyroscope, accelerometer, magnetometer, dt);

    // algorithm outputs
    fusion->quaternion =  FusionAhrsGetQuaternion(&fusion->ahrs);

    return 0;
}

int ahrs_get_quaternion(void *ahrs, float quaternion[4])
{
    struct ahrs_fusion_t *fusion = (struct ahrs_fusion_t *)ahrs;

    quaternion[0] = fusion->quaternion.element.x;
    quaternion[1] = fusion->quaternion.element.y;
    quaternion[2] = fusion->quaternion.element.z;
    quaternion[3] = fusion->quaternion.element.w;

    return 0;
}

int ahrs_get_euler(void *ahrs, float euler[3])
{
    struct ahrs_fusion_t *fusion = (struct ahrs_fusion_t *)ahrs;
    const FusionEuler e = FusionQuaternionToEuler(fusion->quaternion);

    euler[0] = e.angle.roll;
    euler[1] = e.angle.pitch;
    euler[2] = e.angle.yaw;

    return 0;
}
