// Copyright (c) Direct Drive Technology Co., Ltd. All rights reserved.
// Author: Zhibin Wu <zhibin.wu@directdrivetech.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "sensors.h"

LOG_MODULE_REGISTER(sensors, CONFIG_LOG_DEFAULT_LEVEL);

#ifdef CONFIG_IST8310
#define DDT_MAG_METER
const struct device *const mag_dev = DEVICE_DT_GET_ONE(isentek_ist8310);
#endif
const struct device *const gyr_dev = DEVICE_DT_GET_ONE(bosch_bmi08x_gyro);
const struct device *const acc_dev = DEVICE_DT_GET_ONE(bosch_bmi08x_accel);


int sensors_init(struct robot_t *robot)
{
#ifdef DDT_MAG_METER
    if (!device_is_ready(mag_dev)) {
        LOG_ERR("Device %s is not ready\n", mag_dev->name);
        return -1;
    }
#endif

    if (!device_is_ready(gyr_dev)) {
        LOG_ERR("Device %s is not ready\n", gyr_dev->name);
        return -1;
    }

    if (!device_is_ready(acc_dev)) {
        LOG_ERR("Device %s is not ready\n", gyr_dev->name);
        return -1;
    }

    return 0;
}

int sensors_fresh(struct robot_t *robot)
{
    int ret = 0;
    struct sensor_value acc[3], gyr[3], mag[3], temp[1];

    // 0.5s dt
#ifdef DDT_MAG_METER
    ret = sensor_sample_fetch(mag_dev);

    if (ret == 0) {
        sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_XYZ, mag);

        robot->mag[0] = sensor_value_to_float(&mag[0]);
        robot->mag[1] = sensor_value_to_float(&mag[1]);
        robot->mag[2] = sensor_value_to_float(&mag[2]);
    }
#endif

    ret = sensor_sample_fetch(acc_dev);
    if (ret == 0) {
        sensor_channel_get(acc_dev, SENSOR_CHAN_ACCEL_XYZ, acc);

        robot->accel[0] = robot->imu_cfg.x_dir * sensor_value_to_float(&acc[robot->imu_cfg.x_map]) / (SENSOR_G / 1000000.0);
        robot->accel[1] = robot->imu_cfg.y_dir * sensor_value_to_float(&acc[robot->imu_cfg.y_map]) / (SENSOR_G / 1000000.0);
        robot->accel[2] = robot->imu_cfg.z_dir * sensor_value_to_float(&acc[robot->imu_cfg.z_map]) / (SENSOR_G / 1000000.0);

        if (robot->cycle_count % robot->frequency == 0) {
            sensor_channel_get(acc_dev, SENSOR_CHAN_DIE_TEMP, temp);

            robot->temperature = sensor_value_to_float(&temp[0]);
        }
    }

    ret = sensor_sample_fetch(gyr_dev);
    if (ret == 0) {
        sensor_channel_get(gyr_dev, SENSOR_CHAN_GYRO_XYZ, gyr);

        robot->gyro[0] = robot->imu_cfg.x_dir * sensor_value_to_float(&gyr[robot->imu_cfg.x_map]);
        robot->gyro[1] = robot->imu_cfg.y_dir * sensor_value_to_float(&gyr[robot->imu_cfg.y_map]);
        robot->gyro[2] = robot->imu_cfg.z_dir * sensor_value_to_float(&gyr[robot->imu_cfg.z_map]);
    }

    return 0;
}
