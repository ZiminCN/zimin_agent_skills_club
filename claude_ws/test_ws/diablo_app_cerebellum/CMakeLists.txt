# Copyright (c) Direct Drive Technology Co., Ltd. All rights reserved.
# Author: Zhibin Wu <zhibin.wu@directdrivetech.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../)

if(CONFIG_APP_ABUP_FOTA)
    list(APPEND ZEPHYR_EXTRA_MODULES
        ${CMAKE_CURRENT_LIST_DIR}/../../../modules/lib/abup)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cerebellum)

include_directories(src)

if(CONFIG_APP_ABUP_FOTA)
    zephyr_library_include_directories(${CMAKE_CURRENT_LIST_DIR}/../../../modules/lib/abup/inc)
endif()

set(COMMON_SOURCES
    src/led_gpio.c
    src/gpio_ctrl.c
    src/sensors.c
    src/main.c
    src/rc_sbus.c
    src/robot.c
)

if(CONFIG_DDT_ROBOT_API_CANFD)
    set(COMMON_SOURCES ${COMMON_SOURCES} src/api_canfd.c)
endif()

if(CONFIG_DDT_ROBOT_API_ACM)
    set(COMMON_SOURCES ${COMMON_SOURCES} src/api_acm.c)
endif()

if(CONFIG_DDT_LEGACY_ATTITUDE_ALG)
    set(AHRS_SOURCES
        lib/attitude_comp/ahrs_attitude_comp.cpp
        lib/attitude_comp/Attitude_Comp.cpp
    )
    include_directories(lib/attitude_comp)
    # legacy attitude dependent on diablo math library
    zephyr_library_include_directories(${CMAKE_CURRENT_LIST_DIR}/../../../modules/lib/sim-webots/diablo_A1/controllers/diablo_webots/inc/utility/Math)
else()
    set(AHRS_SOURCES
        lib/madgwick/ahrs_madgwick.c
        lib/madgwick/FusionAhrs.c
        lib/madgwick/FusionCompass.c
        lib/madgwick/ahrs_madgwick.c
        lib/madgwick/FusionOffset.c
    )
    include_directories(lib/madgwick)
endif()

set(ATHENA_SOURCES src/motor_stub.cpp src/diablo_wrapper.cpp src/motor_athena.c)
set(APOLLO_SOURCES src/apollo_evk_wrapper.cpp src/motor_apollo_evk.c)

target_sources_ifdef(CONFIG_DIABLO_A1_SERIES_LOCOMOTION app PRIVATE ${ATHENA_SOURCES})
target_sources_ifdef(CONFIG_DIABLO_A2_SERIES_LOCOMOTION app PRIVATE ${APOLLO_SOURCES})
target_sources_ifdef(CONFIG_DIABLO_STUB_LOCOMOTION app PRIVATE ${APOLLO_SOURCES})

target_compile_definitions(app PRIVATE _GNU_SOURCE=1 DIABLO_ZEPHYR=1)
target_sources(app PRIVATE ${COMMON_SOURCES} ${AHRS_SOURCES})
