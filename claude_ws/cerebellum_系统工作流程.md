# Cerebellum 系统工作流程

## 目录

1. 系统概览
2. 完整数据流
3. 启动序列
4. 系统状态机（robot）
   - 4.1 状态定义
   - 4.2 ROBOT_SELFTEST_STATE — 自检
   - 4.3 ROBOT_READY_STATE — 就绪
   - 4.4 ROBOT_LOCOMOTION_STATE — 本机运动控制
   - 4.5 ROBOT_DIRECT_STATE — 直驱透传
   - 4.6 ROBOT_FAULT_STATE — 故障保护
   - 4.7 ROBOT_MOTOR_CALIBRATION_STATE — 电机标零
   - 4.8 ROBOT_MOTOR_OTA_STATE — 电机固件升级
5. 定时器与工作队列
   - 5.1 主循环定时器（robot_timer）
   - 5.2 双工作队列架构
   - 5.3 时间戳与抖动监测
6. 传感器数据采集
   - 6.1 IMU 初始化
   - 6.2 传感器数据刷新
   - 6.3 IMU 坐标轴映射
7. 姿态解算（AHRS）
   - 7.1 Madgwick 滤波器
   - 7.2 互补滤波器（legacy）
8. 电机驱动层
   - 8.1 wrapper 抽象层
   - 8.2 电机模型自动检测
   - 8.3 电机组合与硬件拓扑
   - 8.4 力矩-电流转换与 PD 前馈
   - 8.5 motor_stub 适配器
9. 通信协议栈
   - 9.1 CAN-FD 上行通道
   - 9.2 USB ACM 上行通道
   - 9.3 协议数据格式
   - 9.4 定点数压缩编码
   - 9.5 RPC 远程调用
   - 9.6 SBUS 遥控接收
10. 前后机协同
    - 10.1 GPIO 硬件身份识别
    - 10.2 motor_index_offset 地址偏移
    - 10.3 CAN ID 动态分配
11. 安全机制
    - 11.1 电机超时保护
    - 11.2 输入超时降级
    - 11.3 故障状态强制停止
    - 11.4 电机丢帧检测
12. 构建配置与产品矩阵
13. 接口汇总

---

## 1. 系统概览

Cerebellum 是运行在 Zephyr RTOS 上的单片机固件，部署于 Diablo 系列轮足机器人的运动控制板（Apollo/Athena 硬件平台）。其主要职责为：

- **电机驱动**：通过双路 CAN 总线驱动左右腿共 6~8 个关节电机，确保同侧三/四电机的指令在同一 CAN 帧内下发，时序严格一致
- **传感器采集**：读取板载 IMU（BMI088 加速度计+陀螺仪，可选 IST8310 磁力计），完成坐标轴映射和姿态解算
- **数据上行**：通过 CAN-FD 或 USB ACM 将电机反馈（位置/速度/力矩）和 IMU 数据（加速度/陀螺仪/四元数）实时传输给上层 SOC（运行 ROS 2）
- **指令下行**：接收来自上层 SOC 的运动控制指令（目标位置/速度/力矩/kp/kd），透传给电机执行
- **状态管理**：内置完整的状态机，管理自检→就绪→运动/直驱→故障的生命周期

本固件不包含运动控制算法（规划/控制闭环在 SOC 端 ROS 2 的 rl_controller 中完成），但在 A2 系列平台上集成了本机 legacy 运动控制库（WB6_Task），可在 LOCOMOTION 状态下本地闭环。

| 属性 | 说明 |
|------|------|
| RTOS | Zephyr RTOS |
| 主控芯片 | STM32（Apollo EVK / Apollo / Athena 硬件） |
| 自由度 | 6 DOF（A1 系列）/ 8 DOF（A2 系列） |
| 电机总线 | 双路 CAN 2.0（左腿 can1 / 右腿 can0），最高 1 Mbps |
| 上行总线 | CAN-FD（最高 8 Mbps）或 USB CDC ACM |
| 控制频率 | 10 Hz ~ 500 Hz，按状态动态切换 |
| 传感器 | BMI088 6-axis IMU + 可选 IST8310 3-axis 磁力计 |
| 姿态算法 | Madgwick AHRS（默认）/ 互补滤波器（legacy） |
| 编译系统 | CMake + Zephyr Build System |

---

## 2. 完整数据流

```
═══════════════════════════════════════════════════════════════════════════════
                              物理硬件
═══════════════════════════════════════════════════════════════════════════════

  BMI088 IMU (SPI)    左腿电机 x3/x4 (CAN1)   右腿电机 x3/x4 (CAN0)   电池/电源管理 (CAN)
       │                      │                       │                    │
       │ accel,gyro,temp      │ position,velocity,     │ position,velocity,  │ voltage,SOC
       │                      │ torque                 │ torque              │
       ▼                      ▼                       ▼                    ▼
═══════════════════════════════════════════════════════════════════════════════
                    Cerebellum 固件（Zephyr RTOS，MCU）
═══════════════════════════════════════════════════════════════════════════════

┌─────────────────────────────────────────────────────────────────────────────┐
│  sensors.c                    robot.c (主状态机 + 定时器驱动)                │
│  ├─ sensor_sample_fetch()     ├─ robot_timer_callback()                     │
│  ├─ 坐标轴映射 (imu_cfg)       ├─ robot_handler()  ← k_timer 硬件定时器       │
│  └─ 温度采样 (1Hz降频)         │   ├─ 计算 dt + 抖动监测                      │
│                               │   ├─ sensors_fresh() → IMU 原始数据          │
│  ahrs (姿态解算)               │   ├─ ahrs_update() → 四元数                  │
│  ├─ Madgwick AHRS (默认)       │   ├─ rc_update_input() / api_update_input()  │
│  └─ Complementary (legacy)     │   ├─ robot_update_estimator() (异步)         │
│                               │   ├─ smf_run_state() → 状态机推进             │
│                               │   └─ cycle_count++                           │
└──────────┬──────────────────────────────────────────────┬───────────────────┘
           │                                              │
           │ motor data in/out                            │ IMU + motor data
           ▼                                              ▼
┌──────────────────────────┐              ┌───────────────────────────────────┐
│ wrapper (电机驱动抽象)     │              │ api_canfd.c / api_acm.c (上行通道) │
│ ├─ left leg (CAN1)        │              │ ├─ api_swap_motor_data()          │
│ │   ├─ ABAD (P1010/Z1/…)  │              │ │   下行: motor_out → CAN → 电机   │
│ │   ├─ HIP   (P1010/Z1/…) │              │ │   上行: 电机反馈 → CAN → SOC     │
│ │   ├─ KNEE  (P1010/Z1/…) │              │ ├─ api_upload_imu_data()          │
│ │   └─ ANKLE (M1505/RS1/…)│              │ │   accel,gyro,quaternion,temp    │
│ ├─ right leg (CAN0)       │              │ ├─ api_upload_status_data()        │
│ │   └─ …                  │              │ │   运动状态/locomotion status     │
│ └──────────────────────────┘              │ └─ api_report_motor_status()       │
                                           │   电机错误码上报                   │
                                           └───────────┬───────────────────────┘
                                                       │
                                                       │ CAN-FD / USB ACM
                                                       ▼
═══════════════════════════════════════════════════════════════════════════════
                    上层 SOC（Jetson/域控，ROS 2）
═══════════════════════════════════════════════════════════════════════════════
                    DdtHardware → rl_controller (400 Hz)
```

数据流向说明：

- **上行（MCU → SOC）**：IMU 数据每控制周期通过 CAN-FD（CAN_ID_IMU1/IMU2）或 USB ACM 发送；电机反馈在 DIRECT 状态下每周期双向交换（swap），在 LOCOMOTION 状态下按需上传
- **下行（SOC → MCU）**：在 DIRECT 状态下，SOC 下发目标位置/速度/力矩/kp/kd（motor_out），MCU 解析后写入电机输出结构体，由 wrap_drive_fresh() 统一发送到 CAN 总线
- **电机总线（MCU → 电机）**：左右腿各一条独立 CAN 2.0 总线，每个控制周期发送一帧广播指令（CAN ID 0x32），帧内依次打包 3~4 个电机的电流指令

---

## 3. 启动序列

```
[Reset] main()
    │
    ├─ Step 1: hardware_init()
    │   ├─ gpio_ctrl_init()      → 读取 GPIO 引脚，识别前/后机身份
    │   └─ led_gpio_init()       → 初始化 LED 指示灯
    │
    ├─ Step 2: gpio_get_robot_model() → 打印机器人型号日志
    │
    ├─ Step 3: new_wrapper()     → 分配 wrapper_robot_t 结构体
    │
    ├─ Step 4: wrap_robot_init(wrapper) → 初始化运动控制库
    │   ├─ 创建 WB6_Base / WB6_Task / Attitude_Complementary
    │   ├─ 配置 IMU 坐标轴映射（按硬件平台区分）
    │   └─ wrap_set_motor_stub() → 创建 6 个 motor_stub 适配器
    │
    ├─ Step 5: wrap_drvie_setup(wrapper) → 初始化电机
    │   ├─ CAN1/CAN0 启动 → 注册 CAN RX 滤波器
    │   ├─ m15_motor_config() → 配置左右腿电机方向、零位偏置
    │   ├─ m15_motor_init() → 设置电机电流模式，使能
    │   ├─ p1010a_motor_init() → P1010 电机参数配置（电流环带宽/限流/超时等）
    │   └─ 注册各电机型号对应的 RX 回调（lk/p1010a/m1505a/z1_rs1/dm）
    │
    ├─ Step 6: robot_init(wrapper) → 初始化机器人运行时
    │   ├─ 创建工作队列: estimator_work_q (prio 10) + robot_work_q (prio -10)
    │   ├─ robot->ahrs = ahrs_init(DEFAULT_RATE) → 初始化姿态解算
    │   ├─ api_acm_init() / api_canfd_init() → 启动上行通信通道
    │   ├─ rc_sbus_init() → 启动 SBUS 遥控 DMA 接收
    │   ├─ sensors_init() → 检查 IMU 设备就绪
    │   ├─ k_timer_start(&robot_timer, ...) → 启动硬件定时器
    │   └─ robot_sm_init(robot, ROBOT_SELFTEST_STATE) → 进入自检状态
    │
    └─ Step 7: while(1) { led_gpio_toggle_indicator(); k_msleep(1000); }
        → 主线程退化为心跳 LED 闪烁，所有实时逻辑由定时器驱动
```

---

## 4. 系统状态机（robot）

状态机基于 Zephyr SMF（State Machine Framework）实现，共 11 个状态。

### 4.1 状态定义

| 状态 | 枚举值 | 说明 |
|------|--------|------|
| ROBOT_SELFTEST_STATE | 0 | 上电自检，等待电机上线 |
| ROBOT_READY_STATE | 1 | 自检通过，等待指令 |
| ROBOT_LOCOMOTION_STATE | 2 | 本机运动控制模式（legacy） |
| ROBOT_DIRECT_STATE | 3 | 直驱透传模式，SOC 直接控制电机 |
| ROBOT_FAULT_STATE | 4 | 故障保护，电机零力矩输出 |
| ROBOT_RECOVERY_STATE | 5 | 故障恢复（预留） |
| ROBOT_SLEEP_STATE | 6 | 深度休眠，电机断电（预留） |
| ROBOT_IMU_CALIBRATION_STATE | 7 | IMU 标定（预留） |
| ROBOT_MOTOR_CALIBRATION_STATE | 8 | 电机标零 |
| ROBOT_MOTOR_OTA_STATE | 9 | 电机固件 OTA 升级 |
| ROBOT_BOOT_RECOVERY_MODE_STATE | 10 | Boot 恢复模式（预留） |

状态跳转图：

```
                    ┌─────────┐
        ┌──────────→│  READY  │←──────────┐
        │           └────┬─────┘           │
        │                │                 │
   ready_next        ready_next       ready_next
   = FORCE_LOCO      = FORCE_DIRECT   = READY_WAITING
        │                │                 │
        ▼                ▼                 │
  ┌───────────┐   ┌───────────┐            │
  │ LOCOMOTION│   │  DIRECT   │────────────┘
  └─────┬─────┘   └─────┬─────┘
        │                │
   motor_fault       motor_fault
        │                │
        ▼                ▼
  ┌───────────┐
  │   FAULT   │
  └───────────┘
        ▲
        │
  ┌─────┴──────┐
  │  SELFTEST  │ (超时 → FAULT)
  └────────────┘
```

### 4.2 ROBOT_SELFTEST_STATE — 自检

- **频率**：SELFTEST_RATE = 100 Hz
- **超时**：cycle_count > 1000（即 10 秒）→ 跳转 FAULT
- **电机模型检测**：循环查询 wrap_get_motor_model()，直到检测到具体电机型号（非 MODEL_UNKNOWN）
- **电机上线检查**：cycle_count > 100（即 1 秒）后，wrap_drive_check() == 0 即视为电机全部在线，跳转 READY
- **过渡期间**：每周期发送零力矩指令 wrap_motor_stub_zero()，防止电机异常输出

### 4.3 ROBOT_READY_STATE — 就绪

- **频率**：READY_RATE = 10 Hz
- **CAN-FD 速率重配**：若 canfd_8mbps_mode 置位，在进入 READY 时调用 canfd_reconfigure_rate_real()，将 CAN-FD 数据段速率从 5 Mbps 切换为 8 Mbps
- **前后机检测**：每周期调用 gpio_ctrl_update() 读取引脚，动态设置 motor_index_offset（0 = 前机，8 = 后机）
- **状态分支**（按 ready_next 值）：

| ready_next | 跳转目标 |
|------------|---------|
| rc_connected / FORCE_LOCOMOTION / AUTO_LOCOMOTION | ROBOT_LOCOMOTION_STATE |
| FORCE_DIRECT | ROBOT_DIRECT_STATE |
| MOTOR_ZERO_CAL | ROBOT_MOTOR_CALIBRATION_STATE |
| BOOT_RECOVERY_MODE | ROBOT_BOOT_RECOVERY_MODE_STATE |

### 4.4 ROBOT_LOCOMOTION_STATE — 本机运动控制

- **频率**：LOCOMOTION_RATE = 500 Hz（CAN 2.0 带宽受限机型降为 500 Hz）
- **LED**：LED_LOCOMOTION
- **每周期执行**：
  1. `wrap_drive_check()` — 检查电机状态，异常则跳转 FAULT
  2. **输入超时检测**：如果 input.timestamp 超过 1 秒未更新，强制输入归零、stand_mode=1（站立）
  3. `api_upload_status_data()` — 每 (frequency/100) 周期上传一次运动状态（10 Hz 降频）
  4. `wrap_update_battery_info()` — 更新电池电压
  5. `wrap_update_input()` — 将 input 写入运动控制库
  6. `wrap_update_imu_data()` — 将 IMU 数据写入运动控制库（含互补滤波姿态更新）
  7. `wrap_update_motor_stub()` — 将电机反馈刷新到 motor_stub
  8. `wrap_ctrl_loop(dt)` — 执行一次运动控制闭环（plan_update + ctrl_update）
  9. `wrap_motor_stub_fresh()` — 将运动控制输出从 motor_stub 写入电机 out
  10. `wrap_drive_fresh()` — 通过 CAN 总线向电机发送力矩/电流指令

### 4.5 ROBOT_DIRECT_STATE — 直驱透传

- **频率**：DIRECT_RATE = 400 Hz
- **LED**：LED_LOCOMOTION
- **每周期执行**：
  1. `wrap_drive_check()` — 电机状态检查
  2. `api_swap_motor_data()` — **双向数据交换**：
     - 下行：从 api_state.out[]（SOC 下发）→ motor.out[]（写入电机输出结构体）
     - 上行：从 motor.in[]（电机反馈）→ api_state.in[]（打包 CAN 帧发送给 SOC）
  3. `wrap_drive_fresh()` — 向电机总线发送指令
  4. `api_upload_imu_data()` — 上传 IMU 数据
- 此状态下**不执行本机运动控制**，电机 kp/kd/position/velocity/torque 全部由 SOC 端 rl_controller 决定

### 4.6 ROBOT_FAULT_STATE — 故障保护

- **频率**：FAULT_RATE = 5 Hz
- **LED**：LED_FAULT
- **每周期执行**：
  1. `wrap_motor_stub_zero()` — 强制清零所有电机输出
  2. `wrap_drive_fresh()` — 发送零力矩指令到电机
  3. `robot->api_ops->report_motor_status()` — 上报电机错误码

### 4.7 ROBOT_MOTOR_CALIBRATION_STATE — 电机标零

- **频率**：MOTOR_CAL_RATE = 1 Hz
- 通过 motor_zero 标志位（bit0=左腿，bit1=右腿）触发对应侧电机的零位校准
- 标零完成后清除对应标志位

### 4.8 ROBOT_MOTOR_OTA_STATE — 电机固件升级

- **频率**：1 Hz
- 调用 wrap_drive_ota_upgrade()，按 OTA 状态机逐步完成电机固件传输
- 升级失败 → 跳转 FAULT

---

## 5. 定时器与工作队列

### 5.1 主循环定时器（robot_timer）

```
K_TIMER_DEFINE(robot_timer, robot_timer_callback, NULL);
```

- 类型：Zephyr k_timer，硬件定时器驱动
- 触发周期：由 robot_set_frequency() 动态设置，K_USEC(1000000 / frequency)
- 回调 `robot_timer_callback()`：将 robot_work 投递到 robot_work_q

### 5.2 双工作队列架构

```
                    ┌────────────────────┐
                    │   robot_timer      │
                    │ (硬件定时器回调)    │
                    └────────┬───────────┘
                             │ k_work_submit_to_queue()
                             ▼
              ┌──────────────────────────────┐
              │  robot_work_q                │
              │  stack: 4096 bytes           │
              │  priority: -10 (cooperative) │
              │  no_yield: true              │
              │                              │
              │  robot_handler():            │
              │    ├─ dt/jitter 计算          │
              │    ├─ 传感器刷新 + AHRS       │
              │    ├─ 输入更新                │
              │    ├─ 异步 estimator 投递     │
              │    ├─ 安全检查                │
              │    └─ smf_run_state()         │
              └──────────────────────────────┘

              ┌──────────────────────────────┐
              │  estimator_work_q            │
              │  stack: 4096 bytes           │
              │  priority: 10                │
              │  no_yield: true              │
              │                              │
              │  estimator_handler():        │
              │    └─ wrap_update_estimator() │
              │       (LQR K矩阵更新等)       │
              └──────────────────────────────┘
```

- robot_work_q 优先级 `-10`（cooperative 级别），保证控制循环不被普通线程打断
- estimator_work_q 优先级 `10`，异步执行状态估计更新（如 LQR 增益矩阵重算），不阻塞主循环
- estimator 按降频比触发：`frequency / estimator_freq` (默认 30 Hz)，例如 500 Hz 主循环时每 ~17 个周期触发一次

### 5.3 时间戳与抖动监测

robot_handler() 在每个控制周期开始时：

1. 读取 `sys_clock_cycle_get_32()` 获取硬件周期计数
2. 处理 32 位计数器溢出（overflow_count 递增）
3. 计算 `dt = interval / CYCLES_PER_SEC`（实际时间间隔）
4. 计算 `jitter = (interval - 理论间隔) / 理论间隔 * 100%`
5. 若 `jitter > 10%` → LOG_ERR 告警
6. 合成 64 位微秒级时间戳：`timestamp = (overflow_count << 32 | now) / (CYCLES_PER_SEC / 1e6)`

---

## 6. 传感器数据采集

### 6.1 IMU 初始化

```c
const struct device *const gyr_dev = DEVICE_DT_GET_ONE(bosch_bmi08x_gyro);
const struct device *const acc_dev = DEVICE_DT_GET_ONE(bosch_bmi08x_accel);
// 可选: const struct device *const mag_dev = DEVICE_DT_GET_ONE(isentek_ist8310);
```

- 加速度计：Bosch BMI08x Accelerometer（SPI）
- 陀螺仪：Bosch BMI08x Gyroscope（SPI）
- 磁力计（可选）：iSentek IST8310（I2C），通过 `CONFIG_IST8310` 编译开关控制

### 6.2 传感器数据刷新

sensors_fresh() 每控制周期调用一次：

```
(1) sensor_sample_fetch(acc_dev)
   └─ sensor_channel_get(acc_dev, SENSOR_CHAN_ACCEL_XYZ, acc)
      → robot->accel[0..2] = imu_cfg.{x,y,z}_dir * raw / (SENSOR_G / 1e6)
      → 按 imu_cfg.{x,y,z}_map 索引重映射坐标轴

(2) sensor_sample_fetch(gyr_dev)
   └─ sensor_channel_get(gyr_dev, SENSOR_CHAN_GYRO_XYZ, gyr)
      → robot->gyro[0..2] = imu_cfg.{x,y,z}_dir * raw

(3) (可选) sensor_sample_fetch(mag_dev)
   └─ sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_XYZ, mag)
      → robot->mag[0..2]

(4) 温度降频采样 (每 frequency 个周期 1 次)
   └─ sensor_channel_get(acc_dev, SENSOR_CHAN_DIE_TEMP, temp)
```

温度采样以 1 Hz 等效频率降频读取（`cycle_count % frequency == 0`），减少 SPI 总线开销。

### 6.3 IMU 坐标轴映射

不同硬件平台的 IMU 芯片物理安装方向不同，通过 `imu_config_t` 结构体实现轴映射：

```c
typedef struct imu_config_t {
    uint8_t x_map;   // 物理轴 → 逻辑 X 的映射 (0=X,1=Y,2=Z)
    uint8_t y_map;
    uint8_t z_map;
    int8_t  x_dir;   // 方向: +1 正向, -1 反向
    int8_t  y_dir;
    int8_t  z_dir;
} imu_config_t;
```

以 Apollo EVK 平台为例：X 逻辑轴映射物理 Y 轴（正向），Y 逻辑轴映射物理 X 轴（反向），Z 逻辑轴映射物理 Z 轴（正向）。

---

## 7. 姿态解算（AHRS）

### 7.1 Madgwick 滤波器（默认）

使用 [Fusion AHRS](https://github.com/xioTechnologies/Fusion) 库（Madgwick 算法）：

```
ahrs_init(rate) → 创建 FusionAhrs 实例，设置采样频率
ahrs_update(ahrs, accel, gyro, mag, dt) → FusionAhrsUpdate()
ahrs_get_quaternion(ahrs, q) → FusionAhrsGetQuaternion()
ahrs_reset(ahrs, rate) → 状态切换时重置滤波器
```

- 输入：加速度计 (m/s²)、陀螺仪 (rad/s)、可选磁力计 (μT)
- 输出：四元数 [w, x, y, z]
- 核心收敛增益 beta 由库内部根据采样率自动调节

### 7.2 互补滤波器（legacy）

通过 `CONFIG_DDT_LEGACY_ATTITUDE_ALG` 编译开关启用：

```
Attitude_Complementary::update_accl(accel, stamp, dt)
Attitude_Complementary::update_gyro(gyro, stamp, dt)
Attitude_Complementary::update_heading()
```

- 加速度计低频分量 + 陀螺仪高频积分 → 互补融合
- 在 LOCOMOTION 状态下，IMU 数据额外经过 Attitude_Complementary 处理，供 WB6_Task 运动控制库使用

---

## 8. 电机驱动层

### 8.1 wrapper 抽象层

wrapper 是本固件的核心抽象，通过函数指针表将具体电机的 CAN 协议差异与上层 robot 状态机解耦。两类 wrapper 实现：

| 平台 | wrapper 实现 | 电机型号 |
|------|-------------|---------|
| A1 系列 (Athena) | diablo_wrapper.cpp | M1502/M1505，仅 torque 模式 |
| A2 系列 (Apollo EVK) | apollo_evk_wrapper.cpp | P1010 / MG10015 / Z1 + M1505 / RS1 / DM8006 组合 |

wrapper 核心接口：

| 函数 | 功能 |
|------|------|
| `wrap_robot_init()` | 初始化运动控制库、IMU 配置 |
| `wrap_drvie_setup()` | 启动 CAN 总线、注册 RX 滤波器、配置电机参数、上使能 |
| `wrap_drive_fresh()` | 每周期将 motor.out[] 编码为 CAN 帧发送给电机 |
| `wrap_drive_check()` | 汇总所有电机错误状态 |
| `wrap_get_motor_model()` | 自动检测当前连接的电机组合型号 |
| `wrap_drvie_set_zero()` | 触发电机机械零位标定 |
| `wrap_drive_reconfigure()` | 切换控制频率时通知电机调整回传速率 |

### 8.2 电机模型自动检测

在自检和就绪状态中，wrap_get_motor_model() 通过识别 CAN 总线上各电机回传帧的特征 CAN ID 确定电机型号：

| 电机型号 | 回传 CAN ID 范围 | 识别特征 |
|---------|-----------------|---------|
| P1010 | 0x050~0x05F | 15-bit 编码器，力矩常数 1.85~3.0 Nm/A（带减速器） |
| MG10015 (LK) | 0x140~0x143 | 14-bit 编码器，10:1 减速比，力矩常数 1.4 Nm/A |
| M1505A | 0x090~0x09F | 15-bit 编码器，力矩常数 0.63 Nm/A |
| Z1 | 扩展 ID (CMD=2) | 16-bit 定点数编码，力矩常数范围更大 |
| RS1 | 扩展 ID (CMD=2, motor_id=0x7F) | 与 Z1 共用协议，通过 motor_id 区分 |
| DM8006 | 标准 ID=0, data[0]&0x0F=1 | 达妙电机，12-bit 定点编码 |

检测逻辑：遍历左右腿 6 个关节电机，取逻辑与确认同类型电机的一致性，再根据踝关节电机型号确定最终 MODEL 枚举值。

### 8.3 电机组合与硬件拓扑

每条腿由 3~4 个电机组成（按 DOF 配置）：

```
  6 DOF (DIABLO_SIX_DOF):
  LEFT Leg (CAN1)          RIGHT Leg (CAN0)
  ├─ M_HIP  (ABAD+HIP合并) ├─ M_HIP
  ├─ M_KNEE                ├─ M_KNEE
  └─ M_ANKLE (轮子)         └─ M_ANKLE

  8 DOF (DIABLO_EIGHT_DOF):
  LEFT Leg (CAN1)          RIGHT Leg (CAN0)
  ├─ M_ABAD (外展/内收)     ├─ M_ABAD
  ├─ M_HIP  (髋)           ├─ M_HIP
  ├─ M_KNEE (膝)           ├─ M_KNEE
  └─ M_ANKLE (踝/轮)        └─ M_ANKLE
```

支持的电机组合：

| MODEL 枚举 | 髋/膝/ABAD 电机 | 踝/轮电机 | DoF |
|-----------|---------------|----------|-----|
| MODEL_P1010_M1505 | P1010 x 3 | M1505 x 1 | 8 |
| MODEL_P1010_RS1 | P1010 x 3 | RS1 x 1 | 8 |
| MODEL_P1010_DM8006 | P1010 x 3 | DM8006 x 1 | 8 |
| MODEL_P1010_P1010 | P1010 x 4 | P1010 x 1 | 8 |
| MODEL_MG10015_M1505 | MG10015 (LK) x 3 | M1505 x 1 | 8 |
| MODEL_Z1_M1505 | Z1 x 3 | M1505 x 1 | 8 |
| MODEL_Z1_RS1 | Z1 x 3 | RS1 x 1 | 8 |
| MODEL_M1502_M1502 | M1502 x 2 | M1502 x 1 | 6 |

### 8.4 力矩-电流转换与 PD 前馈

电机输出指令（motor.out）包含目标位置、速度、力矩以及 kp/kd 增益。在发送到电机前，根据电机类型将目标力矩转换为电流指令：

**P1010A**：
```
torque = out.torque + out.kp * (out.position - in.position) + out.kd * (out.velocity - in.velocity)
torque = clamp(torque, -90, 90)  // 限幅 +-90 Nm
current = (torque / 2.5) * 100   // 力矩常数 2.5 Nm/A -> 电流标度
```

**M1505A**：
```
torque = out.torque + out.kp * position_error + out.kd * velocity_error
torque = clamp(torque, -16, 16)  // 限幅 +-16 Nm
current = (torque / 0.63) / 55.0 * 32767  // -> int16
```

**MG10015 (LK)**：
```
current = (out.torque / 1.4) * (2000 / 32.0)  // 10:1 减速器输出
```

**Z1/RS1 (扩展 CAN ID 协议)**：
```
// 位置/速度/力矩/kp/kd 分别以 16-bit uint 编码到 CAN 帧
p_out = float_to_uint(position, p_min, p_max, 16)
v_out = float_to_uint(velocity, v_min, v_max, 16)
t_out = float_to_uint(torque,   t_min, t_max, 16)
kp_out = float_to_uint(kp, 0, kp_max, 16)
kd_out = float_to_uint(kd, 0, kd_max, 16)
// CAN ID 编码: (cmd << 24) | (t_out << 8) | motor_id
```

**DM8006**：
```
// 位置 16-bit, 速度 12-bit, kp 12-bit, kd 12-bit, 力矩 12-bit
// 紧凑打包到一个 8 字节 CAN 帧中
```

### 8.5 motor_stub 适配器

motor_stub 类继承自 Joint_motor，作为运动控制库（WB6_Task）与硬件电机之间的桥梁：

```
                    WB6_Task (运动控制)
                         │
                    Joint_motor 接口
                         │
              ┌──────────┴──────────┐
              │     motor_stub      │  <- C++ 类
              │  .update(pos,vel,   │     从 motor.in 读取反馈
              │         torque)     │
              │  .refresh(*torque)  │     向 motor.out 写入指令
              └──────────┬──────────┘
                         │
              ┌──────────┴──────────┐
              │  struct motor_t     │  <- C 结构体
              │  .in  (motor_in_t)  │     电机反馈数据
              │  .out (motor_out_t) │     电机指令数据
              └─────────────────────┘
```

- A1 平台：6 个 motor_stub（左 hip/knee/wheel + 右 hip/knee/wheel）
- A2 平台：不使用 motor_stub，而是通过 Middleware_Class 的 downbridge 直接与运动控制库交互

---

## 9. 通信协议栈

### 9.1 CAN-FD 上行通道

CAN-FD 接口作为 MCU 与 SOC 之间的主通信通道（A2 系列默认），通过编译开关 `CONFIG_DDT_ROBOT_API_CANFD` 启用。

**CAN-FD 初始化**：
- 仲裁段：1 Mbps
- 数据段：默认 5 Mbps，可在 READY 状态通过 canfd_reconfigure_rate_real() 升级到 8 Mbps
- 模式：CAN_MODE_FD（支持 64 字节数据段，BRS 比特率切换）
- RX 滤波器：
  - motor_out_filter (0x120, mask 0x7F8) -> motor_out_rx_callback（接收 SOC 下发的电机指令）
  - motor_out_fixed_filter (0x128, mask 0x7FC) -> motor_out_rx_callback（定点数格式电机指令）
  - any_filter (0x000, mask 0x000) -> any_rx_callback（接收 RPC、传感器配置、PMS 电池、CAN-FD 速率切换等）

**CAN ID 分配**：

| CAN ID | 宏定义 | 方向 | 内容 |
|--------|--------|------|------|
| 0x102 | CAN_ID_STATE | MCU -> SOC | 电机状态（错误码） |
| 0x103 | CAN_ID_LOCOMOTION_STATE | MCU -> SOC | 运动状态（模式/跳跃等） |
| 0x108~0x10B | CAN_ID_MOTOR_IN | MCU -> SOC | 电机反馈数据（4 帧 x 2 电机，float） |
| 0x10C~0x10D | CAN_ID_MOTOR_FIXED_IN | MCU -> SOC | 电机反馈数据（定点数编码） |
| 0x110~0x117 | CAN_ID_MOTOR_OUT | SOC -> MCU | 电机指令数据（8 帧 x 1 电机，float） |
| 0x118 | CAN_ID_IMU1 | MCU -> SOC | IMU 数据（前机） |
| 0x119 | CAN_ID_IMU2 | MCU -> SOC | IMU 数据（后机） |
| 0x120~0x12B | CAN_ID_MOTOR_SWAP | SOC -> MCU | 直驱模式电机指令（float）/ 定点格式 |

**any_rx_callback 多路分发**：

```
any_rx_callback(frame)
  ├─ (frame->id & 0x780) == 0x080 -> pms_rx_callback()
  │   ├─ 0x81 -> 左电池电压
  │   ├─ 0x82 -> 右电池电压
  │   ├─ 0x83 -> 超级电容电压
  │   └─ 0x9F -> canfd_reconfigure_rate()
  ├─ 0x170 -> rpc_rx_callback()
  │   ├─ SET_STAND_MODE -> robot->input.stand_mode
  │   ├─ SET_READY_NEXT -> robot->ready_next
  │   ├─ SET_JUMP -> robot->input.jump_state
  │   └─ SET_MOTOR_ZERO -> robot->motor_zero
  ├─ 0x12D -> sensor_rx_callback()
  │   -> 解析 api_channel_input_t (SOC 下发的控制指令)
  ├─ 0x381 -> canfd_reconfigure_rate()
  └─ 0x385 -> canfd_reconfigure_motor_offset()
      -> 设置 motor_index_offset (前后机切换)
```

### 9.2 USB ACM 上行通道

通过 `CONFIG_DDT_ROBOT_API_ACM` 编译开关启用（A1 系列默认），使用 Zephyr USB CDC ACM 驱动。

**数据封装**：
```
+--------+--------+----------+------+------------------+
|  tag   | length | checksum | type |   CAN payload    |
| 0xAA55 | 2bytes | CRC16    | 0x01 | (id+dlc+flags+data) |
| 2bytes |        |          |      |                  |
+--------+--------+----------+------+------------------+
```

- tag: 帧同步头 0xAA55
- checksum: CRC16-ANSI 校验
- type: ACM_TYPE_CAM = 0x01 (CAN over ACM)

**发送流程**：
1. 将 CAN 帧（id/dlc/flags/data）封装为 acm_can_pkt_t
2. 外层再包裹 acm_pkt_t（tag/length/checksum/type）
3. 写入 Ring Buffer -> 使能 UART TX IRQ -> 硬件 FIFO 发送

**中断驱动的收发**：
- RX: `uart_irq_rx_ready()` -> 从 FIFO 读到 Ring Buffer
- TX: `uart_irq_tx_ready()` -> 从 Ring Buffer 写到 FIFO，Ring Buffer 空则关 TX IRQ

### 9.3 协议数据格式

所有跨总线数据结构使用 `__attribute__((__packed__))` 紧凑对齐，确保跨平台兼容。

**IMU 数据帧（44 bytes）**：
```c
struct api_imu_data_t {
    uint32_t timestamp;       // us 时间戳
    float accel[3];           // m/s^2
    float gyro[3];            // rad/s
    float quaternion[4];      // w,x,y,z
    float temperature;        // degC
};
```

**电机指令帧（24 bytes）**：
```c
struct api_motor_out_t {
    uint32_t timestamp;
    float position;   // rad
    float kp;
    float velocity;   // rad/s
    float kd;
    float torque;     // Nm
};
```

**电机反馈帧（16 bytes）**：
```c
struct api_motor_in_t {
    uint32_t timestamp;
    float position;   // rad
    float velocity;   // rad/s
    float torque;     // Nm
};
```

**通道输入帧（32 bytes）**：
```c
struct api_channel_input_t {
    uint32_t timestamp;
    float forward, yaw, pitch, roll, height, split, tilt;
    float forward_accel, yaw_accel;
};
```

### 9.4 定点数压缩编码

通过 `CONFIG_DDT_CANFD_USE_FIXED_POINT` 编译开关启用定点数传输，减少 CAN 总线带宽占用。

**量化算法** (`calculate_optimal_q`)：
```
给定 min, max 范围，找到最大 Q 值使 min*2^Q >= -32768 且 max*2^Q <= 32767
max_q = 15（默认上限）
```

**编码** (`float_to_int16`)：
```
q = calculate_optimal_q(min, max, 15)
scaled = value * (1 << q)
return clamp(round(scaled), -32768, 32767)
```

**定点数电机指令帧（14 bytes，vs float 24 bytes，节省 42%）**：
```c
struct api_motor_out_16_t {
    uint32_t timestamp;
    int16_t position;   // Q 值动态计算
    int16_t kp;
    int16_t velocity;
    int16_t kd;
    int16_t torque;
};
```

### 9.5 RPC 远程调用

SOC 通过 CAN ID 0x170 下发 RPC 请求（`api_rpc_request_t`），MCU 在 any_rx_callback 中即时处理：

| RPC Key | 枚举值 | 功能 |
|---------|--------|------|
| SET_STAND_MODE | 0x222 | 设置站立模式（idle/craw/stand/transform_up/transform_down） |
| SET_READY_NEXT | 0x200 | 触发状态切换（FORCE_DIRECT/FORCE_LOCOMOTION/MOTOR_ZERO_CAL等） |
| SET_JUMP | 0x231 | 触发跳跃 |
| SET_MOTOR_ZERO | 0x280 | 触发电机标零 |
| GET_MODEL_INFO | 0x100 | 查询机器人型号（预留） |
| GET_SERIAL_NUMBER | 0x101 | 查询序列号（预留） |

### 9.6 SBUS 遥控接收

rc_sbus.c 实现 CRSF/SBUS 协议遥控信号的 DMA 异步接收：

- **串口配置**：100 kbps / 8E2（SBUS 标准格式）
- **帧格式**：25 字节（start=0x0F, 22 data bytes, flags, end=0x00）
- **DMA 缓冲区**：3 块 ping-pong 缓冲（SBUS_DMA_BUF_CNT=3），每块 32 字节
- **UART 事件驱动**：
  - `UART_RX_RDY` -> 检查帧头尾对齐，未对齐则重启动 DMA
  - `UART_RX_BUF_REQUEST` -> 切换 DMA 缓冲区
  - `UART_RX_DISABLED` -> 自动重启（最多重试 10 次）

**摇杆解码**：

| SBUS 通道 | 映射输出 | 解码方式 |
|-----------|---------|---------|
| CH1 | yaw | (ch - median) / median / 0.8, 死区 +-1% |
| CH2 | forward (反向) | 同上 |
| CH3 | pitch | 同上 |
| CH4 | roll | 同上 |
| CH8 | height | (ch - 200) / 1600, clamp(0, 1) |

**遥控器中位自动标定**：上电后采集 480 帧（约 3.4 秒 @ 7ms 帧间隔）摇杆中位值做滑动平均，标定完成前不输出控制量。

---

## 10. 前后机协同

Diablo 机器人支持两台机器人通过 Dock 连接器物理拼接，形成前后机（Front/Rear）协同工作模式。

### 10.1 GPIO 硬件身份识别

```
gpio_ctrl_init():
  读取 GPIO pin_d13 -> 高电平 = ROBOT_MODEL_REAR (后机)
                      低电平 = ROBOT_MODEL_FRONT (前机)
  读取 GPIO pin_b14 -> 高电平 = DVT3 标记 (硬件版本识别)
```

### 10.2 motor_index_offset 地址偏移

前后机在 CAN-FD 总线上使用不同的 CAN ID 偏移避免冲突：

| 机器人身份 | motor_index_offset | CAN ID 基础偏移 |
|-----------|-------------------|----------------|
| 前机 (FRONT) | 0 | 0x108/0x110/0x128 |
| 后机 (REAR) | 8 | 0x10C/0x114/0x12A |

在 CAN-FD RX 回调中，通过检查 `robot->motor_index_offset` 将收到的 CAN 帧路由到正确的 api_state 槽位。

### 10.3 CAN ID 动态分配

SOC 可通过 CAN ID 0x385 下发 `canfd_reconfigure_motor_offset` 指令动态切换前后机身份：
- `data[0:1] = 0xFFFF` -> motor_index_offset = 8（后机）
- `data[0:1] = 0x0000` -> motor_index_offset = 0（前机）

---

## 11. 安全机制

### 11.1 电机超时保护

在 LOCOMOTION 和 DIRECT 状态的每个周期，robot_handler() 检查 wrap_drive_check() 返回值：
- 非零 -> 立即跳转 ROBOT_FAULT_STATE
- FAULT 状态下，所有电机输出强制清零（`wrap_motor_stub_zero()`），确保物理安全

### 11.2 输入超时降级

在 LOCOMOTION 状态中：
```
if (input.timestamp 未更新超过 1 秒)
    -> forward = yaw = pitch = roll = 0
    -> stand_mode = 1 (强制站立)
```

确保在 SOC 断连或通信中断时机器人自动进入安全站姿。

### 11.3 故障状态强制停止

robot_handler() 中设有全局故障检查（独立于状态机执行）：
```
if ((wrap_drive_check() != 0 || ready_next == ESTOP_MODE) && 当前不在 FAULT 状态)
    -> 强制跳转 ROBOT_FAULT_STATE
```

### 11.4 电机丢帧检测

在 wrap_drive_fresh() 中，每周期检查 tx_count - rx_count：
- 差值 > 500 -> 该电机对应的 motor_status bit 置位
- 任意电机 motor_status 非零 -> 全部电机输出清零（`wrap_motor_stub_zero()`），本周期不发送控制指令

额外检查：
- Z1 电机 status 字段 != 0x80 -> 故障标记
- M1505 电机 status 字段 != 0x00 -> 故障标记（包含超时/过流/过温等）

---

## 12. 构建配置与产品矩阵

通过 Kconfig 实现产品矩阵管理：

```
DIABLO_ROBOT (目标选择)
├─ DIABLO_A2_SERIES_LOCOMOTION  -> Apollo EVK 硬件, CAN-FD 上行, 8 DOF, 含本机运动控制
├─ DIABLO_A1_SERIES_LOCOMOTION  -> Athena 硬件, USB ACM 上行, 6 DOF, 含本机运动控制
└─ DIABLO_STUB_LOCOMOTION       -> 最小配置, 8 DOF, 无本机运动控制

DIABLO_DOF (自由度)
├─ DIABLO_EIGHT_DOF  -> STUB_MOTOR_NUM=4, 4关节/腿
└─ DIABLO_SIX_DOF    -> STUB_MOTOR_NUM=3, 3关节/腿

关键编译开关:
  CONFIG_DDT_ROBOT_API_CANFD       -> 启用 CAN-FD 上行通道
  CONFIG_DDT_ROBOT_API_ACM         -> 启用 USB ACM 上行通道
  CONFIG_DDT_CANFD_USE_FIXED_POINT -> 启用定点数压缩
  CONFIG_DDT_LEGACY_ATTITUDE_ALG   -> 使用 legacy 互补滤波器
  CONFIG_APP_ABUP_FOTA             -> 启用 ABUP OTA 远程升级
```

**源文件编译矩阵**：

| 文件 | A1 (Athena) | A2 (Apollo EVK) | Stub |
|------|:-----------:|:---------------:|:----:|
| main.c, robot.c, sensors.c, led_gpio.c, gpio_ctrl.c, rc_sbus.c | Y | Y | Y |
| api_canfd.c | | Y | |
| api_acm.c | Y | | |
| diablo_wrapper.cpp + motor_stub.cpp + motor_athena.c | Y | | |
| apollo_evk_wrapper.cpp + motor_apollo_evk.c | | Y | Y |
| lib/madgwick/* | Y (默认) | Y (默认) | Y (默认) |
| lib/attitude_comp/* | Y (legacy) | | |

---

## 13. 接口汇总

### 13.1 传感器硬件接口

| 接口 | 设备 | 总线 |
|------|------|------|
| acc_dev | bosch_bmi08x_accel | SPI |
| gyr_dev | bosch_bmi08x_gyro | SPI |
| mag_dev | isentek_ist8310（可选） | I2C |
| can1 | 左腿电机总线 | CAN 2.0, 1 Mbps |
| can0 | 右腿电机总线 | CAN 2.0, 1 Mbps |
| canfd | SOC 上行通道 | CAN-FD, 1+5/8 Mbps |
| usbacm | SOC 上行通道（A1） | USB FS CDC ACM |
| uart_dev (sbus0) | 遥控接收机 | UART 100kbps 8E2 |

### 13.2 对外 CAN-FD 帧定义

| CAN ID | 方向 | 数据长度 | 内容 |
|--------|------|---------|------|
| 0x102 | TX | 14B | 电机状态（错误码） |
| 0x103 | TX | <=64B | 运动状态（模式/状态机/调试） |
| 0x108~0x10B | TX | 64B | 电机反馈 x8（float） |
| 0x10C~0x10D | TX | 64B | 电机反馈 x8（定点） |
| 0x110~0x117 | RX | 24B | 电机指令 x8（float, SOC->MCU） |
| 0x118/0x119 | TX | 44B | IMU 数据（前/后机） |
| 0x120~0x12B | RX | 28/14B | 直驱电机指令（float/定点） |
| 0x12D | RX | 32B | 通道控制输入（SOC->MCU） |
| 0x170 | RX | 可变 | RPC 远程调用 |
| 0x081~0x083 | RX | 8B | 电池电压（PMS） |
| 0x381/0x9F | RX | 8B | CAN-FD 速率切换触发 |
| 0x385 | RX | 8B | 前后机身份切换 |

### 13.3 控制频率速查

| 状态 | 频率 (Hz) | 周期 (ms) |
|------|----------|----------|
| ROBOT_SELFTEST_STATE | 100 | 10 |
| ROBOT_READY_STATE | 10 | 100 |
| ROBOT_LOCOMOTION_STATE | 500* | 2 |
| ROBOT_DIRECT_STATE | 400 | 2.5 |
| ROBOT_FAULT_STATE | 5 | 200 |
| ROBOT_MOTOR_CALIBRATION_STATE | 1 | 1000 |
| ROBOT_MOTOR_OTA_STATE | 1 | 1000 |

*CAN 2.0 带宽受限机型（Z1+M1505 / Z1+RS1）限制为 500 Hz

### 13.4 电机力矩常数速查

| 电机型号 | 力矩常数 | 峰值力矩 | 编码器分辨率 |
|---------|---------|---------|------------|
| P1010A | 1.85~3.0 Nm/A | +-90 Nm | 15-bit (0~32767) |
| M1505A | 0.63 Nm/A | +-16 Nm | 15-bit (0~32767) |
| M1502 | 0.8 Nm/A | -- | 15-bit |
| MG10015 (LK) | 1.4 Nm/A (10:1减速) | -- | 14-bit (0~16383) |
| Z1 | -- | +-120 Nm | 16-bit 定点 |
| RS1 | -- | +-12 Nm | 16-bit 定点 |
| DM8006 | -- | +-20 Nm | 12-bit 定点 |

### 13.5 QoS 与可靠性设计

| 机制 | 实现 |
|------|------|
| CAN TX 丢帧处理 | EAGAIN/ENETUNREACH 静默丢弃，不阻塞控制循环 (K_NO_WAIT) |
| 电机通信超时 | 电机端本地超时保护（M15 100ms / P1010 1000ms），超时自动失能 |
| 控制抖动监测 | 10% 阈值 LOG_ERR 告警 |
| CAN 总线错误计数 | bus_tx_error / bus_rx_error，超过 100 触发 FAULT |
| 电机收发计数重置 | 每 5 秒通过 wrap_drive_reset_count() 清零计数器 |
| SBUS 帧对齐恢复 | 3 块 DMA ping-pong 缓冲 + 自动重启，最多重试 10 次 |
