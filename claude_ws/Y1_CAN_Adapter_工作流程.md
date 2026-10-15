

## 目录

1. [系统概览](#1-系统概览)
2. [硬件拓扑与引脚映射](#2-硬件拓扑与引脚映射)
3. [完整数据流](#3-完整数据流)
4. [CAN FD 软件热插拔机制](#4-can-fd-软件热插拔机制)
   - 4.1 设计背景与问题
   - 4.2 双重心跳检测体系
   - 4.3 连接状态机
   - 4.4 断连检测与确认
   - 4.5 重连恢复
   - 4.6 发送保护与防"耳聋"
5. [主循环与时隙调度](#5-主循环与时隙调度)
6. [心跳与连接管理](#6-心跳与连接管理)
   - 6.1 真实心跳
   - 6.2 虚假心跳（数据心跳）
   - 6.3 心跳超时与发送失败恢复
7. [CAN FD 消息转发引擎](#7-can-fd-消息转发引擎)
   - 7.1 双端口架构
   - 7.2 ID 映射与转发
   - 7.3 消息分类与优先级
   - 7.4 中断驱动接收
   - 7.5 速率限制
8. [Docker 锁定/解锁控制](#8-docker-锁定解锁控制)
   - 8.1 PWM 伺服控制
   - 8.2 命令路径与执行
   - 8.3 H 桥驱动器管理
9. [BOX 模式](#9-box-模式)
   - 9.1 硬件版本检测
   - 9.2 透传通道
   - 9.3 箱体电机控制
   - 9.4 箱体传感器与限位
10. [OTA 升级机制](#10-ota-升级机制)
11. [设备信息管理](#11-设备信息管理)
12. [安全与恢复机制](#12-安全与恢复机制)
    - 12.1 心跳超时恢复
    - 12.2 发送失败恢复
    - 12.3 CAN 总线离线恢复
    - 12.4 强制断连恢复
13. [状态机汇总](#13-状态机汇总)
14. [时序汇总](#14-时序汇总)

---

## 1. 系统概览

Y1 CAN FD 适配器是一款基于 STM32G473RCT6（Cortex-M4, 160 MHz）的嵌入式固件，作为机器人底盘中两块控制板之间的 CAN FD 桥接设备运行。固件以 bare-metal 方式实现，不依赖任何 RTOS，通过硬件定时器驱动的主循环时隙调度所有任务。

| 模块 | 更新频率 | 硬件资源 | 职责 |
|------|----------|----------|------|
| CAN FD 转发引擎 | 中断驱动 | FDCAN2 (PB12/PB13), FDCAN3 (PB3/PB4) | 双端口 CAN FD 消息收发与 ID 映射转发, 5/8 Mbps 数据段速率 |
| 热插拔检测 | 5 ms / 1 s | 软件状态机 | 检测 A2A 总线物理连接/断开，管理连接生命周期 |
| 心跳管理 | 50 ms | FDCAN3 TX, 软件计数器 | 双重心跳(真实+虚假)发送与超时检测 |
| Docker 控制 | 1 s (命令)/实时(PWM) | TIM1 CH2 (PA9), TIM8 CH1 (PA15) | 舵机 PWM 锁定/解锁，H 桥使能 |
| BOX 模式 | 5 ms / 50 ms | GPIO 多路 | 箱体电机控制与限位传感器检测 |
| OTA 升级 | 事件驱动 | 闪存最后 2 KB | 引导加载程序跳转、设备信息更新、升级确认 |
| 系统监控 | 1 s / 10 s | 软件计数器 | 总线离线恢复、发送失败统计、看门狗式软重启 |

系统启动时通过读取 MCU 引脚电平（PA3）自动确定本机角色（Master/Slave），通过读取闪存中的设备信息确定 CAN FD 波特率（5 Mbps 或 8 Mbps）以及是否为 BOX 版本。

---

## 2. 硬件拓扑与引脚映射

### 2.1 CAN FD 总线拓扑

```
  ┌──────────────┐         ┌──────────────────┐         ┌──────────────┐
  │  机器人控制板  │◄─FDCAN2─►│  Y1 CAN FD 适配器  │◄─FDCAN3─►│  Y1 CAN FD 适配器  │
  │   (Robot)    │   R2A    │     (本机)       │   A2A    │     (对端)       │
  └──────────────┘         └──────────────────┘         └──────────────────┘
                                    │
                              FDCAN1 (PA12/PB8)
                              未使用，进入掉电模式
```

- **FDCAN2（端口 2）**：适配器与机器人控制板之间的通信（R2A 链路）
  - TX: PB13, RX: PB12, AF9
  - 使用 RX FIFO0 + 中断线 0 + TX Buffer 0/1
- **FDCAN3（端口 3）**：适配器与对端适配器之间的通信（A2A 链路）
  - TX: PB4, RX: PB3, AF11
  - 使用 RX FIFO1 + 中断线 1 + TX Buffer 2
- **FDCAN1（端口 1）**：未使用，初始化后立即进入掉电模式

### 2.2 CAN FD 位时序

| 参数 | 标称段 (Arbitration) | 数据段 5 Mbps | 数据段 8 Mbps |
|------|---------------------|---------------|---------------|
| 预分频器 | 16 | 4 | 4 |
| Seg1 | 7 | 5 | 3 |
| Seg2 | 2 | 2 | 1 |
| SJW | 1 | 1 | 1 |
| 速率 | 1 Mbps | 5 Mbps | 8 Mbps |

8 Mbps 模式下额外启用 TX 延迟补偿（SSP=19, FSS=15）。

### 2.3 功能引脚映射

| 引脚 | 功能 | 方向 | 说明 |
|------|------|------|------|
| PA2 | HDRIVER | 输出 | H 桥驱动器使能（Docker 供电控制） |
| PA3 | CANFD_ID_OFFSET | 输入(上拉) | Master(高)/Slave(低) 角色选择 |
| PA4 | ADC2_IN17 | 模拟输入 | 硬件版本检测（分压电阻） |
| PA9 | TIM1_CH2 | AF2 | Docker 舵机 PWM 通道 2 |
| PA15 | TIM8_CH1 | AF2 | Docker 舵机 PWM 通道 1 |
| PB6 | HEARTBEAT_LED | 输出(上拉) | 心跳状态指示灯 |
| PB7 | SYS_LED | 输出(上拉) | 系统运行指示灯 |
| PA8 | UP_SIGNAL | 输入(上拉) | BOX 模式：顶部限位开关 |
| PC8 | LEFT_SIGNAL | 输入 | BOX 模式：左侧传感器 |
| PC9 | RIGHT_SIGNAL | 输入 | BOX 模式：右侧传感器 |
| PA6/PB1 | BOX_MOTOR | 输出 | BOX 模式：电机 PH/EN 控制 |
| PA7/PB0 | BOX_MOTOR | 输出 | BOX 模式：电机 PH/EN 控制 |

### 2.4 定时器与 DMA 配置

| 定时器 | 通道 | 预分频 | 周期 | 频率 | DMA | 用途 |
|--------|------|--------|------|------|-----|------|
| TIM1 | CH2 | 159 | 19999 | 50 Hz | DMA1 CH2 | Docker 舵机 PWM |
| TIM8 | CH1 | 159 | 19999 | 50 Hz | DMA1 CH1 | Docker 舵机 PWM |
| TIM3 | CH2/CH3 | 159 | 49 | 20 kHz | DMA1 CH3/CH4 | 通用 PWM（预留） |
| TIM5 | CH3 | 159 | 999 | 1 kHz | DMA1 CH5 | 通用 PWM（预留） |

---

## 3. 完整数据流

```
                              ═══════════════════════════════════════
                                          物理层
                              ═══════════════════════════════════════

  ┌─────────────────────────┐                    ┌─────────────────────────┐
  │     机器人控制板          │                    │     机器人控制板          │
  │    (CAN FD 1 Mbps)      │                    │    (CAN FD 1 Mbps)      │
  └───────────┬─────────────┘                    └───────────┬─────────────┘
              │                                              │
        FDCAN2 (R2A)                                    FDCAN2 (R2A)
        PB12/PB13                                       PB12/PB13
              │                                              ▲
              ▼                                              │
  ═══════════════════════════════════════    ═══════════════════════════════════════
         Y1 适配器 (Master)                          Y1 适配器 (Slave)
  ═══════════════════════════════════════    ═══════════════════════════════════════
              │                                              │
   ┌──────────┴──────────┐                      ┌──────────┴──────────┐
   │                     │                      │                     │
   │  FIFO0 中断回调      │                      │  FIFO0 中断回调      │
   │  (R2A → A2A 转发)   │                      │  (R2A ← A2A 转发)   │
   │                     │                      │                     │
   │  ID 映射:           │                      │  ID 映射:           │
   │  R2A_ID - offset    │                      │  A2A_ID + offset    │
   │  = A2A_ID           │                      │  = R2A_ID           │
   │                     │                      │                     │
   └──────────┬──────────┘                      └──────────┬──────────┘
              │                                              │
        FDCAN3 (A2A)                                    FDCAN3 (A2A)
        PB3/PB4                                        PB3/PB4
        @ 1/5/8 Mbps                                  @ 1/5/8 Mbps
              │                                              │
              ▼                                              ▼
  ════════════════════════════════════════════════════════════════
                    CAN FD 总线 (A2A, 双绞线直连)
        承载: 心跳、电机指令、电池信息、Docker命令、用户数据、OTA数据
  ════════════════════════════════════════════════════════════════


  消息分类与转发路径:

  ┌──────────────────────────────────────────────────────────────────┐
  │  消息类别            R2A 方向              A2A 方向      优先级   │
  ├──────────────────────────────────────────────────────────────────┤
  │  根信息              Robot → 适配器转发    对端 → Robot    普通    │
  │  电机主→从           Robot → 适配器转发    对端 → Robot    高      │
  │  电机从→主           Robot → 适配器转发    对端 → Robot    高      │
  │  电机主→从(固定)     Robot → 适配器转发    对端 → Robot    高      │
  │  电机从→主(固定)     Robot → 适配器转发    对端 → Robot    高      │
  │  电池信息             Robot → 适配器转发    对端 → Robot    普通    │
  │  电机状态             Robot → 适配器转发    对端 → Robot    普通    │
  │  电源控制             Robot → 适配器转发    对端 → Robot    普通    │
  │  Docker 锁定          Robot → 本机执行+转发  对端 → Robot    普通    │
  │  Docker 指令          Robot → 适配器转发    对端 → Robot    普通    │
  │  用户数据             Robot → 适配器转发    对端 → Robot    普通    │
  │  心跳                 ---                  本机 → 对端      普通    │
  │  广播(偏移/组合状态)  本机 → Robot          ---             普通    │
  │  OTA/系统命令         Robot → 本机/BOX      ---             普通    │
  │  BOX 数据            Robot ↔ 适配器透传   适配器 ↔ 对端透传  高     │
  └──────────────────────────────────────────────────────────────────┘
```

---

## 4. CAN FD 软件热插拔机制

### 4.1 设计背景与问题

Y1 适配器使用 CAN FD 总线在两个适配器之间进行板间通信（A2A 链路）。与带有物理热插拔检测引脚的连接器（如 USB、HDMI）不同，CAN FD 的 A2A 连接是两根差分信号线直接对接，**硬件层面没有提供连接/断开的电气检测机制**。

这意味着固件必须以纯软件方式判断 A2A 总线是否处于连接状态，来替代硬件热插拔检测。需要解决的核心问题包括：

1. **连接建立检测**：如何确定对端适配器已经接入？
2. **连接断开检测**：如何判断对端适配器已经拔出？
3. **假连接防护**：如何避免因偶发噪声或残留总线数据导致的"假连接"判断？
4. **发送保护**：在确认断开后，如何防止本机继续向已不存在的对端发送数据导致的 CAN 仲裁失败和总线阻塞？

### 4.2 双重心跳检测体系

固件设计了**双重心跳**机制来解决上述问题：

#### 真实心跳 (Real Heartbeat)

- **载体**：专用的 A2A 心跳 CAN 消息，每 50 ms 由 `heartbeat_pongtong()` 定时发送
- **方向**：Master 发送固定心跳 ID，Slave 发送心跳 ID+1
- **载荷**：16 字节，携带本机全部心跳状态（是否使能、是否收到对端心跳、是否收到伪心跳、主从角色、完全分离标志、各计数器值）
- **建立条件**：需连续收到 10 个（`CONTINUE_HEARTBEAT_CNT`）匹配的对端心跳消息，才将 `is_received_heartbeat` 置为 true
- **超时条件**：连续 10 个心跳周期（50 ms × 10 = 500 ms）未收到匹配心跳，则将 `is_received_heartbeat` 置为 false
- **计数器保护**：`continue_heartbeat_cnt` 上限为 10×CONTINUE_HEARTBEAT_CNT，到达后回退到 5×CONTINUE_HEARTBEAT_CNT，防止无限增长

#### 虚假心跳 / 数据心跳 (Fake Heartbeat / Data Heartbeat)

- **载体**：端口 3（A2A）上收到的**任何非心跳 CAN 数据消息**都可视为虚假心跳
- **检测位置**：在 `HAL_FDCAN_RxFifo1Callback()` 中，任何进入 FIFO1 的消息（先于过滤器匹配判断之前）都会增加 `continue_fake_heartbeat_cnt`
- **建立条件**：需连续收到 100 个（`CONTINUE_FAKE_HEARTBEAT_CNT`）任意 A2A 消息
- **超时条件**：连续 5 个心跳周期（50 ms × 5 = 250 ms）未收到任何 A2A 消息，则将 `is_received_data_fake_heartbeat` 置为 false
- **计数器保护**：`continue_fake_heartbeat_cnt` 上限为 5×CONTINUE_FAKE_HEARTBEAT_CNT

#### 双重心跳的设计意图

引入虚假心跳的核心原因是解决 CAN 总线仲裁问题。在标准 CAN/CAN FD 中，**ID 越小的帧仲裁优先级越高**。真实心跳的 ID 虽然固定，但如果某一时刻总线负载较高，仅依赖真实心跳可能出现以下情况：

- 心跳消息因仲裁失败被延迟，导致误判断连
- 对端正在发送大量数据，说明总线物理连接确定存在——此时不应判为断连

因此虚假心跳提供了一条**正交的检测路径**：只要 A2A 链路上有数据流动（无论是什么消息），就说明物理连接存在。虚假心跳的建立阈值（100 个）远高于真实心跳（10 个），但超时阈值（5 个周期）更短，形成了"容易建立、快速超时"的特性，使其成为一个灵敏的"连接存在"探测器。

两条心跳路径的关系是**逻辑或**：只要真实心跳或虚假心跳任一判定为连接，固件即认为 A2A 链路处于连接状态。

### 4.3 连接状态机

适配器的 A2A 连接管理是一个由多个标志位和计数器协作实现的分布式状态机（而非单一状态变量），主要状态标志包括：

| 标志 | 类型 | 含义 |
|------|------|------|
| `is_received_heartbeat` | bool | 真实心跳已建立（需 10 个连续匹配） |
| `is_received_data_fake_heartbeat` | bool | 虚假心跳已建立（需 100 个任意消息） |
| `is_complete_separation` | bool | 已确认完全分离（断连确认后置 true） |
| `force_a2a_disconnected_flag` | bool | 强制 A2A 断开模式（阻止端口 3 发送） |
| `force_a2a_disconnected_cooldown_flag` | bool | 断连冷却检测窗口已打开 |
| `is_just_triggered_separated_flag` | bool | 刚触发了分离流程，等待恢复 |

连接状态的判定逻辑：

```
连接正常 = (is_received_heartbeat == true) OR (is_received_data_fake_heartbeat == true)
完全断连 = (is_received_heartbeat == false) AND (is_received_data_fake_heartbeat == false)
```

### 4.4 断连检测与确认

断连检测分为**两级**，分别在不同的时隙中运行：

#### 第一级：心跳超时检测（50 ms 时隙）

在 `heartbeat_pongtong()` 中执行：

```
每个 50 ms 周期:
  if 真实心跳已建立:
    timeout_cnt += 1
    if timeout_cnt >= 10:
      is_received_heartbeat = false       // 标记真实心跳丢失
      timeout_cnt = 0
      continue_heartbeat_cnt = 0          // 重置建立计数器

  if 虚假心跳已建立:
    fake_heartbeat_timeout_cnt += 1
    if fake_heartbeat_timeout_cnt >= 5:
      is_received_data_fake_heartbeat = false  // 标记虚假心跳丢失
      fake_heartbeat_timeout_cnt = 0
      continue_fake_heartbeat_cnt = 0
```

#### 第二级：断连确认与强制断开（5 ms 时隙 + 1 s 时隙协同）

当 `force_a2a_disconnected_cooldown_flag == true` 且两个心跳都为 false 时，进入断连确认流程：

```
5 ms 时隙中（断连确认逻辑）:
  if force_a2a_disconnected_cooldown_flag == true:
    if (!is_received_heartbeat) AND (!is_received_data_fake_heartbeat):
      if is_just_triggered_separated_flag == false:
        // 需要连续 DISCONNECTED_REPEAT_LIMIT(2) 个广播周期确认
        if disconnected_repeat_boardcast_cnt >= 2:
          // 确认断连，进入强制断开模式
          set_heartbeat_led_status(HIGH)        // 点亮心跳灯指示断连
          is_just_triggered_separated_flag = true
          force_a2a_disconnected_flag = true    // ★ 设置强制断开标志
          force_a2a_disconnected_recovery_cnt = 0
          force_a2a_disconnected_cooldown_flag = false
          adapter_heartbeat.is_complete_separation = true  // ★ 标记完全分离
        disconnected_repeat_boardcast_cnt += 1

        // ★ 断连期间广播清零：偏移状态、组合状态全部清零
        boardcast_r2a_heartbeat_data.offset_status = 0x0000
        boardcast_r2a_heartbeat_data.combind_status[0] = 0x0000
        boardcast_r2a_heartbeat_data.combind_status[1] = 0x0000
        // 立即通过端口 2 通知机器人当前已断连
        canfd_send_data(hfdcan2, 广播ID, ...)

  else:  // force_a2a_disconnected_cooldown_flag == false
    // 完全分离状态存在且未锁定 → 重新锁定 Docker
    if (docker_lock_status != DOCKER_LOCK) AND (is_complete_separation == true):
      docker_lock()
```

关键设计考量：
- **二级确认机制**：心跳丢失后，不会立即判定为断连，而是需要连续 2 个广播周期确认（通过 `disconnected_repeat_boardcast_cnt` 计数）
- **冷却窗口**：`force_a2a_disconnected_cooldown_flag` 作为检测窗口的使能，仅在 1 s 时隙中当心跳存在时被置为 true——这保证了"建立连接 → 窗口打开 → 检测断连"的时序约束，避免启动时的误判
- **广播清零**：断连确认期间，向机器人发送的广播数据中偏移状态和组合状态全部清零，通知机器人 A2A 链路已断开

### 4.5 重连恢复

重连检测在 5 ms 时隙中持续进行：

```
5 ms 时隙中（重连检测逻辑）:
  if (is_received_heartbeat) OR (is_received_data_fake_heartbeat):
    // 任一条件满足 → 正在重连
    is_just_triggered_separated_flag = false
    disconnected_repeat_boardcast_cnt = 0   // 重置断连计数

    if force_a2a_disconnected_flag == true:
      force_a2a_disconnected_flag = false   // ★ 解除强制断开
      force_a2a_disconnected_recovery_cnt = 0
      adapter_heartbeat.is_complete_separation = false  // 清除完全分离标志

1 s 时隙中（强制断开恢复计数）:
  if force_a2a_disconnected_flag == true:
    if force_a2a_disconnected_recovery_cnt > 3:  // FORCE_A2A_DISCONNECTED_RECOVERY_LIMIT_CNT
      force_a2a_disconnected_flag = false  // 超时自动解除（故障保护）
      force_a2a_disconnected_recovery_cnt = 0

    // 强制断开期间确保 Docker 为解锁状态（安全考虑）
    if docker_lock_status != DOCKER_UNLOCK:
      docker_unlock()

    force_a2a_disconnected_recovery_cnt += 1
```

1 s 时隙中（重连后的正常广播）：
```
if (is_received_heartbeat) OR (is_received_data_fake_heartbeat):
  // 恢复广播数据
  boardcast_r2a_heartbeat_data.offset_status = GPIO 读取值
  boardcast_r2a_heartbeat_data.combind_status[0] = 0xFFFF
  boardcast_r2a_heartbeat_data.combind_status[1] = 0xFFFF
  boardcast_r2a_heartbeat_data.docker_status = docker_lock_status
  canfd_send_data(hfdcan2, 广播ID, ...)  // 通知机器人已重连
```

### 4.6 发送保护与防"耳聋"

在强制断开期间，`canfd_send_data()` 函数中设有发送拦截逻辑：

```c
bool canfd_send_data(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, ...) {
    // ★ 关键保护：强制断开时阻止向端口 3 (A2A) 发送任何数据
    if ((hfdcan == &hfdcan3) && (force_a2a_disconnected_flag)) {
        return true;  // 静默丢弃，返回成功
    }
    // ... 正常发送逻辑 ...
}
```

这是热插拔机制中最关键的工程实践之一。当 A2A 物理链路断开时，如果固件继续向 FDCAN3 的 TX FIFO 写入消息，FDCAN 控制器会持续尝试发送——由于总线无应答，每次发送都会经历完整的仲裁失败→重试→最终超时流程。这会导致两个严重后果：

1. **TX FIFO 阻塞**：后续合法的（重连后的）消息无法及时发送
2. **中断风暴**：每次发送失败都可能触发错误中断

因此在确认断连后，**釜底抽薪地阻止一切 A2A 发送**，直到心跳检测到重连后自动恢复。这个过程对上层消息转发逻辑完全透明——转发函数无需关心连接状态，`canfd_send_data()` 在底层统一处理。

---

## 5. 主循环与时隙调度

固件的主循环没有任何 RTOS 调度，完全基于 `HAL_GetTick()` 驱动的时隙分派：

```
主循环 (while(1)):
  currentTick = HAL_GetTick()

  ┌─ 1 ms 时隙 ─────────────────────────────────────
  │  (预留，当前无任务)
  │
  ├─ 5 ms 时隙 ─────────────────────────────────────
  │  • 重置用户数据速率限制器 (user_id_400~404_limit_flag)
  │  • BOX 模式：读取三个限位开关 GPIO
  │  • BOX 模式：执行底部箱体传感器检查 (check_bottom_box_sensor)
  │  • ★ 热插拔断连确认逻辑 (force_a2a_disconnected_cooldown_flag 判断)
  │  • ★ 热插拔重连检测逻辑 (心跳恢复时清除分离标志)
  │  • 完全分离但非强制断开时：自动重锁 Docker
  │
  ├─ 50 ms 时隙 ────────────────────────────────────
  │  • ★ heartbeat_pongtong() — 心跳超时检测 + 心跳消息发送
  │  • 设置电池/电机状态速率限制器
  │  • BOX 模式：更新箱体电机状态机 (update_box_motor_status)
  │
  ├─ 100 ms 时隙 ───────────────────────────────────
  │  (预留，当前无任务)
  │
  ├─ 1 s 时隙 ──────────────────────────────────────
  │  • 切换系统运行 LED (toggle_sys_led_status)
  │  • ★ 根据心跳状态设置心跳 LED 和 H 桥驱动器
  │  • ★ 首次心跳建立 → 自动锁定 Docker + 打开冷却检测窗口
  │  • ★ 向机器人发送广播消息 (偏移/组合/Docker 状态)
  │  • 执行 Docker 锁定/解锁命令
  │  • OTA 升级签名检查 (check_ota_upgrade_sign)
  │  • ★ heartbeat_timeout_recovery() — 心跳超时恢复
  │  • ★ 强制断开恢复计数器管理 (force_a2a_disconnected_recovery_cnt)
  │
  └─ 10 s 时隙 ─────────────────────────────────────
     • canfd_bus_off_recovery() — CAN 总线离线恢复检查
```

---

## 6. 心跳与连接管理

### 6.1 真实心跳

**发送端**（每 50 ms 在 `heartbeat_pongtong()` 中执行）：

```
心跳数据包 (16 字节 CAN FD 帧):
  [0]    is_enable              — 心跳功能使能
  [1]    is_received_heartbeat  — 本机是否收到对端真实心跳
  [2]    is_received_data_fake_heartbeat — 本机是否收到对端伪心跳
  [3]    is_master_dev          — 本机是否是 Master
  [4]    is_slave_dev           — 本机是否是 Slave
  [5]    is_complete_separation — 本机判定是否完全断连
  [6]    continue_heartbeat_cnt — 真实心跳连续计数
  [7]    timeout_cnt            — 真实心跳超时计数
  [8-9]  continue_fake_heartbeat_cnt — 伪心跳连续计数 (大端序)
  [10]   fake_heartbeat_timeout_cnt  — 伪心跳超时计数
  [11]   send_offset_boardcast_failed_cnt — 广播发送失败计数
  [12]   send_heartbeat_failed_cnt       — 心跳发送失败计数
  [13-14] send_fake_heartbeat_failed_cnt — 伪心跳发送失败计数 (大端序)
  [15]   0x00                   — 保留
```

通过在心跳消息中携带本机的全部心跳状态，对端适配器可以了解链路的双向状态，便于调试和故障定位。

**接收端**：在 `HAL_FDCAN_RxFifo1Callback()` 中触发 `A2A_HEARTBEAT_process()`

```
收到对端心跳:
  if 本机是 Master:
    只接受 Slave 的心跳
  if 本机是 Slave:
    只接受 Master 的心跳

  if continue_heartbeat_cnt >= 10:
    is_received_heartbeat = true
    timeout_cnt = 0

  continue_heartbeat_cnt += 1
  if continue_heartbeat_cnt >= 100:  // 10×阈值，防溢出
    continue_heartbeat_cnt = 50      // 回退到 5×阈值
```

### 6.2 虚假心跳（数据心跳）

**接收端**：在 `HAL_FDCAN_RxFifo1Callback()` 中，在任何过滤器匹配处理之前执行：

```
收到任何 A2A 消息 (先于过滤器匹配):
  if continue_fake_heartbeat_cnt >= 100:
    is_received_data_fake_heartbeat = true
    fake_heartbeat_timeout_cnt = 0

  continue_fake_heartbeat_cnt += 1
  if continue_fake_heartbeat_cnt >= 500:  // 5×阈值，防溢出
    continue_fake_heartbeat_cnt = 200     // 回退到 2×阈值
```

### 6.3 心跳超时与发送失败恢复

在 `heartbeat_pongtong()` 中还监控**发送失败计数器**，处理"能收不能发"的故障场景：

```
发送失败监控 (50 ms):
  if send_offset_boardcast_failed_cnt >= 2:
    // 广播发送失败2次 → 预留处理位置

  if send_heartbeat_failed_cnt >= 30:
    // 心跳消息发送失败30次 → 软重启 (非BOX版本)

  if send_fake_heartbeat_failed_cnt >= 6000:
    // 伪心跳发送失败6000次 → 软重启 (非BOX版本)
```

发送失败计数器在每次成功发送后清零，因此只有在持续发送失败时才会触发恢复。

---

## 7. CAN FD 消息转发引擎

### 7.1 双端口架构

固件维护两个独立的 FDCAN 外设实例，每个端口有独立的 FIFO、过滤器组和中断线：

| 属性 | 端口 2 (R2A) | 端口 3 (A2A) |
|------|-------------|-------------|
| FDCAN 实例 | FDCAN2 | FDCAN3 |
| RX FIFO | FIFO0 | FIFO1 |
| 中断线 | Line 0 | Line 1 |
| TX Buffer | Buffer 0 + Buffer 1 | Buffer 2 |
| 全局过滤器 | REJECT（白名单模式） | REJECT（白名单模式） |
| RX FIFO 模式 | 覆盖模式 | 覆盖模式 |

### 7.2 ID 映射与转发

消息在两个端口之间转发时，CAN ID 按固定偏移规则转换：

```
R2A → A2A 转发:  A2A_ID = R2A_ID - R2A_BASE + A2A_BASE
A2A → R2A 转发:  R2A_ID = A2A_ID - A2A_BASE + R2A_BASE
```

每种消息类别有独立的偏移基准和 ID 范围。所有转发在中断上下文中同步完成——收到消息后立即计算目标 ID 并写入目标端口的 TX FIFO，整个过程不经过任何消息队列。

### 7.3 消息分类与优先级

固件将 CAN 消息按功能分为多种类别，通过 FDCAN 硬件过滤器分配到不同优先级的 FIFO：

**高优先级（FIFO0_HP / FIFO1_HP）**：
- 电机主→从控制指令（实时性要求最高）
- 电机从→主状态反馈
- 电机主→从固定指令
- 电机从→主固定状态
- BOX 数据透传

**普通优先级（FIFO0 / FIFO1）**：
- 根信息
- 电池信息（受 10 Hz 速率限制）
- 电机状态（受 10 Hz 速率限制）
- 电源控制
- Docker 锁定/解锁指令
- Docker 控制指令（掩码过滤器，匹配整个 ID 范围）
- 用户自定义数据（受 200 Hz 速率限制）
- OTA 升级数据
- 系统命令查询
- 心跳消息

### 7.4 中断驱动接收

**端口 2 接收**（`HAL_FDCAN_RxFifo0Callback`）：

```
FIFO0 中断触发:
  HAL_FDCAN_GetRxMessage(hfdcan2, FIFO0, ...)
  ↓
  根据 FilterIndex 分发:
    FILTER_R2A_ROOT            → R2A_ROOT_process()          // 根信息
    FILTER_R2A_MOTER_M2S       → R2A_MOTER_M2S_process()     // 电机主→从
    FILTER_R2A_MOTER_S2M       → R2A_MOTER_S2M_process()     // 电机从→主
    FILTER_R2A_LOW_FREQ_INFO   → R2A_LOW_FREQ_INFO_process() // 低频信息(电池/电机状态/Docker/电源)
    FILTER_R2A_DOCKER_ORDER    → R2A_DOCKER_ORDER_process()  // Docker 指令
    FILTER_OTA_ORDER           → R2A_OTA_ORDER_process()     // OTA/BOX 数据
    FILTER_SYSTEM_ORDER        → R2A_SYSTEM_ORDER_process()  // 系统命令
    FILTER_R2A_USER_INFO       → R2A_USER_INFO_process()     // 用户数据
    FILTER_R2A_MOTER_M2S_FIXED → R2A_MOTER_M2S_FIXED_process()
    FILTER_R2A_MOTER_S2M_FIXED → R2A_MOTER_S2M_FIXED_process()
```

**端口 3 接收**（`HAL_FDCAN_RxFifo1Callback`）：

```
FIFO1 中断触发:
  // ★ 先于过滤器匹配 — 任何 A2A 消息都计入虚假心跳
  continue_fake_heartbeat_cnt += 1

  HAL_FDCAN_GetRxMessage(hfdcan3, FIFO1, ...)
  ↓
  根据 FilterIndex 分发:
    FILTER_A2A_ROOT            → A2A_ROOT_process()
    FILTER_A2A_MOTOR_M2S       → A2A_MOTOR_M2S_process()
    FILTER_A2A_MOTOR_S2M       → A2A_MOTOR_S2M_process()
    FILTER_A2A_USER_INFO       → A2A_USER_INFO_process()
    FILTER_A2A_LOW_FREQ_INFO   → A2A_LOW_FREQ_INFO_process()
    FILTER_A2A_HEARTBEAT       → A2A_HEARTBEAT_process()     // ★ 真实心跳
    FILTER_A2A_MOTER_M2S_FIXED → A2A_MOTOR_M2S_FIXED_process()
    FILTER_A2A_MOTER_S2M_FIXED → A2A_MOTER_S2M_FIXED_process()
    FILTER_A2A_BOX_DATA        → A2A_BOX_DATA_process()      // BOX 透传
```

### 7.5 速率限制

为防止高频消息（如 400 Hz 的电机控制数据）占用过多 A2A 带宽，固件在转发路径中实施了速率限制：

- **用户数据（多通道）**：每 5 ms 允许转发一次（等效 200 Hz）。每个 ID 通道有独立的限流标志 `user_id_xxx_limit_flag`
- **电池信息**：每 50 ms 允许转发一次（10 Hz），由 `battery_info_limit_flag` 控制
- **电机状态**：每 50 ms 允许转发一次（10 Hz），由 `motor_state_limit_flag` 控制

限流实现在中断回调中的转发函数内：先检查限流标志，如果未置位则直接 return 而不转发。限流标志在 5 ms（用户数据）和 50 ms（电池/电机）时隙中由主循环置位。

---

## 8. Docker 锁定/解锁控制

### 8.1 PWM 伺服控制

Docker（电磁锁/舵机）由两路 PWM 信号控制：

- **TIM1 CH2 (PA9)**：Docker 舵机通道 2
- **TIM8 CH1 (PA15)**：Docker 舵机通道 1

两个定时器配置相同：预分频 159（1 MHz 定时器时钟）→ 周期 19999（20 ms PWM 周期，50 Hz 舵机频率）→ 重复计数器 8（每 9 个事件更新一次）。

PWM 脉冲宽度直接对应舵机位置：

```
docker_lock():   pulse = 2000 → 2.0 ms 脉宽 → 锁定位置
docker_unlock(): pulse = 1000 → 1.0 ms 脉宽 → 解锁位置
```

### 8.2 命令路径与执行

Docker 命令可通过两条 CAN 路径到达：

**路径 1 — 来自本机机器人**（端口 2）：
- Docker 锁定消息到达 → 设置 `docker_order_status = DOCKER_LOCK/DOCKER_UNLOCK`
- Docker 指令消息同理
- 在中断回调中设置命令标志，不直接执行 PWM 操作

**路径 2 — 来自对端适配器**（端口 3）：
- 对端 Docker 消息经过 ID 映射后由 `A2A_LOW_FREQ_INFO_process` 处理
- 同样设置 `docker_order_status`，与路径 1 逻辑一致

**实际执行**在 1 s 时隙中：

```
1 s 时隙:
  if docker_order_status == DOCKER_LOCK:
    docker_lock()                      // 设置 TIM1/TIM8 比较值 = 2000
    docker_order_status = DOCKER_UNKNOW
  else if docker_order_status == DOCKER_UNLOCK:
    docker_unlock()                    // 设置 TIM1/TIM8 比较值 = 1000
    docker_order_status = DOCKER_UNKNOW
```

Docker 还会在以下场景被自动控制：
- **启动时**：首次收到心跳后，自动执行 `docker_lock()`（`is_init_timer_8_flag` 保护只执行一次）
- **断连恢复时**：完全分离且未锁定时，自动执行 `docker_lock()`
- **强制断开时**：确保 Docker 为解锁状态（安全考虑——断连时应释放电磁锁）

### 8.3 H 桥驱动器管理

H 桥驱动器（PA2, HDRIVER）为 Docker 舵机供电：
- 有心跳 → `set_hdriver_status(HIGH)` — 使能舵机电源
- 无心跳 → `set_hdriver_status(LOW)` — 关闭舵机电源（省电/安全）

---

## 9. BOX 模式

### 9.1 硬件版本检测

固件通过 ADC2（PA4, IN17）读取分压电阻值，映射到硬件版本号。硬件版本 ≥ 1.6.0 时，`is_box_software_version` 置为 true，固件进入 BOX 模式。

BOX 模式下的适配器不仅执行 CAN FD 桥接，还直接控制箱体的机械部件（电机、传感器）。

### 9.2 透传通道

BOX 模式维护一条双向透传通道，特定 ID 范围的数据在端口 2 和端口 3 之间原封不动地转发，不做 ID 映射。数据流：

```
Robot → port2 → [BOX 适配器] → port3 → 对端（或箱体模块）
Robot ← port2 ← [BOX 适配器] ← port3 ← 对端（或箱体模块）
```

透传通道承载：
- 箱体 A2A 心跳（携带顶部/底部锁定状态）
- 箱体电机控制命令
- OTA 数据（ID 映射：端口 2 ↔ 端口 3 不同 ID 空间）
- 系统命令

### 9.3 箱体电机控制

BOX 模式下，固件直接控制两个直流丝杆电机，通过 PH/EN 对驱动：

| 电机 | PH（方向） | EN（使能） |
|------|-----------|-----------|
| 电机 1 | PB1 | PA7 |
| 电机 2 | PA6 | PB0 |

电机状态枚举：

```
MOTOR_STATUS: UNKNOW → LOCK → UNLOCK → STOP
```

箱体电机控制状态机在 50 ms 时隙中由 `update_box_motor_status()` 更新：

```
每 50 ms:
  if box_bottom_motor_order == LOCK && 当前状态 != LOCK:
    设置两个电机 PH=HIGH, EN=HIGH → 开始锁定运动
    启动 60 秒超时计时

  if box_bottom_motor_order == UNLOCK && 当前状态 != UNLOCK:
    设置两个电机 PH=LOW, EN=HIGH → 开始解锁运动
    立即设置状态 = UNLOCK (无位置反馈，开环控制)
    启动 60 秒超时计时

  if box_bottom_motor_order == STOP:
    设置两个电机 EN=LOW → 停止
    状态 = STOP

  if 60 秒超时:
    强制 STOP (安全保护)
```

### 9.4 箱体传感器与限位

三个限位开关在 5 ms 时隙中读取：

| 传感器 | GPIO | 逻辑 | 含义 |
|--------|------|------|------|
| UP_SIGNAL | PA8 | 上拉, LOW=触发 | 箱体顶部锁定到位 |
| LEFT_SIGNAL | PC8 | LOW=触发 | 左侧限位 |
| RIGHT_SIGNAL | PC9 | LOW=触发 | 右侧限位 |

在 `check_bottom_box_sensor()` 中：当 LEFT 和 RIGHT 同时触发（均为 LOW）且当前命令为 LOCK 时，判定箱体锁定到位 → 停止电机并更新状态为 LOCK。

---

## 10. OTA 升级机制

固件支持通过 CAN FD 总线进行 OTA（空中升级），分为两个阶段：

**阶段 1 — 触发进入引导加载程序**：

```
Robot 发送 OTA 连接请求
  → 适配器收到 → jump2boot()
  → MCU 软复位，从 0x08000000 启动引导加载程序
  → 引导加载程序通过 CAN 接收新固件并写入 APP 区 (0x08012000)
```

**阶段 2 — 升级确认与版本同步**：

```
APP 启动后:
  main() → 读取设备信息 → check_ota_upgrade_sign()
  若检测到升级签名:
    ① verify_dev_info()   — 将设备信息中的软件版本更新为当前版本
    ② return_app_upgrade_ack() — 向 Robot 发送 OTA 确认
    ③ jump2boot() — 再次重启，清除引导加载程序的升级标志
```

**系统命令支持**：
- 获取设备信息：分段返回完整的 328 字节设备信息（通过 OTA 确认通道，每段最多 64 字节 CAN FD 帧）
- 切换 CAN FD 波特率：动态切换 5 Mbps ↔ 8 Mbps，写入闪存后重启生效

---

## 11. 设备信息管理

设备信息（`ADAPTER_DEV_INFO_T`，328 字节）存储在 MCU 闪存的最后一个 2 KB 页面（`0x0803F800`），使用魔数 `0xDEADC0DE` 验证有效性。

读取设备信息时先检查魔数：若魔数无效则使用默认值初始化。写入时先擦除整个页面再编程。

主要字段说明：

| 字段类别 | 说明 |
|----------|------|
| 设备标识 | UUID、产品编号、产品名称、序列号 |
| 版本信息 | 引导加载程序版本、软件版本、硬件版本及各自构建日期 |
| 制造商 | 制造商数据 |
| OTA 标志 | 引导加载程序/应用程序 OTA 确认信号 |
| CAN FD 配置 | 8 Mbps 使能标志（非零=8 Mbps，零=5 Mbps） |
| 其他 | OTA 重启标志 |

硬件版本字段是 BOX 模式检测的依据——固件通过 ADC 读取分压电阻值映射出版本号，与设备信息中存储的硬件版本共同判定。

---

## 12. 安全与恢复机制

固件实现了多层恢复机制，按严重程度分级：

### 12.1 心跳超时恢复

```
heartbeat_timeout_recovery() 在 1 s 时隙中执行:

  if 两个心跳都丢失:
    timeout_sec += 1

    if timeout_sec >= 7 (SERVERE_RECOVERY_CNT):  // 严重级别
      jump2app()  // ★ 软复位整个固件

    else if timeout_sec >= 3 (MILD_RECOVERY_CNT):  // 轻度级别
      // 停止 + 反初始化 FDCAN2 和 FDCAN3
      // 重新初始化 FDCAN2 和 FDCAN3
      // 重新启动端口并配置过滤器
      // ★ 在硬件层面重新建立 CAN 控制器状态

  else:  // 心跳存在
    timeout_sec = 0  // 重置计时
```

恢复策略的核心理念：先尝试重新初始化 CAN 控制器（轻度恢复），若仍无法恢复则软复位整个固件（严重恢复）。

### 12.2 发送失败恢复

在 `heartbeat_pongtong()` 中监控：
- 心跳消息发送失败 ≥ 30 次 → `jump2app()`
- 伪心跳发送失败 ≥ 6000 次 → `jump2app()`

成功发送后对应计数器立即清零，因此只有持续性发送失败才会触发。两种阈值的差异（30 vs 6000）反映了真实心跳和伪心跳的不同重要性。

### 12.3 CAN 总线离线恢复

在 10 s 时隙中执行 `canfd_bus_off_recovery()`：检查 FDCAN2/FDCAN3 的 FIFO 满标志等错误码。当前实现主要作为故障监控占位，为未来的自动恢复逻辑预留。

### 12.4 强制断连恢复

在 1 s 时隙中：`force_a2a_disconnected_recovery_cnt` 达到上限（3 个周期 = 3 秒）后自动清除 `force_a2a_disconnected_flag`，确保强制断开模式不会永久卡死。这是一种兜底保护——即使心跳检测逻辑异常，强制断开状态也会在 3 秒后自动解除。

### 12.5 发送临界区保护

`canfd_send_data()` 在写入 TX FIFO 期间根据当前执行上下文（线程模式 vs 中断模式）决定是否关闭全局中断：

```
if (__get_IPSR() == 0):  // 线程模式
  __disable_irq()         // 关闭中断保护临界区
// ... HAL_FDCAN_AddMessageToTxFifoQ() ...
if (__get_IPSR() == 0 && !irq_was_disabled):
  __enable_irq()
```

这确保在主循环中调用 `canfd_send_data()` 时不会被 FDCAN 中断打断，避免 TX FIFO 的竞态条件。中断上下文中调用时则不需要额外保护。

---

## 13. 状态机汇总

| 状态机 | 关键状态变量 | 触发频率 | 说明 |
|--------|-------------|---------|------|
| **A2A 连接管理** | `is_received_heartbeat`, `is_received_data_fake_heartbeat`, `is_complete_separation`, `force_a2a_disconnected_flag`, `force_a2a_disconnected_cooldown_flag`, `is_just_triggered_separated_flag` | 5 ms + 50 ms + 1 s | 双重心跳驱动的热插拔检测，二级确认断连，自动恢复。分布式状态机，无单一状态变量 |
| **Docker 命令** | `docker_order_status` (UNKNOW/LOCK/UNLOCK) | 中断(设置) + 1 s(执行) | 来自 CAN 总线的锁定/解锁命令延迟到 1 s 时隙执行，保证 PWM 更新时序 |
| **BOX 电机控制** | `box_bottom_motor_order`, 电机状态 (UNKNOW/LOCK/UNLOCK/STOP) | 中断(设置) + 50 ms(执行) | 控制箱体丝杆电机，含 60 s 超时保护，通过限位开关判定到位 |
| **OTA 升级** | `is_boot_confirm_ota_upgrade_signal`, `is_app_confirm_ota_upgrade_signal` | 事件驱动 + 1 s(检查) | 引导加载程序跳转 + 升级确认 + 版本同步，写入闪存设备信息 |
| **心跳超时恢复** | `timeout_sec` | 1 s | 三级恢复：重置计数器 → 重初始化 CAN 控制器（3 s）→ 软复位（7 s） |
| **速率限制** | `user_id_xxx_limit_flag`, `battery_info_limit_flag`, `motor_state_limit_flag` | 5 ms + 50 ms | 防止高频消息占用过多 A2A 带宽，200 Hz / 10 Hz / 10 Hz 分别限流 |

---

## 14. 时序汇总

| 时隙 | 频率 | 执行任务 |
|------|------|---------|
| 中断上下文 | 事件驱动 | CAN 消息接收、ID 映射、消息转发、虚假心跳计数、Docker 命令标志设置 |
| 1 ms | 1 kHz | 预留（当前无任务） |
| 5 ms | 200 Hz | 用户数据限流标志复位、BOX 传感器读取、BOX 限位检查、**热插拔断连确认/重连恢复** |
| 50 ms | 20 Hz | **心跳超时检测与发送**、电池/电机限流复位、BOX 电机状态机 |
| 100 ms | 10 Hz | 预留（当前无任务） |
| 1 s | 1 Hz | **广播发送、Docker 命令执行、OTA 检查、H 桥/LED 控制、心跳超时恢复、强制断开恢复** |
| 10 s | 0.1 Hz | CAN 总线离线恢复检查 |

### 中断优先级

| 中断源 | 优先级 | 说明 |
|--------|--------|------|
| FDCAN2/3 IRQ | 0（最高） | CAN 消息接收与转发，需最低延迟 |
| DMA1 CH1-5 | 0（最高） | PWM 更新与日志 DMA，与 CAN 同优先级 |
| SysTick | 15（最低） | HAL 滴答时钟，不影响实时数据路径 |

### 启动序列

```
上电 / 复位
  │
  ├─ ① HAL_Init() + SystemClock_Config() — 160 MHz
  ├─ ② MX_GPIO_Init(), MX_DMA_Init(), MX_ADC2_Init()
  ├─ ③ 读取设备信息 (闪存最后一页)
  ├─ ④ 判断 BOX 模式 + CAN FD 波特率
  ├─ ⑤ 若 8 Mbps → 配置 TX 延迟补偿
  ├─ ⑥ 读取 GPIO 角色 (Master/Slave)
  ├─ ⑦ MX_TIM1_Init(), MX_TIM8_Init() — 启动 PWM
  ├─ ⑧ MX_TIM3_Init() — 启动 20 kHz PWM
  ├─ ⑨ DWT_Init() — 使能性能计数器
  ├─ ⑩ MX_FDCAN1/2/3_Init() → canfd_start() → set_canfd_filter()
  │
  └─ 进入主循环 (6 级时隙调度)
```

---

> 文档版本 1.0 | 基于固件版本 1.0.0.127 | 2026-07-20
