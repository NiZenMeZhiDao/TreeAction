# Action 接口与通信说明

*最后更新: 2026-06-12*

本文档定义 `SuspensionControl`、`ArmAction`、`SpearAction` 三个语义 Action 的接口规范，以及各层之间的通信协议。

---

## 目录

1. [总体架构](#1-总体架构)
2. [SuspensionControl — 主动悬挂](#2-suspensioncontrol--主动悬挂)
3. [ArmAction — 机械臂](#3-armaction--机械臂)
4. [SpearAction — 矛头机构](#4-spearaction--矛头机构)
5. [通信协议](#5-通信协议)

---

## 1. 总体架构

```
外部规划模块
  │  /planning/segments (std_msgs/String JSON)
  ▼
┌─────────────────────────────────────────────────┐
│  r2_bt (BT 决策层)                               │
│  PopNextSegment → IsSegmentType → Action Client  │
│  黑板: segment fields, execution_state, ...      │
└────────┬──────────┬──────────┬──────────────────┘
         │          │          │
  move_to_pose  suspension  arm_action
         │       _control       │
         ▼          ▼           ▼
┌─────────────────────────────────────────────────┐
│  r2_hardware (硬件执行层)                         │
│  Action Server: 状态机 / 控制循环                 │
│  发布: /t0x0101, /t0x0102_action, /t0x0103_     │
└────────┬──────────┬──────────┬──────────────────┘
         │          │          │
         └──────────┼──────────┘
                    │  Float32MultiArray (USB 透传)
                    ▼
┌─────────────────────────────────────────────────┐
│  ares_usb (USB Bridge)                          │
│  SyncFrame 封装 → USB Bulk 发送                  │
└────────────────────┬────────────────────────────┘
                     │  USB Bulk (64B/帧)
                     ▼
┌─────────────────────────────────────────────────┐
│  下位机 (STM32)                                  │
│  电机控制 / 传感器采集 / 状态回传                   │
└─────────────────────────────────────────────────┘
```

三层 Action 全部使用 ROS 2 Action 协议（`rclcpp_action` / `rclpy.action`），BT 层通过 `StatefulActionNode` 异步调用，硬件层通过 `ActionServer` 执行。

---

## 2. SuspensionControl — 主动悬挂

### 2.1 概述

四轮独立悬挂升降 Action，配合底盘运动完成上下台阶。支持四种工作模式，由内部状态机自动执行升降序列。

- **ROS 2 Action 服务名**: `suspension_control`（默认，可通过 BT 端口配置）
- **Action 定义**: `r2_interfaces/action/SuspensionControl.action`
- **BT 节点**: `r2_bt::SuspensionControl` (`StatefulActionNode`)
- **Action Server**: `r2_hardware/action_servers/suspension_action_server.py`

### 2.2 Goal（请求）

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `mode` | `uint8` | 是 | 工作模式 |
| `direction` | `uint8` | 是 | 机器人相对台阶的运动方向 |
| `height` | `float32` | 是 | 台阶高度 (mm)；正=上台阶，负=下台阶，0=AUTO 自动判断；已知 200/400 时直接映射准备高度 |
| `timeout_sec` | `float32` | 是 | 超时秒数，超时后 Action 中止 |

#### mode 枚举

| 值 | 常量 | 说明 |
|----|------|------|
| `0` | `MODE_AUTO` | 根据 `height` 符号自动选择上/下台阶 |
| `1` | `MODE_CLIMB_UP` | 上台阶 |
| `2` | `MODE_CLIMB_DOWN` | 下台阶 |
| `3` | `MODE_DIRECT` | 直接设置四轮统一高度（height 参数为目标高度 mm） |

#### direction 枚举

| 值 | 常量 | 说明 |
|----|------|------|
| `0` | `DIR_FORWARD` | 前进方向 |
| `1` | `DIR_LEFT` | 左侧 |
| `2` | `DIR_RIGHT` | 右侧 |


### 2.3 Result（响应）

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 是否成功完成 |
| `message` | `string` | 结果描述 (`"Sequence complete"`, `"Recovered"`, `"Timeout"`, `"Cancelled"`) |
| `final_state` | `int32` | 完成时的状态机状态码 |
| `elapsed_sec` | `float32` | 实际耗时 (s) |

### 2.4 Feedback（执行中反馈）

| 字段 | 类型 | 说明 |
|------|------|------|
| `current_state` | `int32` | 当前状态机状态码 |
| `elapsed_sec` | `float32` | 已耗时 (s) |
| `distance_data` | `float32[8]` | 8 路 TOF 测距滤波值 (mm) |
| `photoelectric_data` | `float32[4]` | 4 路光电开关消抖值 |
| `wheel_heights_current` | `float32[4]` | 四轮当前高度 (mm) |
| `wheel_heights_target` | `float32[4]` | 四轮目标高度 (mm) |

### 2.5 状态机

状态码 | 状态 | 说明
-------|------|------
`0` | `IDLE` | 空闲
**上台阶** | |
`10` | `UP_1_PREPARE` | 四轮升到准备高度；已知 200mm→205mm，已知 400mm→410mm
`11` | `UP_2_LIFT` | 仅未知高度时用距离判断台阶高度，决定是否继续升高
`12` | `UP_3_FRONT_DOCK` | 等待前轮距离 < 80mm（接近台阶）
`13` | `UP_4_RETRACT_FRONT` | 前两轮缩回到 5mm
`14` | `UP_5_FRONT_LAND` | 等待前轮光电触发 → 前轮着台阶
`15` | `UP_6_SIDE_DOCK_RETRACT_REAR` | 等待侧轮光电触发 → 中轮着台阶 → 后轮缩回
`16` | `UP_7_REAR_LAND` | 等待后轮光电触发 → 后轮着台阶
`17` | `UP_8_RECOVER` | 四轮恢复到 20mm → IDLE
**下台阶** | |
`20` | `DOWN_1_PREPARE` | 四轮降到下台阶准备高度，等待前轮离开台阶边缘
`21` | `DOWN_2_FRONT_HOVER_LAND` | 前两轮抬升至目标高度，前轮悬空着陆
`22` | `DOWN_3_WAIT_REAR_HOVER_LAND` | 等待后轮到达台阶边缘
`23` | `DOWN_4_REAR_HOVER_LAND` | 后两轮抬升至目标高度，等待后轮距离 > 200mm 判断已下台阶
`24` | `DOWN_5_RECOVERY` | 四轮恢复到 20mm → IDLE

### 2.6 BT 节点端口

**输入端口**:

| 端口名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `mode` | `int` | `0` | `0=AUTO, 1=CLIMB_UP, 2=CLIMB_DOWN, 3=DIRECT` |
| `direction` | `int` | `0` | `0=FORWARD, 1=LEFT, 2=RIGHT` |
| `height` | `double` | `0.0` | 台阶高度 (mm)；0 表示使用默认；MODE_DIRECT 时为直接设置的目标高度 |
| `timeout_sec` | `double` | `30.0` | 超时秒数 |
| `server_name` | `string` | `"suspension_control"` | Action 服务名 |

**输出端口**:

| 端口名 | 类型 | 说明 |
|--------|------|------|
| `message` | `string` | 结果消息 |
| `final_state` | `int` | 最终状态码 |

### 2.7 硬件层通信

```
BT Node (C++)                               Action Server (Python)
     │                                              │
     ├── async_send_goal ─────────────────────────► │
     │   (rclcpp_action, /suspension_control)       │
     │                                              ├── 订阅 /sensor_distances (TOF)
     │                                              ├── 订阅 /r0x0201 (下位机状态)
     │◄── publish_feedback ──────────────────────── │ (100Hz)
     │                                              │
     │◄── goal result ───────────────────────────── │
     │                                              │
                                                     │
                                                     ▼
                                           发布 /t0x0102_action
                                           (Float32MultiArray [h0,h1,h2,h3])
                                                     │
                                                     ▼
                                           ares_usb → SyncFrame(0x0102)
                                                     → USB Bulk → STM32
```

- `/t0x0102_action`：四轮目标高度 `[FL, FR, RL, RR]`，单位 mm，100Hz 发布
- `/r0x0201`：下位机状态回传，包含光电开关 `[PE0,PE1,PE2,PE3]` + 当前轮高 `[H0,H1,H2,H3]`
- `/sensor_distances`：8 路 TOF 距离传感器数据
- 已知 `height` 时，Action Server 不再依赖 TOF 判定 200/400 台阶高度；上 200mm 台阶直接准备到 205mm，上 400mm 台阶直接准备到 410mm，动作结束恢复到 20mm。

---

## 3. ArmAction — 机械臂

### 3.1 概述

语义化机械臂控制 Action，统一所有机械臂操作。支持同步和后台两种执行模式。

- **ROS 2 Action 服务名**: `arm_action`（默认，可通过 BT 端口配置）
- **Action 定义**: `r2_interfaces/action/ArmAction.action`
- **BT 节点**: `r2_bt::ArmAction` (`StatefulActionNode`)
- **Action Server**: **待实现**（BT 客户端已完备）

### 3.2 Goal（请求）

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `command` | `string` | 是 | 语义命令名 |
| `wait_result` | `bool` | 否 | `true`=同步等待结果，`false`=后台执行立即返回 |
| `timeout_sec` | `float32` | 否 | 超时秒数 |

### 3.3 Result（响应）

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 是否成功 |
| `message` | `string` | 结果描述 |
| `final_state` | `int32` | 最终状态 |
| `elapsed_sec` | `float32` | 实际耗时 (s) |

### 3.4 Feedback（执行中反馈）

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | `string` | 当前执行的子状态描述 |
| `progress` | `float32` | 进度 (0.0 ~ 1.0) |
| `message` | `string` | 当前状态消息 |

### 3.5 command 枚举

| command | 值 | 使用区域 | 说明 |
|---------|-----|----------|------|
| `idle` | 0 | 全区域 | 空闲 / 停止 |
| `grasp` | 1 | 梅林 | 抓取 KFS，到稍微安全位置，不影响车移动 |
| `store_to_body` | 2 | 梅林 | 将 KFS 转存到车体储存位，再回到空闲位置 |
| `store_on_arm` | 3 | 梅林 | KFS 暂持在机械臂上 |
| `get_body` | 4 | 竞技 | 取出车内的 KFS 到机械臂上 |
| `place_mid` | 5 | 竞技 | 放置 KFS 到中层槽位 |
| `place_high` | 6 | 竞技 | 放置 KFS 到上层槽位 |



### 3.6 后台执行模式

当 `wait_result=false` 时：

1. `ArmAction` BT 节点发送 Goal 后立即返回 `SUCCESS`
2. Python Action Server 发布后台状态到 `/arm_runtime_state` (std_msgs/Bool)
3. 后续 `WaitArmIdle` 节点订阅 `/arm_runtime_state`，等待 `true` 表示完成
4. 后台失败通过 `Bool.data = false` 传递

**典型用法**：

```xml
<!-- STORE1 后台执行，底盘先走 -->
<Sequence>
  <IsSegmentType expected="STORE1" actual="{segment_type}"/>
  <ArmAction command="store_to_body" wait_result="false"/>
</Sequence>

<!-- 下次抓取前必须等待后台完成 -->
<Sequence>
  <IsSegmentType expected="GRASP" actual="{segment_type}"/>
  <WaitArmIdle/>
  <Parallel success_count="2" failure_count="1">
    <MoveToPose .../>
    <ArmAction command="grasp" wait_result="true"/>
  </Parallel>
</Sequence>
```

### 3.7 BT 节点端口

**输入端口**:

| 端口名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `command` | `string` | *必填* | 语义命令名 |
| `server_name` | `string` | `"arm_action"` | Action 服务名 |

**输出端口**:

| 端口名 | 类型 | 说明 |
|--------|------|------|
| `message` | `string` | 结果消息 |

### 3.8 硬件层通信

```
BT Node (C++)                    Action Server (待实现)
     │                                    │
     ├── async_send_goal ────────────────► │
     │   (rclcpp_action, /arm_action)      │
     │                                     ├── 订阅 /r0x0201 (下位机状态)
     │◄── publish_feedback ─────────────── │
     │                                     │
     │◄── goal result ──────────────────── │
     │                                     │
                                           │
                                           ▼
                                   发布 /t0x0103_
                                   (Float32MultiArray, 抓取指令)
                                           │
                                           ▼
                                   ares_usb → SyncFrame(0x0103)
                                           → USB Bulk → STM32
```

> **注意**：`/arm_action` Action Server 尚未实现。BT 客户端 (`ArmAction` BT 节点) 已完备，包含异步发送、超时、取消和后台执行逻辑。实现 Action Server 时需要：
> 1. 创建 `r2_interfaces::action::ArmAction` 的 Action Server
> 2. 根据 `command` 字段分发到对应机械臂控制逻辑
> 3. 订阅 `/r0x0201` 获取硬件状态反馈
> 4. 发布 `/t0x0103_` 控制抓取机构
> 5. 通过 Feedback 上报执行进度和当前子状态

---

## 4. SpearAction — 矛头机构

### 4.1 概述

控制矛头机构的专用 Action，用于准备区的矛头抓取和对接流程。与 `ArmAction` 操作不同的物理机构（矛头夹爪/锁紧/伸缩 vs 机械臂吸盘/关节）。

> **状态**：`SpearAction` 已完整实现独立的 `.action` 文件、BT 节点和 Action Server。准备区 segment 类型已从 `PICK_SPEAR_HEAD`/`DOCK_SPEAR` 迁移至 `SPEAR_PREP`/`SPEAR_GRASP`/`ALIGN`/`DOCK`。

### 4.2 建议接口定义

#### Goal（请求）

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `command` | `string` | 是 | 语义命令 |
| `timeout_sec` | `float32` | 否 | 超时秒数（默认 5.0） |

#### command 枚举

| command | 对应 Segment | 说明 |
|---------|-------------|------|
| `prepare` | `SPEAR_PREP` | 张开夹爪、解锁机构、移动到待抓取位姿 |
| `grasp` | `SPEAR_GRASP` | 夹爪闭合抓取矛头 → 检测到位 → 锁紧 - 稍微抬起机构，使可以取出矛头 |
| `dock_extend` | `DOCK` | 将矛头伸出到准备对接的位置 |
| `dock_release` | `DOCK` | 对接，然后松开矛头并收回机械臂 |

#### Result（响应）

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 是否成功 |
| `message` | `string` | 结果描述 |
| `elapsed_sec` | `float32` | 实际耗时 (s) |

#### Feedback（执行中反馈）

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | `string` | 当前子状态描述 |
| `progress` | `float32` | 进度 (0.0 ~ 1.0) |

### 4.3 流程时序

```
准备区流程:

SPEAR_PREP:
  1. SpearAction(command=prepare)  — 张开夹爪、解锁
  2. Align                          — 底盘精调对准
  → 硬同步点，完成后进入下一步

SPEAR_GRASP:
  3. SpearAction(command=grasp)    — 夹爪闭合 → 检测 → 锁紧
  → 硬同步点，不能和底盘移动并行
  → 成功后矛头固定在车体上
  → 失败（超时/传感器未确认）进入异常处理

ALIGN:
  4. Align                          — 精调到对接位

DOCK:
  5. SpearAction(command=dock_extend)  — 矛头伸出到对接位
  6. 等待 R1 到位信号 (Condition 节点轮询 IO)
  
  7. SpearAction(command=dock_release) — 松开矛头、收回
  8. 机械臂回到安全位
```

### 4.4 与 ArmAction 的关系

| 维度 | ArmAction | SpearAction |
|------|-----------|-------------|
| 控制对象 | 机械臂（吸盘、关节） | 矛头机构（夹爪、锁紧、伸缩） |
| 使用区域 | 梅林、竞技 | 准备 |
| 坐标系 | `arm_base` | 车体 + 视觉定位 |
| 传感器 | 吸力/距离 | 抓取力/到位传感器 |
| 硬件 topic | `/t0x0103_` | `/t0x0103_` 或独立 topic |

两者物理上可能共用下位机抓取指令通道 (`/t0x0103_`)，通过 command 区分指令内容。如果控制逻辑差异大，建议使用独立 topic 或 DataID。



迁移步骤（已完成）：

1. ✅ 在 `r2_interfaces/action/` 中创建 `SpearAction.action`
2. ✅ 在 `r2_bt` 中创建 `SpearAction` BT 节点
3. ✅ 在 `r2_hardware` 中创建 `spear_action` Action Server
4. ✅ 更新 `bt_engine_node.cpp` 中的 segment 解析逻辑
5. ✅ 更新 BT XML 树中的准备区子树

---

## 5. 通信协议

### 5.1 ROS 2 Action 协议（BT ↔ 硬件层）

三个 Action 统一使用 ROS 2 Action 协议：

```
Client (BT Node / StatefulActionNode)       Server (Hardware Action Server)
     │                                                │
     ├── async_send_goal(goal) ──────────────────────►│  goal_callback
     │                                                │    → ACCEPT / REJECT
     │◄─── goal_response_callback ────────────────────┤
     │                                                │
     │◄─── publish_feedback(feedback) ────────────────┤  周期发布
     │                                                │
     │◄─── result_callback(result) ───────────────────┤  完成/超时/取消
     │                                                │
     │   async_cancel_goal() ────────────────────────►│  cancel_callback
```

**BT 节点约定**：
- `onStart()`：构造 Goal、异步发送、返回 `RUNNING`
- `onRunning()`：检查 `goal_done_` 标志，返回 `SUCCESS` / `FAILURE` / `RUNNING`
- `onHalted()`：取消正在执行的 Action，返回 `FAILURE`
- 不在 BT 节点中 `sleep` 或阻塞等待

**Action Server 约定**：
- 使用 `ReentrantCallbackGroup` 支持并发 goal 处理
- `execute_callback` 在循环中发布 Feedback、检查取消和超时
- 超时调用 `goal_handle.abort()`，取消调用 `goal_handle.canceled()`
- 成功调用 `goal_handle.succeed()`

### 5.2 USB 透传协议（硬件层 ↔ 下位机）

`ares_usb` 包将 ROS 2 topic 透明映射为 USB Bulk SyncFrame。

#### 5.2.1 Topic → DataID 映射

| ROS 2 Topic | DataID | 方向 | Payload | 频率 | 说明 |
|-------------|--------|------|---------|------|------|
| `/t0x0101_` | `0x0101` | ROS→MCU | `float32[3]` [vx, vy, vyaw] | ≤100Hz | 底盘速度指令 |
| `/t0x0102_action` | `0x0102` | ROS→MCU | `float32[4]` [h0, h1, h2, h3] | 100Hz | 四轮悬挂目标高度 (mm) |
| `/t0x0103_` | `0x0103` | ROS→MCU | arm抓取相关指令 | 按需 | 夹爪/吸盘控制 |
| `/t0x0104_` | `0x0104` | ROS→MCU | spear抓取相关指令 | 按需 | 夹爪/吸盘控制 |
| `/r0x0201` | `0x0201` | MCU→ROS | `float32[4]` 底盘实际速度 | 100Hz |  |
| `/r0x0202` | `0x0202` | MCU→ROS | `float32[12+]` [PE0-3, H0-3, ...] | 200Hz | 光电开关 + 轮高反馈 |
| `/r0x0203` | `0x0203` | MCU→ROS | `float32[9]` armaction完成情况+arm电机角度 | 100Hz | |
| `/r0x0204` | `0x0204` | MCU→ROS | `float32[9]` spearaction完成情况+spear电机角度或者位置反馈 | 100Hz |


#### 5.2.2 帧格式

```
SyncFrame (0x5A5A):
┌──────┬────────┬─────────────────────────┬──────┐
│ Head │ DataID │     Payload (≤60B)       │ CRC  │
│ 2B   │ 2B     │     可变长度              │ 1B   │
└──────┴────────┴─────────────────────────┴──────┘
  BE      BE         LE (float32)            CRC-8
```

- **字节序**：Head/DataID 为大端 (BE)，Payload 中 float32 为小端 (LE)
- **CRC**：CRC-8 (多项式 0x07)，不含 Head 和 Tail
- **心跳**：每 3ms 双向发送 ErrorFrame (`request_id=0xFF, error_code=0x0100`)

#### 5.2.3 Topic 自动发现

`usb_bridge_node` 在 1Hz 定时器上动态扫描所有以 `t0x` 开头的 topic，自动创建订阅并转发为 SyncFrame。这意味着硬件层只需以 `t0x<hex>` 命名发布 topic，USB bridge 即可自动透传，无需修改 bridge 代码。

多开发板场景下，`usb_bridge_node` 按 USB PID 与 DataID 高字节分流，不再向所有开发板广播：

| USB PID | DataID 范围 | 说明 |
|---------|-------------|------|
| `0x0001` | `0x01xx` | 只透传 01 开头的 DataID |
| `0x0002` | `0x02xx` | 只透传 02 开头的 DataID |

### 5.3 传感器数据流

```
┌──────────────────┐
│ 8× TOF 距离传感器  │  串口 /dev/ttyCH9344USB0~7
│ (230400 baud)    │  195B/包, 起始字节 0xAA
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ multi_serial_node │  解析 TOF 距离 (offset=10, stride=15)
│ (Python)         │  置信度过滤: confidence==100
└────────┬─────────┘
         │  /sensor_distances (Float32MultiArray[8], 100Hz)
         ▼
┌──────────────────┐
│ SuspensionAction │  用于台阶检测、距离判断
│ Server           │
└──────────────────┘

┌──────────────────┐
│ 下位机 (STM32)    │
└────────┬─────────┘
         │  USB Bulk SyncFrame 0x0201
         │  /r0x0201 (Float32MultiArray)
         ▼
┌──────────────────┐
│ SuspensionAction │  光电开关 [PE0-3] + 当前轮高 [H0-3]
│ Server           │
└──────────────────┘
```

### 5.4 规划输入协议（外部 → BT）

外部规划模块通过 ROS 2 topic 发送 segment 序列：

暂时未定

支持的 `stage` 值：`FULL_MATCH`、`PREPARE`、`MEILIN`、`FINAL`

BT 引擎收到后解析 JSON → 填充 `Segment` 结构体 → 入队 `SegmentQueue` → `PopNextSegment` 逐个弹出执行。

### 5.5 坐标系约定汇总

| 数据 | 坐标系 | 单位 |
|------|--------|------|
| `move_target.x/y` | 全局世界坐标 | m |
| `move_target.yaw` | 全局世界朝向 | rad |
| `max_vel` (`max_speed` legacy alias) | — | m/s |
| `max_wz` | — | rad/s |
| `climb_height` / `height` | — | mm |
| `timeout_sec` | — | s |
| 悬挂高度 | — | mm |
| TOF 距离 | 传感器局部 | mm |

### 5.6 Action Server 命名约定

| Action | 默认服务名 | 可配置 |
|--------|-----------|--------|
| `MoveToPose` | `move_to_pose` | 否 |
| `SuspensionControl` | `suspension_control` | 是 (`server_name` 端口) |
| `ArmAction` | `arm_action` | 是 (`server_name` 端口) |
| `SpearAction` (待实现) | `spear_action` | 建议可配置 |

---

## 附录 A：当前实现状态

| 组件 | SuspensionControl | ArmAction | SpearAction |
|------|:--:|:--:|:--:|
| `.action` 定义 | ✅ | ✅ | ✅ |
| BT 节点 (C++) | ✅ | ✅ | ✅ |
| Action Server (Python) | ✅ | ✅ | ✅ |
| 硬件 topic 下发 | ✅ (`/t0x0102_action`) | ✅ (`/t0x0103_`) | ✅ (`/t0x0104_`) |
| BT XML 集成 | ✅ | ✅ | ✅ |
| 后台执行模式 | — | ✅ | — |

**备注**: SpearAction 已完整实现。Segment 类型已从 `PICK_SPEAR_HEAD`/`DOCK_SPEAR` 更新为 `SPEAR_PREP`/`SPEAR_GRASP`/`ALIGN`/`DOCK`。

**待实现**:
- DOCK segment 中等待 R1 到位信号的 Condition 节点

## 附录 B：相关文件索引

| 文件 | 内容 |
|------|------|
| `r2_interfaces/action/SuspensionControl.action` | Suspension Action 接口定义 |
| `r2_interfaces/action/ArmAction.action` | Arm Action 接口定义 |
| `r2_interfaces/action/SpearAction.action` | Spear Action 接口定义 |
| `Motion_control_accurate/src/action_of_motion_interfaces/action/MoveToPose.action` | Motion_control_accurate MoveToPose Action 接口定义 |
| `r2_bt/include/r2_bt/nodes/actions/suspension_control.hpp` | Suspension BT 节点头文件 |
| `r2_bt/include/r2_bt/nodes/actions/arm_action.hpp` | Arm BT 节点头文件 |
| `r2_bt/include/r2_bt/nodes/actions/spear_action.hpp` | Spear BT 节点头文件 |
| `r2_bt/src/nodes/suspension_control.cpp` | Suspension BT 节点实现 |
| `r2_bt/src/nodes/arm_action.cpp` | Arm BT 节点实现 |
| `r2_bt/src/nodes/spear_action.cpp` | Spear BT 节点实现 |
| `r2_bt/include/r2_bt/segment.hpp` | Segment 和 ArmRuntimeState 数据结构 |
| `r2_bt/src/bt_engine_node.cpp` | Segment JSON 解析和入队逻辑 |
| `r2_hardware/r2_hardware/action_servers/suspension_action_server.py` | Suspension Action Server |
| `r2_hardware/r2_hardware/action_servers/arm_action_server.py` | Arm Action Server |
| `r2_hardware/r2_hardware/action_servers/spear_action_server.py` | Spear Action Server |
| `Motion_control_accurate/src/action_of_motion/action_of_motion/motion_action_node.py` | Motion_control_accurate MoveToPose Action Server |
| `ares_usb/ares_usb_comm_design.md` | USB 通信协议详细设计 |
| `design.md` | 全区域流程设计和 SpearAction 语义定义 |
| `r2_bt/trees/full_match.xml` | 全赛程 BT XML 树 |
| `r2_bt/trees/meilin_stage.xml` | 梅林区 BT XML 树 |
