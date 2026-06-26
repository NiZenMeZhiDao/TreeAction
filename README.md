# R2 Autonomy Core

RoboCon 2026 "武林探秘" R2 全自主决策与执行中间件。

本仓库负责把外部全流程规划结果转换为机器人语义动作：接收 `/planning/segments` 的 JSON（准备区/竞技区）和 `/mf_action_seq` 的 Float32MultiArray（梅林区），维护 `segment_queue`，由固定 BehaviorTree.CPP 顶层树和 3 个子树逐段调用底盘、悬挂、机械臂和矛头机构 Action。本仓库不负责路径规划、KFS 选择、入口/出口选择、路线合法性判断或 R1/R2 无线通信。

## 架构

```text
外部规划模块
  ├── /planning/segments (std_msgs/String JSON)    ← 准备区/竞技区
  └── /mf_action_seq (std_msgs/Float32MultiArray)  ← 梅林区
        ↓
r2_bt (bt_engine_node)
  ├── 固定 BT XML: full_match.xml
  │     ├── PrepareArea  (矛头抓取、对接)
  │     ├── MeilinArea   (KFS 拿取、搬运、放置)
  │     │     ├── PopNextMeilinSegment → Move / Fetch
  │     │     └── Move/Fetch 内部完成 grid→world、爬台计算、抓取位姿计算
  │     └── FinalArea    (中层/上层 KFS 放置)
  ├── segment_queue
  ├── Groot2 调试黑板
  └── Action Clients
        ↓
r2_hardware (Action Server 层)
  ├── move_to_pose / align        (底盘导航/精调)
  ├── suspension_control          (四轮独立悬挂)
  ├── arm_action                  (机械臂语义控制)
  └── spear_action                (矛头机构控制)
        ↓
ares_usb / sensor nodes / 下位机
```

### 包职责

| 包 | 职责 |
|----|------|
| `r2_bt` | BT 决策层；解析 `/mf_action_seq` 和 JSON segment 计划，按固定树和 3 个子树调度语义 Action，维护 Groot2 调试字段 |
| `r2_interfaces` | 跨包 Action 接口定义：`MoveToPose`、`Align`、`SuspensionControl`、`ArmAction`、`SpearAction` |
| `r2_hardware` | 硬件执行层：各 Action Server 实现（底盘、悬挂、机械臂、矛头机构）及传感器驱动 |
| `r2_planner` | 规划层（保留旧翻译器代码，当前梅林区由 bt_engine_node 直接解析 `/mf_action_seq`） |
| `ares_usb` | ROS 2 与下位机 USB 透传 |
| `r2_bringup` | 仿真/实车启动文件 |

## 梅林区路径规划对接

梅林区使用外部路径规划模块 `mf_action_planner`（[GitHub](https://github.com/BetterOIer/mf_action_planner)），本仓库不重新规划、不改写动作顺序。

### 数据流

```text
mf_action_planner (外部)
      │  /mf_action_seq  std_msgs/Float32MultiArray
      ▼
r2_bt::bt_engine_node
      │  解析 → Segment（仅存原始值: row, col, height, yaw）
      │  入队 SegmentQueue → PopNextMeilinSegment → Move / Fetch
      │  Move/Fetch 内部完成: grid→world、爬台计算、抓取位姿计算
      │  超时等配置由 MeilinConfig (blackboard) 提供
      ▼
硬件 Action Server (r2_hardware)
```

### 输入格式：/mf_action_seq

类型 `std_msgs/Float32MultiArray`，每 8 个 float 为一条动作，多条动作以批方式一次发送完整个梅林区计划。

| 参数 | 含义 |
|------|------|
| `arg0` | 动作类型：`0`=move, `1`=fetch |
| `arg1` | 目标格子行 row (int, 0~5)，以武馆区右侧第一个台阶前方为 `(0,0)` |
| `arg2` | 目标格子列 col (int, 0~2) |
| `arg3` | move 时为目标高度 (mm)，与内置 height_map 相互检查；fetch 时为抓取前升降高度差 (mm)，正=上升、负=下降 |
| `arg4` | 目标朝向 (弧度)，逆时针为正，范围 [−π, π] |
| `arg5~7` | 保留，暂未使用 |

### BT 行为树执行（梅林区）

梅林区不使用旧的 `MOVE2`/`GRASP`/`CLIMB`/`STORE*` segment 类型，而是使用新的 `move`/`fetch` 语义：

```xml
<PopNextMeilinSegment queue="{segment_queue}"
                      segment_type="{segment_type}"
                      move_row="{move_row}"
                      move_col="{move_col}"
                      target_height_mm="{target_height_mm}"
                      kfs_row="{kfs_row}"
                      kfs_col="{kfs_col}"
                      height_diff="{height_diff}"
                      target_yaw="{target_yaw}"/>
```

Tree 只暴露 planner 原始值（row/col/height/yaw），所有计算在 Move/Fetch 内部完成：

| BT 节点 | 输入端口 | 内部计算 |
|---------|----------|----------|
| `Move` | `move_row`(int), `move_col`(int), `target_height_mm`(double), `target_yaw`(double) | grid→world、高度差→爬台判断、爬台方向/模式、pre-align |
| `Fetch` | `kfs_row`(int), `kfs_col`(int), `height_diff`(double), `target_yaw`(double) | KFS→抓取位姿、悬挂模式推导（正=升/负=降）、arm_command 固定 GRASP |

### Move 详细逻辑

```
onStart():
  1. 读取输入: move_row(int), move_col(int), target_height_mm, target_yaw
  2. 读取 meilin_config (MeilinConfigPtr) from blackboard
  3. 读取当前状态:
       优先使用 /transformed/pose 的 map 系 base_link 位姿
       world→nearest grid 得到 current_row/col
       yaw 使用 PoseStamped.orientation
       到格中心距离 <= meilin_cell_center_tolerance 时认为 pose_is_cell_center=true
       定位未收到或超时则降级使用 blackboard 状态
  4. grid→world: target_x = origin_x + row × grid_size, target_y = origin_y + col × grid_size
  5. 爬台判断（如需 → onStart 直接发出）:
       height_diff = target_height_mm - current_height
       if |height_diff| > height_tolerance:
         climb_needed = true
         climb_mode    = 1 (UP) or 2 (DOWN)
         climb_height  = |height_diff|
         climb_direction = direction_yaw(current→target)  // 0=FWD,1=LEFT,2=RIGHT
  6. pre_align 判断（如需回中，与 climb 并行发出；TODO: 如果回中会绕路则不回中）:
       if not pose_is_cell_center:
         pre_align target = current cell center

Phase 流程（onStart 直接发 action，tick 检查并推进）:
  Phase::START — 并行发出 pre_align + climb:
    ├── if pre_align_needed  → [TODO: Align(pre_align)]
    └── if climb_needed      → [TODO: SuspensionControl(climb)]
         → Phase::WAIT_PHASE1

  Phase::WAIT_PHASE1 — 等 pre_align 完成（climb 继续并行）:
    └── pre_align done?  → 发 [TODO: Align(target)]（与 climb 并行）
         → Phase::WAIT_PHASE2

  Phase::WAIT_PHASE2 — 等 target_align 和 climb 都完成:
    └── all done?  → Phase::DONE

  Phase::DONE:
    更新 blackboard: meilin_current_row/col/height/yaw = 目标值
    meilin_pose_is_cell_center = true
    return SUCCESS

  执行时序示意（有 pre_align + climb 的完整路径）:
    tick0:  ┬ Align(pre_align)
            └ Suspension(climb)
    tick1:  ┬ Align(pre_align) ✓ → 发 Align(target)
            └ Suspension(climb) 继续
    tick2:  ┬ Align(target) ✓
            └ Suspension(climb) ✓ → DONE
```

> **当前状态**: 完整计算 + RCLCPP_INFO 日志输出，TODO 标记处为待接入的 Action Client。Align 串行（pre_align → target），悬挂与 Align 并行。代码见 `r2_bt/src/nodes/meilin_move.cpp`。

### Fetch 详细逻辑

和 Move 类似：**Align 和 ArmAction 串行，Suspension 与 Align 并行**。

```
onStart():
  1. 读取输入: kfs_row(int), kfs_col(int), height_diff(带正负), target_yaw
  2. 读取 meilin_config from blackboard
  3. 计算抓取位姿:
       KFS 中心 = grid_to_world(kfs_row, kfs_col)
       接近方向 = direction_yaw(current→KFS)  // 仅 FWD/LEFT/RIGHT
       接近距离 = grid_size/2 + grasp_distance
       grasp_x = KFS_x - unit_x × 接近距离
       grasp_y = KFS_y - unit_y × 接近距离
  4. 悬挂模式推导:
       abs_diff = |height_diff|
       if abs_diff > height_tolerance:
         suspension_mode = 1 (上升) or 2 (下降)
       else: 无需调整

Phase 流程（onStart 直接发 action，tick 检查并推进）:
  Phase::START — 并行发出 Align(grasp) + Suspension:
    ├── [TODO: Align(grasp)]           ← 串行链起点
    └── if suspension_mode≠0 → [TODO: SuspensionControl]  ← 与 Align 并行
         → Phase::WAIT_PHASE1

  Phase::WAIT_PHASE1 — 等 Align 和 Suspension 都完成:
    └── both done?  → Phase::START_ARM

  Phase::START_ARM — Align 完成后才发 Arm（不能边动边抓）:
    └── [TODO: ArmAction(command=GRASP, wait_result=true)]
         → Phase::WAIT_ARM

  Phase::WAIT_ARM:
    └── arm done?  → Phase::DONE

  Phase::DONE:
    更新 blackboard: meilin_current_yaw = target_yaw
    meilin_pose_is_cell_center = false
    return SUCCESS

  执行时序示意（有悬挂的完整路径）:
    tick0:  ┬ Align(grasp)
            └ Suspension
    tick1:  ┬ Align ✓ ┐
            └ Suspension ✓ ┘ → 发 ArmAction(GRASP)
    tick2:  ArmAction ✓ → DONE
```

> **当前状态**: 完整计算 + RCLCPP_INFO 日志输出，TODO 标记处为待接入的 Action Client。代码见 `r2_bt/src/nodes/meilin_fetch.cpp`。

### 方向约束

方向由 `meilin_direction_yaw(from_row, from_col, to_row, to_col)` 计算，基于相邻格子相对位置：

| 位移 | yaw (rad) | direction 枚举 | 名称 |
|------|-----------|----------------|------|
| (+1, 0) 向前 | 0 | 0 | FORWARD |
| (0, +1) 向左 | π/2 | 1 | LEFT |
| (0, -1) 向右 | −π/2 | 2 | RIGHT |
| (-1, 0) 向后 | π | — | **禁止** |

### 坐标计算

由 Move/Fetch 内部使用 `MeilinConfig`（blackboard 上的共享配置）完成，外部无需关心：

```
格子中心: x = origin_x + row × grid_size
          y = origin_y + col × grid_size
抓取位:   根据 接近方向 确定，停在边线外 grasp_distance (=0.4m)
          若 current=(r,c) 在 KFS 左侧 → grasp_x = KFS_x - (grid_size/2 + grasp_distance)
攀爬边线: x = origin_x + (min(from_row, to_row) + 0.5) × grid_size
          y = origin_y + col × grid_size
```

### 配置参数

`bt_engine_node` 启动时从 ROS 参数加载，写入 blackboard 的 `meilin_config` (MeilinConfig 结构体)：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `meilin_side` | `blue` | `blue`/`red`，影响高度地图 (row=2 红蓝相反) |
| `meilin_grid_size` | `1.2` | 格子尺寸 (m) |
| `meilin_grid_origin` | `[1.2, 1.2]` | (0,0) 格子中心世界坐标 |
| `meilin_grasp_distance` | `0.4` | 抓取时车身距格子边线距离 (m) |
| `meilin_yaw_tolerance` | `0.12` | yaw 校验容差 (rad) |
| `meilin_height_tolerance` | `1.0` | 高度差判断阈值 (mm) |
| `meilin_initial_grid` | `[0, 0]` | 初始位置 grid index |
| `meilin_initial_height` | `0.0` | 初始平台高度 (mm) |
| `meilin_initial_yaw` | `0.0` | 初始朝向 (rad) |
| `meilin_default_align_timeout` | `30.0` | Align 超时 (s) |
| `meilin_default_suspension_timeout` | `10.0` | SuspensionControl 超时 (s) |
| `meilin_default_grasp_timeout` | `30.0` | ArmAction 超时 (s) |

### 运行时状态（blackboard）

Move/Fetch 执行过程中更新以下 blackboard 字段，供后续节点使用：

| 字段 | 类型 | 更新者 | 说明 |
|------|------|--------|------|
| `meilin_current_row` | int | Move | 当前所在行 |
| `meilin_current_col` | int | Move | 当前所在列 |
| `meilin_current_height` | double | Move | 当前平台高度 (mm) |
| `meilin_current_yaw` | double | Move, Fetch | 当前朝向 (rad) |
| `meilin_pose_is_cell_center` | bool | Move → true, Fetch → false | 是否在格子中心 |

### 启动方式

```bash
# 梅林区完整仿真（BT引擎 + 全部硬件 Action Server）
ros2 launch r2_bringup r2_meilin_sim.launch.py

# 红方梅林区
ros2 launch r2_bringup r2_meilin_sim.launch.py is_red_zone:=true

# 仅 BT 引擎（无硬件仿真，用于联调）
ros2 launch r2_bt meilin_stage.launch.py

# 全场仿真（准备区→梅林区→竞技区）
ros2 launch r2_bringup r2_sim.launch.py match_config:=config/match_blue.json

# 全场实车
ros2 launch r2_bringup r2_full.launch.py
```

发布测试序列：

```bash
# 发 move 到 (2,1)，yaw=0，高度 400mm
ros2 topic pub --once /mf_action_seq std_msgs/msg/Float32MultiArray "{data: [0,2,1,400,0,0,0,0]}"

# 发 fetch KFS (3,1)，高度差 +200mm（需上升），yaw=π/2
ros2 topic pub --once /mf_action_seq std_msgs/msg/Float32MultiArray "{data: [1,3,1,200,1.57,0,0,0]}"
```

## 当前 Segment 架构

BT 引擎同时订阅两个 topic：

```text
/planning/segments        std_msgs/String          ← 准备区/竞技区（JSON，旧路径）
/mf_action_seq            std_msgs/Float32MultiArray ← 梅林区（直接解析）
/transformed/pose         geometry_msgs/PoseStamped ← map 系机器人中心/base_link 定位
```

旧 JSON 路径支持以下 `segment_type`：

```text
SPEAR_PREP, SPEAR_GRASP, ALIGN, DOCK, MOVE2, GRASP,
CLIMB, STORE1, STORE2, EXIT, FINAL_MOVE2, PLACE_MID, PLACE_HIGH, FINISH
```

梅林区使用新路径：`PopNextMeilinSegment` → `SwitchSegmentType` → `Move` / `Fetch`（遇 `PLAN_DONE` 退出）。

BT 不重排、不重规划、不判断路线是否合法，只做字段校验、入队和执行调度。

## 三个子树

当前默认树：`r2_bt/trees/full_match.xml`

| 子树 | 覆盖流程 | 调度方式 |
|------|----------|----------|
| `PrepareArea` | 准备区拿取矛头、精调对齐、与 R1 对接 | 固定参数（match_config），直接读 blackboard |
| `MeilinArea` | 梅林区格间移动、上下台阶、抓取 KFS | `ForEachSegment` → `PopNextMeilinSegment` → `Move`/`Fetch` |
| `FinalArea` | 竞技区中层放置 KFS、与 R1 合体放上层、安全收尾 | 固定参数（match_config），直接读 blackboard |

## BT 节点

| 节点 | 类型 | 作用 |
|------|------|------|
| `PopNextMeilinSegment` | `StatefulActionNode` | 等待并弹出下一个梅林 segment，写入 planner 原始值 |
| `PopNextSegment` | `StatefulActionNode` | 等待并弹出下一个 JSON segment（旧路径） |
| `Move` | `StatefulActionNode` | 梅林区格间移动：grid→world、爬台、pre_align、调用 Align/SuspensionControl |
| `Fetch` | `StatefulActionNode` | 梅林区抓取：KFS→抓取位姿、悬挂调整、调用 Align/SuspensionControl/ArmAction |
| `Align` | `StatefulActionNode` | 底盘精调（短距离直线运动，低速高精度） |
| `SwitchSegmentType` | `ControlNode` | 根据 `segment_type` 分派到对应子节点 |
| `ForEachSegment` | `DecoratorNode` | 循环执行子节点直到 `segment_type == stop_type` |
| `IsSegmentType` | `ConditionNode` | 判断当前 segment 类型 |
| `IsStringEmpty` / `IsStringNonEmpty` | `ConditionNode` | 判断可选字段是否存在 |
| `WaitArmIdle` | `StatefulActionNode` | 等后台机械臂动作结束，并检查结果 |
| `MoveToPose` | `StatefulActionNode` | 异步调用 `move_to_pose`（区域间导航） |
| `SuspensionControl` | `StatefulActionNode` | 异步调用 `suspension_control` |
| `ArmAction` | `StatefulActionNode` | 异步调用 `arm_action` |
| `SpearAction` | `StatefulActionNode` | 异步调用 `spear_action` |
| `RetrySegment` | `DecoratorNode` | segment 失败重试，并维护 `retry_count` |
| `SetDebugStatus` | `SyncActionNode` | 写入 Groot2 可见的调试字段 |

## Groot2 调试字段

黑板维护以下字段，Groot2 连接后可观察：

```text
plan_id
current_segment_index
segment_type
segment_debug_name
active_action
retry_count
last_error
execution_state
```

`execution_state` 取值：

```text
WAITING_PLAN
WAITING_MF_ACTION_SEQ
MF_PLAN_READY
SEGMENT_LOADED
ACTION_RUNNING
ACTION_SUCCESS
ACTION_FAILED
MISSION_SUCCESS
MISSION_FAILED
PLAN_REJECTED
CONFIG_LOADED
CONFIG_ERROR
```

连接方式：

```text
Groot2 -> localhost:1667
```

## Action 接口

| Action | 默认服务名 | 说明 |
|--------|------------|------|
| `MoveToPose` | `move_to_pose` | 底盘运动到目标位姿（区域间导航） |
| `Align` | `align` | 底盘精调（区域内短距离直线运动，低速高精度） |
| `SuspensionControl` | `suspension_control` | 四轮独立悬挂上下台阶、恢复行驶高度 |
| `ArmAction` | `arm_action` | 机械臂语义控制：抓取、转存、放置、安全位，支持后台执行 |
| `SpearAction` | `spear_action` | 矛头机构控制：准备、抓取、伸出、释放 |

所有 Action 均使用 ROS 2 Action 协议（`rclcpp_action` / `rclpy.action`），BT 层通过 `StatefulActionNode` 异步调用，硬件层通过 `ActionServer` 执行。

### Action 实现状态

| Action | .action 定义 | BT 节点 (C++) | Action Server (Python) | 硬件 topic |
|--------|:--:|:--:|:--:|------|
| `MoveToPose` | ✅ | ✅ | ✅ | `/t0x0101` |
| `Align` | ✅ | ✅ | ✅ | `/t0x0101` |
| `SuspensionControl` | ✅ | ✅ | ✅ | `/t0x0102_action` |
| `ArmAction` | ✅ | ✅ | ✅ | `/t0x0103_` |
| `SpearAction` | ✅ | ✅ | ✅ | `/t0x0104_` |

## JSON 示例（准备区/竞技区旧路径）

```json
{
  "stage": "FULL_MATCH",
  "plan_id": "full_demo_001",
  "segments": [
    {
      "segment_type": "SPEAR_PREP",
      "move_target": {"x": 0.4, "y": 0.0, "yaw": 0.0},
      "max_speed": 0.4,
      "timeout_sec": 30.0
    },
    {
      "segment_type": "SPEAR_GRASP",
      "timeout_sec": 5.0
    },
    {
      "segment_type": "ALIGN",
      "move_target": {"x": 0.8, "y": 0.0, "yaw": 0.0},
      "max_speed": 0.4,
      "timeout_sec": 30.0
    },
    {
      "segment_type": "DOCK",
      "timeout_sec": 10.0
    },
    {
      "segment_type": "MOVE2",
      "move_target": {"x": 1.2, "y": 0.8, "yaw": 1.57},
      "max_speed": 0.5,
      "timeout_sec": 20.0
    },
    {
      "segment_type": "GRASP",
      "height_diff": 0
    },
    {
      "segment_type": "STORE1",
      "wait_result": false
    },
    {
      "segment_type": "MOVE2",
      "move_target": {"x": 1.8, "y": 0.8, "yaw": 0.0}
    },
    {
      "segment_type": "CLIMB",
      "climb_mode": "UP",
      "climb_direction": "FORWARD",
      "climb_height": 200,
      "move_target": {"x": 2.2, "y": 0.8, "yaw": 0.0},
      "max_speed": 0.4,
      "timeout_sec": 30.0,
      "arm_command": "store_to_body",
      "wait_result": true
    },
    {
      "segment_type": "EXIT",
      "move_target": {"x": 3.0, "y": 0.8, "yaw": 0.0}
    },
    {
      "segment_type": "FINAL_MOVE2",
      "move_target": {"x": 3.4, "y": 1.0, "yaw": 0.0}
    },
    {
      "segment_type": "PLACE_MID",
      "kfs_id": "KFS_A"
    },
    {
      "segment_type": "PLACE_HIGH",
      "kfs_id": "KFS_A"
    },
    {
      "segment_type": "FINISH"
    }
  ]
}
```

快速发布示例：

```bash
ros2 topic pub --once /planning/segments std_msgs/msg/String "{data: '{\"stage\":\"MEILIN\",\"plan_id\":\"demo_001\",\"segments\":[{\"segment_type\":\"MOVE2\",\"move_target\":{\"x\":1.0,\"y\":0.0,\"yaw\":0.0}}]}'}"
```

## 构建

依赖：

- Ubuntu 22.04
- ROS 2 Humble
- BehaviorTree.CPP v4
- `nlohmann_json`
- libusb-1.0
- Python 3.10+

构建：

```bash
cd robocon26_r2_core
colcon build --symlink-install
source install/setup.bash
```

只构建核心包：

```bash
colcon build --symlink-install --packages-select r2_interfaces r2_bt r2_hardware
source install/setup.bash
```

## 运行

### 梅林区专用启动（推荐调试梅林区时使用）

```bash
# 梅林区完整仿真（BT引擎 + 硬件，一键启动）
ros2 launch r2_bringup r2_meilin_sim.launch.py

# 红方梅林区仿真
ros2 launch r2_bringup r2_meilin_sim.launch.py is_red_zone:=true

# 仅 BT 引擎（不启动硬件仿真）
ros2 launch r2_bt meilin_stage.launch.py
```

### 全场启动

```bash
# 全场仿真（准备区→梅林区→竞技区）
ros2 launch r2_bringup r2_sim.launch.py match_config:=config/match_blue.json

# 全场实车
ros2 launch r2_bringup r2_full.launch.py
```

### 仅 BT 引擎

```bash
ros2 launch r2_bt bt_engine.launch.py
```

单独调试梅林区树时可以把参数 `tree_file` 改为 `meilin_stage.xml`。

## USB 数据通道

`ares_usb` 将 ROS 2 topic 与下位机 DataID 做透传映射：

多开发板按 USB PID 与 DataID 高字节分流，不广播到所有开发板：PID `0x0001` 只处理 `0x01xx`，PID `0x0002` 只处理 `0x02xx`。

| Topic / DataID | 方向 | 说明 |
|----------------|------|------|
| `/t0x0101` | ROS → 下位机 | 底盘运动指令 `[vx, vy, vyaw]` |
| `/t0x0102_action` | ROS → 下位机 | 四轮悬挂目标高度 `[h0, h1, h2, h3]` |
| `/t0x0103_` | ROS → 下位机 | 机械臂控制指令 |
| `/t0x0104_` | ROS → 下位机 | 矛头机构控制指令 |
| `/r0x0201` | 下位机 → ROS | 底盘速度反馈 |
| `/r0x0202` | 下位机 → ROS | 光电开关 + 轮高反馈 |
| `/r0x0203` | 下位机 → ROS | 机械臂状态反馈 |
| `/r0x0204` | 下位机 → ROS | 矛头机构状态反馈 |
| `/sensor_distances` | 传感器 → ROS | 8 路 TOF 距离 |
| `/odom_world` | 定位 → ROS | 世界坐标位姿 |

## 开发原则

1. `r2_bt` 只调度语义 Action，不直接控制电机、串口、GPIO 或 PWM。
2. 外部规划模块负责路线合法性，BT 按已规划 segment 顺序执行。
3. 行为树固定加载，运行时只更新 `segment_queue` 和黑板字段。
4. Tree XML 只暴露 planner 原始值（row/col/height/yaw），计算逻辑封装在 Move/Fetch 内部。
5. Condition 节点必须瞬间返回；耗时工作使用 `StatefulActionNode`。
6. Action Client 使用异步发送、异步结果回调和异步取消。
7. 失败日志和 Groot2 黑板必须包含当前 segment、Action 和错误原因。
