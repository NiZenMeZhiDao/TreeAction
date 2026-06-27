# 本仓库负责内容

1. 本仓库是"规划结果到机器人动作"的中间件：接收外部梅林规划结果，翻译成 segment，再动态调用语义 Action（BT.CPP + Groot2 实现）。
2. 实现 suspension action：四轮独立悬挂升降，配合底盘运动完成上下台阶。
3. 底层 usb 和 sensor 等沿用已有方案不变。

以上都是全局坐标系。

**说明**：本文档描述目标设计架构。部分 Action（如独立 `Align`）尚未完全实现，当前实现状态详见 [README.md](README.md) 的"Action 实现状态"表格。README 中的 segment 命名（`STORE1`/`STORE2`、`PLACE_HIGH`）与本设计文档统一后对应实际代码。

# segment 分类说明

全区域共两类底盘移动 **Action**，控制逻辑完全不同：

| Action | 类型 | 用途 | 特点 |
|--------|------|------|------|
| `MoveToPose` | **导航** | 区域间切换（准备→梅林、梅林→竞技） | 长距离、路径规划、常规速度 |
| `Align` | **精调** | 区域内小段直线运动（格间移动、抓取前对准、对接前微调、槽位精调） | 短距离、直线轨迹、低速高精度 |

导航和精调是两套独立的底盘控制逻辑，不可混用。区域切换走导航，其余所有底盘移动走精调。

# 准备区

比赛开始后的第一个阶段。R2 从启动区出发，拿取矛头，然后与 R1 对接。全程串行执行，各 segment 之间不允许并行。

准备区使用的 segment：

- `SPEAR_PREP`
- `SPEAR_GRASP`
- `ALIGN`
- `DOCK`

## 1. SPEAR_PREP

矛头机构进入准备状态，同时底盘精调对准矛头。

- Action：`SpearAction(command=prepare)` + `Align`
- 参数：
  - `align_target { x, y, yaw }`（全局坐标）
  - `max_speed` 限制为低速档
- 流程：
  1. `SpearAction(command=prepare)`：张开夹爪、解锁机构、移动到待抓取位姿
  2. `Align`：底盘精调对准矛头，使抓取机构正对目标
- 目标位姿由视觉定位模块提供
- 到达后底盘锁定，不响应外部扰动
- 硬同步点，必须等待完成后才能进入下一步
- 完成后矛头机构处于可抓取状态

## 2. SPEAR_GRASP

专用机构抓取矛头并锁紧。

- Action：`SpearAction(command=grasp)`
- 参数：
  - `timeout_sec`：抓取超时（默认 5s）
- 前置条件：`SPEAR_PREP` 完成
- 抓取流程：
  1. 夹爪闭合抓取矛头
  2. 检测抓取力 / 到位传感器确认抓稳
  3. 锁紧机构锁定矛头
- 硬同步点，不能和底盘移动并行
- 抓取完成后矛头固定在车体上，机械臂回到安全位
- 抓取失败（超时或传感器未确认）时进入异常处理

## 3. ALIGN

抓取矛头后精调位置，与 R1 对接位对齐。

- Action：`Align`
- 参数：
  - `align_target { x, y, yaw }`（全局坐标）
  - `max_speed` 限制为低速档
- 目标位姿由对接位标定值给出，无需视觉
- 到达后底盘锁定

## 4. DOCK

与 R1 对接，将矛头传递给 R1。对接完成后 R2 进入梅林区流程。

- Action：`SpearAction(command=dock_extend)` + `SpearAction(command=dock_release)`
- 参数：
  - `dock_timeout_sec`：对接总超时
- 前置条件：接收到 R1 到位信号
- BT 用 `Sequence` 串行执行：
  1. 等待 R1 到位信号（`Condition` 节点轮询 IO 信号）
  2. 伸出阶段：`SpearAction(command=dock_extend)` — 机械臂将矛头送到对接位
  3. 释放阶段：`SpearAction(command=dock_release)` — 机械臂松开矛头并收回
  4. 机械臂回到安全位
- 与 `SPEAR_GRASP` 共用同一套机构（`SpearAction`）
- 对接完成后，R2 立即进入梅林区，开始执行梅林动作序列

# 梅林区

比赛的第二阶段。R2 在梅林区内完成 KFS 的拿取、搬运和放置。

**梅林区的特殊性**：梅林区接收外部路径规划模块的动作序列，本仓库不重新规划、不改写动作顺序，只做"规划结果 → segment"的翻译和执行调度。

## 和梅林路径规划对接

接收动作序列，动作序列为以下类型的组合：

- **移动到哪个格子**：目标全局坐标 `(x, y)`，方向 `yaw`，相对高度落差
- **拿哪个方向的格子上的 KFS**：目标方向，相对高度落差
- **前往哪个坐标**：目标全局坐标 `(x, y)`

## 执行 segment

梅林区内部执行的 segment 只用这几类：

- `MOVE2`
- `GRASP`
- `CLIMB`
- `STORE1`
- `STORE2`

### 1. MOVE2

底盘移动到目标点，机械臂不参与。

- Action：`Align`
- 参数：`align_target { x, y, yaw }`（全局坐标），可选 `max_speed`、`timeout_sec`
- 梅林区内格间移动，使用精调 Action，不走导航

### 2. GRASP

底盘不动根据相差高度完成是否升起下降底盘，机械臂完成抓取。

- Action：`ArmAction(command=grasp)`+`SuspensionControl`
- 前置条件：机械臂处于可以抓取的条件
- 这是硬同步点，不能和底盘移动并行
- 悬挂动作响应很快

### 3. CLIMB

底盘上下台阶，是悬挂升降和底盘运动的协同动作。

- Action：`SuspensionControl` + `MoveToPose`
- 参数：`climb_mode`（要上台阶的高度），要前往的绝对坐标
- BT 用 `Parallel` 同时执行悬挂和底盘，同步关系由行为树协调两者的完成条件
- `CLIMB` 期间允许和安全的 `ArmAction` 并行（如 `store`），但 `GRASP` 不能和 `CLIMB` 并行
- 外部规划模块保证目标台阶合法

### 4. STORE1

机械臂从抓取后位置，把 KFS 转存到车体储存位，然后回到空闲状态。

- Action：`ArmAction(command=store_to_body)`
- 可以设置 `wait_result=false`，后台执行，后续 `MOVE2` 可以先走

### 5. STORE2

机械臂从抓取后位置，把 KFS 暂存在手臂上，然后回到空闲状态。

- Action：`ArmAction(command=store_on_arm)`
- 可以设置 `wait_result=false`，后台执行，后续 `MOVE2` 可以先走

## Action（梅林区）

| Action | 作用 |
|--------|------|
| `MoveToPose` | 区域间导航 |
| `Align` | 区域内精调移动 |
| `SuspensionControl` | 底盘上下台阶、悬挂恢复 |
| `ArmAction` | 机械臂准备、抓取、转存、放置、安全位 |

# 竞技区

比赛的最后阶段。R2 与 R1 协同完成中层和上层 KFS 的放置。

竞技区使用的 segment：

- `MOVE2`
- `FINAL_MOVE2`
- `PLACE_MID`
- `PLACE_HIGH`
- `FINISH`

## 1. MOVE2

从准备区导航到梅林区入口。底盘使用精调模式对准入口方向。

- Action：`Align`
- 参数：
  - `move_target { x, y, yaw }`（全局坐标）
  - `max_speed` 限制为低速档
- 这是区域内精调移动

## 2. FINAL_MOVE2

从梅林区导航到竞技区。底盘抬起以获得更好的通过性。

- Action：`SuspensionControl` + `MoveToPose`
- 参数：
  - `move_target { x, y, yaw }`（全局坐标）
- 这是区域间切换，使用导航 Action `MoveToPose`，走路径规划
- 额外增加悬挂抬升以提升离地间隙
- 到达竞技区后悬挂恢复至行驶高度

## 3. PLACE_MID

放置中层 KFS。根据视觉识别 R1 显示器显示的结果，选择放置中层左、中、右三个槽位之一。

- Action：`SuspensionControl` + `Align` + `ArmAction(command=place)`
- 参数：
  - `slot { left | mid | right }`：目标槽位，由视觉模块识别 R1 显示器后传入
  - `height_level: middle`
- BT 用 `Sequence` 串行执行：
  1. `SuspensionControl(target_height=middle)`：调整四轮悬挂至中层高度。悬挂需在移动开始前就位，避免运动中切换高度
  2. `Align`：精调到具体槽位（小段直线运动），根据 `slot` 参数决定偏移量
  3. `ArmAction(command=place)`：放置 KFS 到目标槽位
  4. `ArmAction(command=go_safe)`：机械臂回到安全位
- 视觉识别在段开始前完成，识别结果（R1 显示器显示的数字 / 颜色 / 图案）映射为 `slot` 参数
- `PLACE_MID` 期间悬挂处于中层高度，`GRASP` 不能和 `PLACE_MID` 并行（悬挂非标准高度下抓取不安全）
- 下一步取决于 R1 的信号

## 4. PLACE_HIGH

R2 底盘升到最高，R1 抬起 R2 后，R2 放置上层 KFS。此段 R2 底盘不主动移动，整体位置由 R1 控制。

- Action：`SuspensionControl` + `ArmAction(command=place_high)`
- 参数：
  - `height_level: upper`
- 前置条件：R1 已抬起 R2，R1 通过接触式信号通知 R2 放置
- BT 用 `Sequence`：
  1. 等待 R1 接触式信号（`Condition` 节点轮询 IO / 接触传感器）
  2. `SuspensionControl(target_height=upper)`：调整悬挂至上层高度
  3. `ArmAction(command=place_high)`：放置 KFS 到上层槽位
  4. `ArmAction(command=go_safe)`：机械臂回到安全位
- 放置完成后：
  - `SuspensionControl(target_height=normal)`：悬挂恢复到行驶高度
  - 通知 R1 放置完成（IO 信号），等待 R1 放下 R2
- 此段期间 R2 底盘不主动移动；如果 R1 移动导致 R2 位置偏移，由 R1 负责补偿
- 上层没有左中右选择，由 R1 移动选择，因此不需要视觉判断

## 5. FINISH

比赛结束，R2 恢复到安全状态。

- Action：`ArmAction(command=idle)` + `SuspensionControl(mode=RECOVER)`
- 机械臂停止，悬挂恢复到行驶高度
- 等待外部终止信号

# Action（全区域汇总）

| Action | 作用 | 使用区域 |
|--------|------|----------|
| `MoveToPose` | **导航**：区域间切换，长距离路径规划 | 梅林、竞技 |
| `Align` | **精调**：区域内小段直线运动，低速高精度 | 梅林、准备、竞技 |
| `SuspensionControl` | 四轮独立悬挂升降、恢复行驶高度 | 梅林、竞技 |
| `ArmAction` | 机械臂控制：抓取、转存、放置、安全位 | 梅林、竞技 |
| `SpearAction` | 矛头机构控制：准备、抓取、伸出、释放 | 准备 |


当前要实现 suspension、arm 和 spear 的 action。

# 异常处理与失败语义

本仓库是执行层，不负责路线规划和决策，因此所有 Action 失败后的恢复策略相对保守。外部规划模块负责路线合法性，BT 按已规划 segment 顺序执行。

## 失败后行为

| Action 类型 | 失败后重试 | 最终失败后果 |
|-------------|-----------|-------------|
| 底盘导航 (`MoveToPose`) | `RetrySegment` 最多重试 2 次 | 当前 segment 失败 → `MISSION_FAILED` |
| 底盘精调 (`Align`) | `RetrySegment` 最多重试 2 次 | 当前 segment 失败 → `MISSION_FAILED` |
| 悬挂动作 (`SuspensionControl`) | `RetrySegment` 最多重试 2 次，CLIMB 三次失败后尝试 `RECOVER` 模式 | CLIMB 段失败 → `MISSION_FAILED`，其他段回退悬挂后继续 |
| 机械臂动作 (`ArmAction`) | `RetrySegment` 最多重试 2 次 | 当前 segment 失败 → `MISSION_FAILED` |
| 矛头机构 (`SpearAction`) | `RetrySegment` 最多重试 2 次 | 准备区段失败 → `MISSION_FAILED`（比赛无法继续） |

## 特殊场景

- **`SPEAR_GRASP` 失败**：准备区无法继续，直接 `MISSION_FAILED`。无 fallback 方案，因为比赛必须拿到矛头。
- **`CLIMB` 失败**：悬挂尝试恢复到正常行驶高度，然后 segment 标记失败。外部规划模块需重新规划路线。
- **`segment_queue` 为空但未收到 `FINISH`**：BT 引擎保持在 `WAITING_PLAN` 状态，等待新的 segment 计划。如果比赛时间耗尽，外部裁判系统会终止。
- **机械臂后台动作失败**：`WaitArmIdle` 会检测并报告错误，后续依赖该动作的 segment 会失败。

## 安全机制

- 所有 Action 都有 `timeout_sec` 参数，防止无限等待。
- 底盘运动异常（如堵转、定位丢失）由 Action Server 内部检测并返回失败。
- 悬挂超时或光电开关异常时，`SuspensionControl` 会尝试恢复到安全高度后返回失败。