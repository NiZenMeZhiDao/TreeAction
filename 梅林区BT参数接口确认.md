# 梅林区 BT 参数接口确认表

这份文档只用于确认接口，不作为最终实现说明。

你可以直接改表格里的“是否保留 / 最终名称 / 来源 / 备注”。我后面按这份文档改代码和 tree。

## 1. 外部输入 `/mf_action_seq`

Topic:

```text
/mf_action_seq
std_msgs/msg/Float32MultiArray
```

每个动作 8 个 float:

```text
[arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7]
```

### 1.1 move 输入

| arg | 当前理解 | 单位 | 是否保留 | 最终名称 | 备注 |
| --- | --- | --- | --- | --- | --- |
| arg0 | 动作类型，0 表示 move | - | 是 | action_type | 只用于解析，不进 tree |
| arg1 | 目标格 row | grid index | 待确认 | move_row | 是否需要进 tree，还是只在解析阶段换成 x |
| arg2 | 目标格 col | grid index | 待确认 | move_col | 是否需要进 tree，还是只在解析阶段换成 y |
| arg3 | 目标高度 | mm | 待确认 | target_height_mm | 是否由 planner 给，还是完全查 height_map |
| arg4 | 目标 yaw | rad | 待确认 | target_yaw | 是否只允许 0/90/180/-90 |
| arg5 | 未使用 | - | 否 | - | 预留 |
| arg6 | 未使用 | - | 否 | - | 预留 |
| arg7 | 未使用 | - | 否 | - | 预留 |

### 1.2 fetch 输入

| arg | 当前理解 | 单位 | 是否保留 | 最终名称 | 备注 |
| --- | --- | --- | --- | --- | --- |
| arg0 | 动作类型，1 表示 fetch | - | 是 | action_type | 只用于解析，不进 tree |
| arg1 | KFS row | grid index | 待确认 | kfs_row | 是否需要进 tree，还是只用于查抓取位 |
| arg2 | KFS col | grid index | 待确认 | kfs_col | 是否需要进 tree，还是只用于查抓取位 |
| arg3 | 抓取前升降高度差 | mm | 待确认 | height_diff_mm | 正负方向是否由正负号表示 |
| arg4 | 抓取 yaw | rad | 待确认 | target_yaw | 是否必须和 KFS 方位一致 |
| arg5 | 未使用 | - | 否 | - | 预留 |
| arg6 | 未使用 | - | 否 | - | 预留 |
| arg7 | 未使用 | - | 否 | - | 预留 |

## 2. 配置参数

这些不建议跟每个动作一起发，放配置或节点参数。

| 参数 | 当前理解 | 单位 | 是否保留 | 来源 | 备注 |
| --- | --- | --- | --- | --- | --- |
| meilin_side | 红蓝方 | - | 是 | 参数 / match_config | 影响高度表和坐标 |
| grid_size | 格子尺寸 | m | 是 | 参数 / config | grid -> world |
| grid_origin | 梅林区原点 | m | 是 | 参数 / config | grid -> world |
| initial_grid | 初始格 | grid index | 是 | 参数 / config | 内部状态初值 |
| initial_height | 初始高度 | mm | 是 | 参数 / config | 内部状态初值 |
| initial_yaw | 初始 yaw | rad | 是 | 参数 / config | 内部状态初值 |
| height_map_blue | 蓝方高度表 | mm | 是 | config | 待确认具体表 |
| height_map_red | 红方高度表 | mm | 是 | config | 待确认具体表 |
| grasp_distance | 抓取位离 KFS 的距离 | m | 待确认 | config | 当前实现用 grid_size/2 + grasp_distance |
| yaw_tolerance | yaw 校验误差 | rad | 待确认 | 参数 | 只用于校验 |
| height_tolerance | 高度误差 | mm | 待确认 | 参数 | 只用于校验 |
| default_align_timeout | 对齐超时 | s | 待确认 | 参数 | 传给 Align |
| default_suspension_timeout | 悬挂超时 | s | 待确认 | 参数 | 传给 SuspensionControl |
| default_grasp_timeout | 抓取超时 | s | 待确认 | 参数 | 传给 ArmAction |

## 3. tree 里需要显式出现的参数

### 3.1 `PopNextMeilinSegment` 输出

这个节点从 `/mf_action_seq` 解析后的队列里取一个动作，并把后续节点要用的参数写到 blackboard。

| 输出参数 | 当前用途 | 是否保留 | 备注 |
| --- | --- | --- | --- |
| segment_type | 区分 move / fetch / PLAN_DONE | 是 | 给 SwitchSegmentType 用 |
| target_x | Align 目标 x | 待确认 | 是否用 x/y，还是保留 row/col 到 Move 内部算 |
| target_y | Align 目标 y | 待确认 | 同上 |
| target_yaw | Align 目标 yaw | 待确认 | rad |
| timeout_sec | Align / Arm 超时 | 待确认 | move 和 fetch 是否分开命名 |no
| suspension_timeout_sec | 悬挂超时 | 待确认 | 只给 SuspensionControl |no
| pre_align_required | move 前是否先回当前格中心 | 待确认 | 只用于 move |no
| pre_align_x | 回中心目标 x | 待确认 | 只用于 move |no
| pre_align_y | 回中心目标 y | 待确认 | 只用于 move |no
| pre_align_yaw | 回中心 yaw | 待确认 | 只用于 move |no
| climb_required | move 是否需要上下台阶 | 待确认 | 只用于 move |no
| climb_mode | 悬挂模式，上/下/恢复 | 待确认 | 只用于 move |no
| climb_direction | 上下台阶方向 | 待确认 | 只用于 move |
| climb_height | 台阶高度差绝对值 | 待确认 | 只用于 move |
| height_diff | fetch 高度差，带正负 | 待确认 | 只用于 fetch 判断是否需要升降 |
| abs_height_diff | fetch 高度差绝对值 | 待确认 | 只用于发 SuspensionControl |no
| grasp_suspension_mode | fetch 升/降模式 | 待确认 | 只用于 fetch |no
| arm_command | 机械臂命令 | 待确认 | fetch 当前理解是 GRASP |no

### 3.2 `Move` 输入

tree 当前预期写成：

```xml
<Move target_x="{target_x}"
      target_y="{target_y}"
      target_yaw="{target_yaw}"
      timeout_sec="{timeout_sec}"
      suspension_timeout_sec="{suspension_timeout_sec}"
      pre_align_required="{pre_align_required}"
      pre_align_x="{pre_align_x}"
      pre_align_y="{pre_align_y}"
      pre_align_yaw="{pre_align_yaw}"
      climb_required="{climb_required}"
      climb_mode="{climb_mode}"
      climb_direction="{climb_direction}"
      climb_height="{climb_height}"
      message="{last_error}"/>
```

| 输入参数 | 是否必须 | 来源 | 最终是否保留 | 备注 |
| --- | --- | --- | --- | --- |
| target_x | 是 | grid_to_world(move_row, move_col) | 待确认 | 发给 Align |
| target_y | 是 | grid_to_world(move_row, move_col) | 待确认 | 发给 Align |
| target_yaw | 是 | `/mf_action_seq.arg4` | 待确认 | 发给 Align |
| timeout_sec | 否 | default_align_timeout | 待确认 | Align 超时 |no
| suspension_timeout_sec | 否 | default_suspension_timeout | 待确认 | SuspensionControl 超时 |no
| pre_align_required | 否 | 内部状态判断 | 待确认 | 不在格子中心时是否先回中心 |no
| pre_align_x | 否 | 当前格中心 x | 待确认 | pre_align_required=true 时用 |no
| pre_align_y | 否 | 当前格中心 y | 待确认 | pre_align_required=true 时用 |no
| pre_align_yaw | 否 | 当前 yaw / 目标方向 | 待确认 | 这里我不确定 |no
| climb_required | 否 | height_diff 判断 | 待确认 | 是否上下台阶 |no
| climb_mode | 否 | height_diff 正负 | 待确认 | 上/下/恢复对应值待确认 |no
| climb_direction | 否 | 当前格 -> 目标格方向 | 待确认 | 方向枚举值待确认 |
| climb_height | 否 | abs(height_diff) | 待确认 | mm |

### 3.3 `Fetch` 输入

tree 当前预期写成：

```xml
<Fetch target_x="{target_x}"
       target_y="{target_y}"
       target_yaw="{target_yaw}"
       timeout_sec="{timeout_sec}"
       suspension_timeout_sec="{suspension_timeout_sec}"
       height_diff="{height_diff}"
       abs_height_diff="{abs_height_diff}"
       grasp_suspension_mode="{grasp_suspension_mode}"
       arm_command="{arm_command}"
       message="{last_error}"/>
```

| 输入参数 | 是否必须 | 来源 | 最终是否保留 | 备注 |
| --- | --- | --- | --- | --- |
| target_x | 是 | KFS + 抓取偏移 | 待确认 | fetch 对齐目标 |
| target_y | 是 | KFS + 抓取偏移 | 待确认 | fetch 对齐目标 |
| target_yaw | 是 | `/mf_action_seq.arg4` 或查表 | 待确认 | 发给 Align |
| timeout_sec | 否 | default_grasp_timeout | 待确认 | Align/Arm 是否共用待确认 |no
| suspension_timeout_sec | 否 | default_suspension_timeout | 待确认 | SuspensionControl 超时 |no
| height_diff | 是 | `/mf_action_seq.arg3` | 待确认 | 判断是否需要升降 |
| abs_height_diff | 否 | abs(height_diff) | 待确认 | 是否应在 Fetch 内部算 |no
| grasp_suspension_mode | 否 | height_diff 正负 | 待确认 | 是否应在 Fetch 内部算 |no
| arm_command | 否 | 固定 GRASP | 待确认 | 是否 tree 传，还是 Fetch 内部固定 |no

## 4. 底层 action 对应关系

### 4.1 Move 内部调用

| 条件 | 调用 | 参数 | 待确认点 |
| --- | --- | --- | --- |
| pre_align_required=true | Align | pre_align_x, pre_align_y, pre_align_yaw, timeout_sec | pre_align_yaw 应该取当前 yaw 还是目标方向 |
| climb_required=true | SuspensionControl | climb_mode, climb_direction, climb_height, suspension_timeout_sec | mode/direction 枚举值 |
| 始终执行 | Align | target_x, target_y, target_yaw, timeout_sec | 是否需要 max_speed |

### 4.2 Fetch 内部调用

| 条件 | 调用 | 参数 | 待确认点 |
| --- | --- | --- | --- |
| 始终执行 | Align | target_x, target_y, target_yaw, timeout_sec | target 是抓取位还是格子中心 |
| height_diff != 0 | SuspensionControl | grasp_suspension_mode, abs_height_diff, suspension_timeout_sec | mode 是否由 height_diff 正负推导 |
| 始终执行 | ArmAction | arm_command, timeout_sec | arm_command 是否固定 GRASP |

## 5. 我现在最拿不准的点

请优先改这里：

1. `Move` 的 tree 参数应该传 `row/col/target_height/yaw`，还是传已经换算好的 `target_x/target_y/target_yaw/climb_*`？`row/col/target_height/yaw`
2. `Fetch` 的 tree 参数应该传 `kfs_row/kfs_col/height_diff/yaw`，还是传已经查表后的 `target_x/target_y/target_yaw`？`kfs_row/kfs_col/height_diff/yaw`
3. `abs_height_diff`、`grasp_suspension_mode` 这种派生参数，要不要出现在 tree，还是由 `Fetch` 内部根据 `height_diff` 算？内部
4. `arm_command` 是否固定为 GRASP？如果固定，是否还需要作为 tree 参数？固定
5. `timeout_sec` 是否需要拆成 `align_timeout_sec`、`suspension_timeout_sec`、`arm_timeout_sec`？要，这些抽到config，不是运行时确定的
6. `pre_align_*` 是否应该暴露在 tree，还是 `Move` 内部根据当前状态自己判断？内部
7. `climb_mode`、`climb_direction` 的枚举值是否和 `SuspensionControl.action` 完全一致？内部解决，先不改变

## 6. 建议确认后的目标

确认后我会按这个目标实现：

```text
/mf_action_seq 只保留 planner 必须给的信息
config 只保留固定地图/尺寸/超时/枚举配置
tree 显式展示 Move / Fetch 真正需要的参数
Move / Fetch 内部只做动作编排，不再重复维护一堆无用中间字段
```
