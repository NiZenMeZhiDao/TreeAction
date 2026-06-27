# 梅林区 BehaviorTree 执行方案

## 1. 核心思路

不单独做 translator 节点。BT 直接订阅 `/mf_action_seq`，把 planner 给出的 `move/fetch` 解析成梅林区内部 segment，按顺序执行。

```text
mf_action_planner -> /mf_action_seq -> Meilin BT / MeilinExecutor -> Align / SuspensionControl / ArmAction
```

第一版只做两类 segment：

| segment | 职责 |
| --- | --- |
| `move` | 移动到目标格子，必要时上下台阶 |
| `fetch` | 到 KFS 抓取位，调整高度，抓取 |

`move/fetch` 都可以在内部调用多个 action。BT 外层只需要顺序执行这些 segment。

## 2. 输入接口

Topic：

```text
/mf_action_seq
std_msgs/msg/Float32MultiArray
```

每个动作 8 个 float：

```text
[arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7]
```

`move`：

```text
arg0 = 0
arg1 = row
arg2 = col
arg3 = target_height_mm
arg4 = yaw
```

`fetch`：

```text
arg0 = 1
arg1 = kfs_row
arg2 = kfs_col
arg3 = height_diff_mm
arg4 = yaw
```

`arg5~arg7` 暂不使用。

## 3. 配置和状态

固定信息放配置，不跟着动作发。

```yaml
meilin:
  side: blue
  grid_size: 1.2
  grid_origin: [1.2, 1.2]
  initial_grid: [0, 0]
  initial_height: 0

meilin_bt:
  retry_count: 3
  default_align_timeout: 30.0
  default_suspension_timeout: 10.0
  default_grasp_timeout: 30.0
```

还需要配置：

- `height_map_blue / height_map_red`：红蓝方高度表。
- `fetch_pose_table`：`forward/left/right` 对应抓取偏移和 yaw。

blackboard 放当前 segment 展开后的值：

```text
segment_type
target_row / target_col / target_x / target_y / target_yaw
target_height / height_diff / climb_direction
grasp_x / grasp_y / grasp_yaw
```

执行节点维护轻量状态：

```text
current_row / current_col / current_height / current_yaw / pose_is_cell_center
```

这些状态只做格子逻辑判断，不替代真实定位。梅林 `move` 优先从 `/transformed/pose`
读取 `geometry_msgs/PoseStamped`，该话题表示 map 坐标系下机器人中心/base_link 位姿；
定位未收到或超时时才降级使用上述状态。action 成功后更新状态，失败不更新。

## 4. move 执行

输入：`row, col, target_height, yaw`

流程：

```text
target_pose = grid_to_world(row, col, yaw)
current_pose = /transformed/pose
current_row/current_col = world_to_nearest_grid(current_pose.x, current_pose.y)
current_yaw = current_pose.yaw
pose_is_cell_center = distance_to_grid_center <= cell_center_tolerance
height_diff = target_height - current_height

如果当前不在格子中心:
  判断当前偏移是否正好沿下一步前进方向
  如果是，可以直接接着走
  如果不是，先回到当前格子中心

如果 height_diff == 0:
  Align(target_pose)
否则:
  对齐上下台阶方向
  调用上下台阶 action
  Align(target_pose)

到达后转到目标 yaw
```

成功后更新：

```text
current_row = row
current_col = col
current_height = target_height
current_yaw = yaw
pose_is_cell_center = true
```

限制：有高度变化时目标格必须和当前格相邻；不支持一次 `move` 跨多个台阶；平动和转向分开做。

## 5. fetch 执行

输入：`height_diff, yaw`

`kfs_row/kfs_col` 由 `ParseCurrentAction` 解析后用于查抓取位，查到的 `grasp_pose` 写入 blackboard。

流程：

```text
校验 yaw 是否和 grasp_pose.yaw 一致

在当前格子中心转到 grasp_pose.yaw
直线移动到 grasp_pose

如果 height_diff != 0:
  SuspensionControl(height_adjust, height_diff)  # 具体高度待定

ArmAction(GRASP)
```

抓取和收纳由下位机抓取流程完成，BT 不单独发 `STORE`。

第一版抓完不强制退回格子中心，由下一条 `move` 判断是否需要回中心，或者能否顺方向直接走。

限制：`kfs_grid` 必须和 `current_grid` 相邻；只支持 `forward/left/right`；车身必须正对 KFS。

## 6. BT 结构

可以拆成：

```text
WaitMfActionSeq
ForEachRawAction
  ParseCurrentAction
  Switch(segment_type)
    move  -> ExecuteMeilinMove
    fetch -> ExecuteMeilinFetch
```

也可以第一版直接做一个 `MeilinExecutor` 节点，内部循环执行 segment。先简单可靠。

## 7. 校验规则

收到 `/mf_action_seq` 后先校验：

```text
数组长度是 8 的倍数
arg0 只能是 0 或 1
row 范围 0~5
col 范围 0~2
yaw 在允许方向附近
```

`move` 额外校验：`target_height` 合法、和高度表一致；如果 `height_diff != 0`，目标格和当前格相邻。

`fetch` 额外校验：KFS 格和当前格相邻；能查到抓取位；planner 给的 yaw 和抓取位 yaw 一致。

校验失败时拒绝整条 plan，写 `last_error`，进入失败状态。

## 8. 暂不做

- 不做单独 translator 节点。
- 不生成中间 JSON segment。
- 不在动作消息里传速度和超时。
- 不支持一次 `move` 跨多个台阶。
- 不做复杂恢复动作。
