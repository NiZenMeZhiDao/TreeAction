#pragma once

#include <behaviortree_cpp/actions/pop_from_queue.hpp>

#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace r2_bt
{

/// 梅林区静态配置（启动时从参数加载，放入 blackboard）
struct MeilinConfig
{
  std::string side = "blue";
  double grid_size = 1.2;
  double grid_origin_x = 1.2;
  double grid_origin_y = 1.2;
  double grasp_distance = 0.4;
  double yaw_tolerance = 0.12;
  double height_tolerance = 1.0;
  double align_timeout_sec = 30.0;
  double suspension_timeout_sec = 10.0;
  double arm_timeout_sec = 30.0;
  double suspension_normal_height = 30.0;  // 正常行驶悬挂高度 (mm)，即 H_INIT
  double pose_timeout_sec = 1.0;
  double cell_center_tolerance = 0.15;
  int rows = 6;
  int cols = 3;
};

using MeilinConfigPtr = std::shared_ptr<MeilinConfig>;

/// 高度地图工具函数
inline double meilin_height_at(int row, int col, const std::string& side)
{
  // 蓝方高度地图 (mm)
  static constexpr double blue[6][3] = {
    {0.0, 0.0, 0.0},
    {400.0, 200.0, 400.0},
    {600.0, 400.0, 200.0},
    {400.0, 600.0, 400.0},
    {200.0, 400.0, 200.0},
    {0.0, 0.0, 0.0},
  };
  // 红方高度地图 (仅 row=2 不同)
  static constexpr double red[6][3] = {
    {0.0, 0.0, 0.0},
    {400.0, 200.0, 400.0},
    {200.0, 400.0, 600.0},
    {400.0, 600.0, 400.0},
    {200.0, 400.0, 200.0},
    {0.0, 0.0, 0.0},
  };
  if (row < 0 || row >= 6 || col < 0 || col >= 3) return 0.0;
  const auto& map = (side == "red") ? red : blue;
  return map[row][col];
}

/// 归一化角度到 [-π, π]
inline double meilin_normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// 角度比较
inline bool meilin_yaw_close(double lhs, double rhs, double tolerance)
{
  return std::abs(meilin_normalize_angle(lhs - rhs)) <= tolerance;
}

/// grid → world
inline std::pair<double, double> meilin_grid_to_world(
    int row, int col, const MeilinConfig& cfg)
{
  return {
    cfg.grid_origin_x + static_cast<double>(row) * cfg.grid_size,
    cfg.grid_origin_y + static_cast<double>(col) * cfg.grid_size
  };
}

/// world/map → nearest grid center
inline bool meilin_world_to_grid(
    double x, double y, const MeilinConfig& cfg, int& row, int& col)
{
  if (cfg.grid_size <= 0.0) return false;
  row = static_cast<int>(std::lround((x - cfg.grid_origin_x) / cfg.grid_size));
  col = static_cast<int>(std::lround((y - cfg.grid_origin_y) / cfg.grid_size));
  return row >= 0 && row < cfg.rows && col >= 0 && col < cfg.cols;
}

/// 判断是否相邻
inline bool meilin_adjacent(int row_a, int col_a, int row_b, int col_b)
{
  return std::abs(row_a - row_b) + std::abs(col_a - col_b) == 1;
}

/// 将 yaw 转成悬挂使用的车体相对方向。
/// @return direction_enum(0=FWD,1=LEFT,2=RIGHT,3=BACKWARD)
inline bool meilin_direction_from_yaw(double yaw, int& direction,
                                      std::string& direction_name,
                                      double tolerance = 0.12)
{
  const double normalized = meilin_normalize_angle(yaw);
  if (meilin_yaw_close(normalized, 0.0, tolerance))
  {
    direction = 0;
    direction_name = "forward";
    return true;
  }
  if (meilin_yaw_close(normalized, M_PI / 2.0, tolerance))
  {
    direction = 1;
    direction_name = "left";
    return true;
  }
  if (meilin_yaw_close(normalized, -M_PI / 2.0, tolerance))
  {
    direction = 2;
    direction_name = "right";
    return true;
  }
  if (meilin_yaw_close(normalized, M_PI, tolerance))
  {
    direction = 3;
    direction_name = "backward";
    return true;
  }
  direction_name = "invalid";
  return false;
}

/// 计算移动方向 yaw (从 from → to)，并按 reference_yaw 转成车体相对方向。
/// reference_yaw 通常来自 /mf_action_seq 的 arg4；默认 0 保持旧调用行为。
/// @return {yaw, direction_enum(0=FWD,1=LEFT,2=RIGHT,3=BACKWARD), direction_name}
inline bool meilin_direction_yaw(
    int from_row, int from_col, int to_row, int to_col,
    double& yaw, int& direction, std::string& direction_name,
    double reference_yaw = 0.0)
{
  const int d_row = to_row - from_row;
  const int d_col = to_col - from_col;
  double move_yaw = 0.0;
  if (d_row == 1 && d_col == 0)       { move_yaw = 0.0; }
  else if (d_row == 0 && d_col == 1)  { move_yaw = M_PI / 2.0; }
  else if (d_row == 0 && d_col == -1) { move_yaw = -M_PI / 2.0; }
  else if (d_row == -1 && d_col == 0) { move_yaw = M_PI; }
  else
  {
    direction_name = "invalid";
    return false;
  }

  yaw = move_yaw;
  const double relative_yaw = meilin_normalize_angle(move_yaw - reference_yaw);
  return meilin_direction_from_yaw(relative_yaw, direction, direction_name);
}

/// 计算抓取位置（Fetch 用）
inline void meilin_calculate_grasp_position(
    int kfs_row, int kfs_col, double yaw,
    const MeilinConfig& cfg,
    int from_row, int from_col,
    double& out_x, double& out_y, double& out_yaw)
{
  const auto [kfs_x, kfs_y] = meilin_grid_to_world(kfs_row, kfs_col, cfg);
  const double approach_distance = cfg.grid_size / 2.0 + cfg.grasp_distance;

  double expected_yaw = 0.0;
  int direction = 0;
  std::string direction_name;
  meilin_direction_yaw(from_row, from_col, kfs_row, kfs_col,
                       expected_yaw, direction, direction_name, yaw);

  const int unit_x = kfs_row - from_row;
  const int unit_y = kfs_col - from_col;

  out_x = kfs_x - static_cast<double>(unit_x) * approach_distance;
  out_y = kfs_y - static_cast<double>(unit_y) * approach_distance;
  out_yaw = yaw;
}

/// 计算攀爬边线位置（Move 用）
inline std::pair<double, double> meilin_calculate_climb_position(
    int from_row, int to_row, int col, const MeilinConfig& cfg)
{
  const double edge_row = static_cast<double>(std::min(from_row, to_row)) + 0.5;
  const double x = cfg.grid_origin_x + edge_row * cfg.grid_size;
  const double y = cfg.grid_origin_y + static_cast<double>(col) * cfg.grid_size;
  return {x, y};
}

struct Segment
{
  int index = 0;
  std::string segment_type;
  std::string debug_name;

  // === 梅林区 move 参数（planner 原始值）===
  int move_row = 0;
  int move_col = 0;
  double target_height_mm = 0.0;  // planner arg3，仅用于校验

  // === 梅林区 fetch 参数（planner 原始值）===
  int kfs_row = 0;
  int kfs_col = 0;
  double height_diff = 0.0;       // 正=升, 负=降, 0=无需调整

  // === 共用 ===
  double target_yaw = 0.0;

  // === 以下字段仅用于旧 JSON segment 路径（准备区/竞技区）===
  double target_x = 0.0;
  double target_y = 0.0;
  double max_speed = 0.5;
  double timeout_sec = 30.0;
  double suspension_timeout_sec = 10.0;
  int climb_mode = 0;
  int climb_direction = 0;
  double climb_height = 0.0;
  uint8_t arm_command = 0;
  bool wait_result = true;
  uint8_t spear_command = 0;
};

using SegmentQueue = BT::ProtectedQueue<Segment>;
using SegmentQueuePtr = std::shared_ptr<SegmentQueue>;

struct ArmRuntimeState
{
  std::mutex mtx;
  bool background_active = false;
  bool background_done = true;
  bool background_success = true;
  std::string background_message;
  std::string current_command;
};

using ArmRuntimeStatePtr = std::shared_ptr<ArmRuntimeState>;

}  // namespace r2_bt
