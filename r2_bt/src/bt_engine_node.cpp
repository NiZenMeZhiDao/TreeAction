#include <ament_index_cpp/get_package_share_directory.hpp>
#include <action_of_motion_interfaces/action/move_to_pose.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <r2_interfaces/srv/get_action_seq.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "r2_bt/nodes/actions/ares_tool_action.hpp"
#include "r2_bt/nodes/actions/meilin_fetch.hpp"
#include "r2_bt/nodes/actions/meilin_move.hpp"
#include "r2_bt/nodes/actions/move_to_pose.hpp"
#include "r2_bt/nodes/actions/pop_next_meilin_segment.hpp"
#include "r2_bt/nodes/actions/pop_next_segment.hpp"
#include "r2_bt/nodes/actions/suspension_control.hpp"
#include "r2_bt/nodes/actions/wait_arm_idle.hpp"
#include "r2_bt/nodes/conditions/is_segment_type.hpp"
#include "r2_bt/nodes/conditions/is_string_empty.hpp"
#include "r2_bt/nodes/conditions/is_string_non_empty.hpp"
#include "r2_bt/nodes/conditions/wait_for_r1_signal.hpp"
#include "r2_bt/nodes/controls/switch_segment_type.hpp"
#include "r2_bt/nodes/decorators/for_each_segment.hpp"
#include "r2_bt/nodes/decorators/retry_segment.hpp"
#include "r2_bt/nodes/debug/set_debug_status.hpp"

#include <behaviortree_cpp/decorators/repeat_node.h>

class BtEngineNode : public rclcpp::Node
{
public:
  BtEngineNode() : Node("bt_engine_node")
  {
    // === 通用参数 ===
    declare_parameter("groot2_port", 1667);
    declare_parameter("tick_frequency", 100.0);
    declare_parameter("segment_topic", "/planning/segments");
    declare_parameter("mf_action_topic", "/mf_action_seq");
    declare_parameter("buffer_service", "/get_action_seq");
    declare_parameter("tree_file", "full_match.xml");
    declare_parameter("match_config", "");

    // === 梅林区参数 ===
    declare_parameter("meilin_side", "blue");
    declare_parameter("meilin_grid_size", 1.2);
    declare_parameter("meilin_grid_origin", std::vector<double>{1.2, 1.2});
    declare_parameter("meilin_initial_grid", std::vector<int64_t>{0, 0});
    declare_parameter("meilin_initial_height", 0.0);
    declare_parameter("meilin_initial_yaw", 0.0);
    declare_parameter("meilin_grasp_distance", 0.4);
    declare_parameter("meilin_yaw_tolerance", 0.12);
    declare_parameter("meilin_height_tolerance", 1.0);
    declare_parameter("meilin_default_align_timeout", 30.0);
    declare_parameter("meilin_default_suspension_timeout", 10.0);
    declare_parameter("meilin_default_grasp_timeout", 30.0);
    declare_parameter("meilin_suspension_normal_height", 30.0);  // 正常行驶悬挂高度 (mm)
    declare_parameter("meilin_pose_topic", "/transformed/pose");
    declare_parameter("meilin_pose_timeout_sec", 1.0);
    declare_parameter("meilin_cell_center_tolerance", 0.15);

    groot2_port_ = static_cast<unsigned>(get_parameter("groot2_port").as_int());
    double tick_freq = get_parameter("tick_frequency").as_double();
    segment_topic_ = get_parameter("segment_topic").as_string();
    mf_action_topic_ = get_parameter("mf_action_topic").as_string();
    buffer_service_ = get_parameter("buffer_service").as_string();
    tree_file_ = get_parameter("tree_file").as_string();
    match_config_ = get_parameter("match_config").as_string();
    load_meilin_parameters();

    // === 注册 BT 节点 ===
    factory_.registerNodeType<r2_bt::MoveToPose>("MoveToPose");
    factory_.registerNodeType<r2_bt::MeilinMove>("Move");
    factory_.registerNodeType<r2_bt::MeilinFetch>("Fetch");
    factory_.registerNodeType<r2_bt::PopNextSegment>("PopNextSegment");
    factory_.registerNodeType<r2_bt::PopNextMeilinSegment>("PopNextMeilinSegment");
    factory_.registerNodeType<r2_bt::SuspensionControl>("SuspensionControl");
    factory_.registerNodeType<r2_bt::AresToolAction>("AresToolAction");
    factory_.registerNodeType<r2_bt::WaitArmIdle>("WaitArmIdle");
    factory_.registerNodeType<r2_bt::SwitchSegmentType>("SwitchSegmentType");
    factory_.registerNodeType<r2_bt::IsSegmentType>("IsSegmentType");
    factory_.registerNodeType<r2_bt::IsStringEmpty>("IsStringEmpty");
    factory_.registerNodeType<r2_bt::IsStringNonEmpty>("IsStringNonEmpty");
    factory_.registerNodeType<r2_bt::WaitForR1Signal>("WaitForR1Signal");
    factory_.registerNodeType<r2_bt::ForEachSegment>("ForEachSegment");
    factory_.registerNodeType<r2_bt::RetrySegment>("RetrySegment");
    factory_.registerNodeType<r2_bt::SetDebugStatus>("SetDebugStatus");
    // factory_.registerNodeType<BT::RepeatNode>("Repeat");

    // === 初始化 blackboard ===
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("segment_queue", std::make_shared<r2_bt::SegmentQueue>());
    blackboard_->set("arm_state", std::make_shared<r2_bt::ArmRuntimeState>());
    blackboard_->set("segment_json", std::string{});
    blackboard_->set("plan_id", std::string{});
    blackboard_->set("current_segment_index", -1);
    blackboard_->set("segment_type", std::string{});
    blackboard_->set("segment_debug_name", std::string{});
    blackboard_->set("active_action", std::string{});
    blackboard_->set("retry_count", 0);
    blackboard_->set("last_error", std::string{});
    blackboard_->set("execution_state", std::string{"WAITING_PLAN"});

    move_to_pose_client_ =
        rclcpp_action::create_client<MoveToPoseAction>(
            get_node_base_interface(),
            get_node_graph_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "move_to_pose");
    blackboard_->set("move_to_pose_client", move_to_pose_client_);
    RCLCPP_INFO(get_logger(), "Prewarmed shared MoveToPose client for /move_to_pose");

    // 梅林区静态配置（Move/Fetch 内部计算用）
    auto meilin_cfg = std::make_shared<r2_bt::MeilinConfig>();
    meilin_cfg->side = meilin_side_;
    meilin_cfg->grid_size = meilin_grid_size_;
    meilin_cfg->grid_origin_x = meilin_origin_x_;
    meilin_cfg->grid_origin_y = meilin_origin_y_;
    meilin_cfg->grasp_distance = meilin_grasp_distance_;
    meilin_cfg->yaw_tolerance = meilin_yaw_tolerance_;
    meilin_cfg->height_tolerance = meilin_height_tolerance_;
    meilin_cfg->align_timeout_sec = meilin_default_align_timeout_;
    meilin_cfg->suspension_timeout_sec = meilin_default_suspension_timeout_;
    meilin_cfg->arm_timeout_sec = meilin_default_grasp_timeout_;
    meilin_cfg->suspension_normal_height = meilin_suspension_normal_height_;
    meilin_cfg->pose_timeout_sec = meilin_pose_timeout_sec_;
    meilin_cfg->cell_center_tolerance = meilin_cell_center_tolerance_;
    meilin_cfg->rows = meilin_rows_;
    meilin_cfg->cols = meilin_cols_;
    blackboard_->set("meilin_config", meilin_cfg);

    // PopNextMeilinSegment 输出默认值（仅 planner 原始值）
    blackboard_->set("move_row", 0);
    blackboard_->set("move_col", 0);
    blackboard_->set("target_height_mm", 0.0);
    blackboard_->set("kfs_row", 0);
    blackboard_->set("kfs_col", 0);
    blackboard_->set("height_diff", 0.0);
    blackboard_->set("target_yaw", 0.0);

    // 梅林区动态状态（Move/Fetch 内部更新）
    blackboard_->set("meilin_current_row", meilin_initial_row_);
    blackboard_->set("meilin_current_col", meilin_initial_col_);
    blackboard_->set("meilin_current_height", meilin_initial_height_);
    blackboard_->set("meilin_current_yaw", meilin_initial_yaw_);
    blackboard_->set("meilin_pose_is_cell_center", true);
    blackboard_->set("meilin_suspension_offset", 0.0);  // 悬挂相对 H_INIT 的偏移量 (mm)
    blackboard_->set("meilin_pose_received", false);
    blackboard_->set("meilin_pose_x", 0.0);
    blackboard_->set("meilin_pose_y", 0.0);
    blackboard_->set("meilin_pose_yaw", 0.0);
    blackboard_->set("meilin_pose_last_update_sec", 0.0);

    // 加载比赛配置（PrepareArea/FinalArea 的固定参数）
    if (!match_config_.empty())
    {
      load_match_config();
    }
    else
    {
      RCLCPP_WARN(get_logger(),
                  "No match_config provided. PrepareArea and FinalArea use named "
                  "blackboard variables — ensure they are set before tick.");
    }

    // === 暂存区 Service Client（替代直接订阅 /mf_action_seq） ===
    buffer_client_ = create_client<r2_interfaces::srv::GetActionSeq>(buffer_service_);

    // === 订阅 ===
    segment_sub_ = create_subscription<std_msgs::msg::String>(
        segment_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&BtEngineNode::segment_callback, this, std::placeholders::_1));
    meilin_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        meilin_pose_topic_, rclcpp::QoS(10),
        std::bind(&BtEngineNode::meilin_pose_callback, this, std::placeholders::_1));

    // 定时轮询暂存区（2 Hz）
    buffer_poll_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&BtEngineNode::buffer_poll_callback, this));

    build_fixed_tree();

    auto tick_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / tick_freq));
    tick_timer_ = create_wall_timer(tick_period,
                                    std::bind(&BtEngineNode::tick_callback, this));

    RCLCPP_INFO(get_logger(),
                "BT Engine started: tick=%.1fHz, groot2_port=%u, segment_topic=%s, "
                "buffer_service=%s, meilin_pose_topic=%s, tree_file=%s, match_config=%s",
                tick_freq, groot2_port_, segment_topic_.c_str(),
                buffer_service_.c_str(), meilin_pose_topic_.c_str(), tree_file_.c_str(),
                match_config_.empty() ? "(none)" : match_config_.c_str());
  }

  ~BtEngineNode() override
  {
    groot2_publisher_.reset();
    if (current_tree_)
    {
      current_tree_->haltTree();
    }
  }

private:
  // =========================================================================
  // 字符串命令 → uint8 常量映射
  // =========================================================================

  static uint8_t parse_spear_command_str(const std::string& value)
  {
    if (value == "prep" || value == "prepare") return 1;
    if (value == "grasp") return 2;
    if (value == "dock_extend") return 3;
    if (value == "dock_release") return 4;
    return 0;
  }

  static uint8_t parse_arm_command_str(const std::string& value)
  {
    if (value == "grasp") return 1;
    if (value == "store_to_body") return 2;
    if (value == "store_on_arm") return 3;
    if (value == "get_body") return 4;
    if (value == "place_mid") return 5;
    if (value == "place_high") return 6;
    return 0;
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return r2_bt::meilin_normalize_angle(std::atan2(siny_cosp, cosy_cosp));
  }

  // =========================================================================
  // 梅林区参数加载
  // =========================================================================

  void load_meilin_parameters()
  {
    meilin_side_ = get_parameter("meilin_side").as_string();
    meilin_grid_size_ = get_parameter("meilin_grid_size").as_double();
    meilin_initial_height_ = get_parameter("meilin_initial_height").as_double();
    meilin_initial_yaw_ = r2_bt::meilin_normalize_angle(
        get_parameter("meilin_initial_yaw").as_double());
    meilin_grasp_distance_ = get_parameter("meilin_grasp_distance").as_double();
    meilin_yaw_tolerance_ = get_parameter("meilin_yaw_tolerance").as_double();
    meilin_height_tolerance_ = get_parameter("meilin_height_tolerance").as_double();
    meilin_default_align_timeout_ =
        get_parameter("meilin_default_align_timeout").as_double();
    meilin_default_suspension_timeout_ =
        get_parameter("meilin_default_suspension_timeout").as_double();
    meilin_default_grasp_timeout_ =
        get_parameter("meilin_default_grasp_timeout").as_double();
    meilin_suspension_normal_height_ =
        get_parameter("meilin_suspension_normal_height").as_double();
    meilin_pose_topic_ = get_parameter("meilin_pose_topic").as_string();
    meilin_pose_timeout_sec_ = get_parameter("meilin_pose_timeout_sec").as_double();
    meilin_cell_center_tolerance_ =
        get_parameter("meilin_cell_center_tolerance").as_double();

    const auto origin = get_parameter("meilin_grid_origin").as_double_array();
    if (origin.size() >= 2)
    {
      meilin_origin_x_ = origin[0];
      meilin_origin_y_ = origin[1];
    }

    const auto initial_grid = get_parameter("meilin_initial_grid").as_integer_array();
    if (initial_grid.size() >= 2)
    {
      meilin_initial_row_ = static_cast<int>(initial_grid[0]);
      meilin_initial_col_ = static_cast<int>(initial_grid[1]);
    }
  }

  // =========================================================================
  // 比赛配置加载（PrepareArea / FinalArea 固定参数）
  // =========================================================================

  void load_match_config()
  {
    std::string config_path = resolve_config_file();
    RCLCPP_INFO(get_logger(), "Loading match config: %s", config_path.c_str());

    std::ifstream ifs(config_path);
    if (!ifs.good())
    {
      RCLCPP_ERROR(get_logger(), "Match config file not found: %s", config_path.c_str());
      return;
    }

    nlohmann::json cfg;
    try { ifs >> cfg; }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse match config JSON: %s", e.what());
      return;
    }

    const std::vector<std::string> required_sections = {
      "prepare.spear_prep", "prepare.ares_tool",
      "final.move2", "final.final_move2", "final.place_mid", "final.place_high", "final.finish"
    };
    for (const auto& path : required_sections)
    {
      if (!resolve_json_path(cfg, path))
      {
        RCLCPP_ERROR(get_logger(),
                     "Match config missing required field: %s.", path.c_str());
        blackboard_->set("execution_state", std::string{"CONFIG_ERROR"});
        blackboard_->set("last_error",
                         std::string{"Missing required config field: " + path});
        return;
      }
    }

    // 更新红蓝方
    const auto match_side = cfg.value("match_side", "unknown");
    if (match_side == "red" || match_side == "blue")
    {
      meilin_side_ = match_side;
      if (auto cfg_ptr = blackboard_->get<r2_bt::MeilinConfigPtr>("meilin_config"))
      {
        cfg_ptr->side = match_side;
      }
    }
    RCLCPP_INFO(get_logger(), "Match side: %s", match_side.c_str());

    // PrepareArea 参数
    {
      const auto& p = cfg["prepare"];
      const auto& spear_prep = p["spear_prep"];
      blackboard_->set("prepare_spear_prep_spear_command",
                       parse_spear_command_str(spear_prep.value("spear_command", "prep")));
      blackboard_->set("prepare_spear_prep_target_x", spear_prep.value("target_x", 0.0));
      blackboard_->set("prepare_spear_prep_target_y", spear_prep.value("target_y", 0.0));
      blackboard_->set("prepare_spear_prep_target_yaw", spear_prep.value("target_yaw", 0.0));
      blackboard_->set("prepare_spear_prep_max_speed", spear_prep.value("max_speed", 0.5));
      blackboard_->set("prepare_spear_prep_pid_profile", spear_prep.value("pid_profile", 1));
      blackboard_->set("prepare_spear_prep_timeout_sec", spear_prep.value("timeout_sec", 30.0));

      const auto& ares_tool = p["ares_tool"];
      blackboard_->set("prepare_ares_tool_action",
                       ares_tool.value("action", "grasp"));
      blackboard_->set("prepare_ares_tool_service_name",
                       ares_tool.value("service_name", "/ares_tool_node/tool_action"));
      blackboard_->set("prepare_ares_tool_timeout_sec",
                       ares_tool.value("timeout_sec", 30.0));
      blackboard_->set("prepare_ares_tool_arg0", ares_tool.value("arg0", 0.0));
      blackboard_->set("prepare_ares_tool_arg1", ares_tool.value("arg1", 0.0));
      blackboard_->set("prepare_ares_tool_arg2", ares_tool.value("arg2", 0.0));
      blackboard_->set("prepare_ares_tool_arg3", ares_tool.value("arg3", 0.0));
      RCLCPP_INFO(get_logger(), "PrepareArea params loaded: move + ares_tool service");
    }

    // FinalArea 参数
    {
      const auto& f = cfg["final"];
      const auto& move2 = f["move2"];
      blackboard_->set("final_move2_target_x", move2.value("target_x", 0.0));
      blackboard_->set("final_move2_target_y", move2.value("target_y", 0.0));
      blackboard_->set("final_move2_target_yaw", move2.value("target_yaw", 0.0));
      blackboard_->set("final_move2_max_speed", move2.value("max_speed", 0.5));
      blackboard_->set("final_move2_timeout_sec", move2.value("timeout_sec", 30.0));

      const auto& final_move2 = f["final_move2"];
      blackboard_->set("final_final_move2_target_x", final_move2.value("target_x", 0.0));
      blackboard_->set("final_final_move2_target_y", final_move2.value("target_y", 0.0));
      blackboard_->set("final_final_move2_target_yaw", final_move2.value("target_yaw", 0.0));
      blackboard_->set("final_final_move2_max_speed", final_move2.value("max_speed", 0.5));
      blackboard_->set("final_final_move2_timeout_sec", final_move2.value("timeout_sec", 30.0));

      const auto& place_mid = f["place_mid"];
      blackboard_->set("final_place_mid_arm_command",
                       parse_arm_command_str(place_mid.value("arm_command", "place_mid")));
      blackboard_->set("final_place_mid_timeout_sec", place_mid.value("timeout_sec", 30.0));

      const auto& place_high = f["place_high"];
      blackboard_->set("final_place_high_arm_command",
                       parse_arm_command_str(place_high.value("arm_command", "place_high")));
      blackboard_->set("final_place_high_timeout_sec", place_high.value("timeout_sec", 30.0));

      const auto& finish = f["finish"];
      blackboard_->set("final_finish_arm_command",
                       parse_arm_command_str(finish.value("arm_command", "idle")));
      blackboard_->set("final_finish_timeout_sec", finish.value("timeout_sec", 30.0));
      blackboard_->set("final_finish_suspension_mode", finish.value("suspension_mode", 3));
      blackboard_->set("final_finish_suspension_direction", finish.value("suspension_direction", 0));
      blackboard_->set("final_finish_suspension_height", finish.value("suspension_height", 30.0));
      blackboard_->set("final_finish_suspension_timeout_sec",
                       finish.value("suspension_timeout_sec", 10.0));
      RCLCPP_INFO(get_logger(), "FinalArea params loaded: 5 segments");
    }

    blackboard_->set("execution_state", std::string{"CONFIG_LOADED"});
    RCLCPP_INFO(get_logger(), "Match config loaded: side=%s", match_side.c_str());
  }

  static bool resolve_json_path(const nlohmann::json& root, const std::string& path)
  {
    const nlohmann::json* current = &root;
    std::string segment;
    for (char ch : path)
    {
      if (ch == '.')
      {
        if (!current->is_object() || !current->contains(segment)) return false;
        current = &(*current)[segment];
        segment.clear();
      }
      else { segment += ch; }
    }
    return current->is_object() && current->contains(segment);
  }

  std::string resolve_config_file() const
  {
    if (!match_config_.empty() && match_config_.front() == '/')
      return match_config_;
    auto share_dir = ament_index_cpp::get_package_share_directory("r2_bt");
    if (match_config_.find("config/") == 0)
      return share_dir + "/" + match_config_;
    return share_dir + "/config/" + match_config_;
  }

  // =========================================================================
  // 梅林区状态快照（用于解析时模拟状态以做序列校验）
  // =========================================================================

  struct MeilinSimState
  {
    int row = 0;
    int col = 0;
    double height = 0.0;
    double yaw = 0.0;
    bool pose_is_cell_center = true;
  };

  MeilinSimState snapshot_meilin_state() const
  {
    MeilinSimState s;
    s.row = meilin_initial_row_;
    s.col = meilin_initial_col_;
    s.height = meilin_initial_height_;
    s.yaw = meilin_initial_yaw_;
    s.pose_is_cell_center = true;
    return s;
  }

  bool grid_in_range(int row, int col) const
  {
    return row >= 0 && row < meilin_rows_ && col >= 0 && col < meilin_cols_;
  }

  // =========================================================================
  // 构建简化的 Meilin Segment（只存 planner 原始值）
  // =========================================================================

  r2_bt::Segment make_move_segment(size_t idx, int row, int col,
                                   double requested_height, double yaw,
                                   const MeilinSimState& state,
                                   std::string& error)
  {
    // 1. 校验 grid
    if (!grid_in_range(row, col))
    {
      std::ostringstream oss;
      oss << "move grid out of range: (" << row << "," << col << ")";
      error = oss.str();
      return {};
    }

    // 2. height_map 与 planner arg3 相互检查
    const double map_height = r2_bt::meilin_height_at(row, col, meilin_side_);
    if (std::abs(requested_height) > meilin_height_tolerance_ &&
        std::abs(requested_height - map_height) > meilin_height_tolerance_)
    {
      std::ostringstream oss;
      oss << "move target_height mismatch at (" << row << "," << col
          << "): planner=" << requested_height << ", height_map=" << map_height;
      error = oss.str();
      return {};
    }

    // 3. 如果相邻且有高度差，校验方向（无后退）
    const double h_diff = map_height - state.height;
    if (std::abs(h_diff) > meilin_height_tolerance_)
    {
      if (!r2_bt::meilin_adjacent(state.row, state.col, row, col))
      {
        error = "move with height change requires adjacent target grid";
        return {};
      }
      double unused_yaw = 0.0;
      int direction = 0;
      std::string dir_name;
      r2_bt::meilin_direction_yaw(state.row, state.col, row, col,
                                  unused_yaw, direction, dir_name);
      if (dir_name == "backward")
      {
        error = "move does not support backward direction";
        return {};
      }
    }

    // 4. 构建 segment — 只存原始值
    r2_bt::Segment seg;
    seg.index = static_cast<int>(idx);
    seg.segment_type = "move";
    seg.debug_name = "move#" + std::to_string(idx);
    seg.move_row = row;
    seg.move_col = col;
    seg.target_height_mm = map_height;
    seg.target_yaw = r2_bt::meilin_normalize_angle(yaw);
    (void)requested_height;
    return seg;
  }

  r2_bt::Segment make_fetch_segment(size_t idx, int row, int col,
                                    double height_diff, double yaw,
                                    const MeilinSimState& state,
                                    std::string& error)
  {
    // 1. 校验
    if (!grid_in_range(row, col))
    {
      std::ostringstream oss;
      oss << "fetch grid out of range: (" << row << "," << col << ")";
      error = oss.str();
      return {};
    }
    if (!state.pose_is_cell_center)
    {
      error = "fetch requires robot pose at current cell center";
      return {};
    }
    if (!r2_bt::meilin_adjacent(state.row, state.col, row, col))
    {
      error = "fetch KFS grid must be adjacent to current grid";
      return {};
    }

    double expected_yaw = 0.0;
    int direction = 0;
    std::string dir_name;
    if (!r2_bt::meilin_direction_yaw(state.row, state.col, row, col,
                                     expected_yaw, direction, dir_name))
    {
      error = "fetch direction is invalid";
      return {};
    }
    if (dir_name == "backward")
    {
      error = "fetch only supports forward/left/right, not backward";
      return {};
    }
    if (!r2_bt::meilin_yaw_close(yaw, expected_yaw, meilin_yaw_tolerance_))
    {
      std::ostringstream oss;
      oss << "fetch yaw mismatch for " << dir_name
          << ": msg=" << yaw << ", expected=" << expected_yaw;
      error = oss.str();
      return {};
    }

    // 2. 构建 segment — 只存原始值
    r2_bt::Segment seg;
    seg.index = static_cast<int>(idx);
    seg.segment_type = "fetch";
    seg.debug_name = "fetch#" + std::to_string(idx);
    seg.kfs_row = row;
    seg.kfs_col = col;
    seg.height_diff = height_diff;
    seg.target_yaw = r2_bt::meilin_normalize_angle(yaw);
    return seg;
  }

  // =========================================================================
  // 解析 /mf_action_seq → Segment 队列（只校验 + 存原始值）
  // =========================================================================

  std::vector<r2_bt::Segment> parse_mf_action_seq(
      const std::vector<float>& data, std::string& plan_id, std::string& error)
  {
    std::vector<r2_bt::Segment> result;
    if (data.empty()) { error = "empty Float32MultiArray"; return result; }
    if (data.size() % 8 != 0)
    {
      std::ostringstream oss;
      oss << "Float32MultiArray length must be divisible by 8, got " << data.size();
      error = oss.str();
      return result;
    }

    auto sim = snapshot_meilin_state();
    const size_t count = data.size() / 8;
    for (size_t i = 0; i < count; ++i)
    {
      const size_t off = i * 8;

      // 解析 arg0 = action_type
      if (!std::isfinite(data[off + 0]))
      {
        error = "action type contains NaN/Inf";
        return {};
      }
      const int action_type = static_cast<int>(std::lround(data[off + 0]));
      if (std::abs(data[off + 0] - static_cast<float>(action_type)) > 1e-3f ||
          (action_type != 0 && action_type != 1))
      {
        std::ostringstream oss;
        oss << "arg0 must be 0(move) or 1(fetch), got " << data[off + 0];
        error = oss.str();
        return {};
      }

      // 解析 arg1 = row, arg2 = col (int)
      auto parse_int = [&](float val, const std::string& label, int& out) -> bool {
        if (!std::isfinite(val)) { error = label + " contains NaN/Inf"; return false; }
        const int r = static_cast<int>(std::lround(val));
        if (std::abs(val - static_cast<float>(r)) > 1e-3f)
        {
          std::ostringstream oss;
          oss << label << " must be an integer grid index, got " << val;
          error = oss.str();
          return false;
        }
        out = r;
        return true;
      };

      int row = 0, col = 0;
      if (!parse_int(data[off + 1], "row", row) ||
          !parse_int(data[off + 2], "col", col))
        return {};

      RCLCPP_INFO(get_logger(),
                  "Meilin grid→world: (%d,%d) → (%.2f, %.2f)",
                  row, col,
                  meilin_origin_x_ + row * meilin_grid_size_,
                  meilin_origin_y_ + col * meilin_grid_size_);

      const double arg3 = data[off + 3];
      const double yaw = r2_bt::meilin_normalize_angle(data[off + 4]);
      if (!std::isfinite(arg3) || !std::isfinite(data[off + 4]))
      {
        error = "arg3/yaw contains NaN/Inf";
        return {};
      }

      r2_bt::Segment seg;
      if (action_type == 0)
      {
        // move
        seg = make_move_segment(i, row, col, arg3, yaw, sim, error);
        if (!error.empty()) return {};
        sim.row = row;
        sim.col = col;
        sim.height = r2_bt::meilin_height_at(row, col, meilin_side_);
        sim.yaw = seg.target_yaw;
        sim.pose_is_cell_center = true;
      }
      else
      {
        // fetch
        seg = make_fetch_segment(i, row, col, arg3, yaw, sim, error);
        if (!error.empty()) return {};
        sim.yaw = seg.target_yaw;
        sim.pose_is_cell_center = false;
      }

      result.push_back(seg);
    }

    ++meilin_plan_counter_;
    plan_id = "mf_action_seq_" + std::to_string(meilin_plan_counter_);
    return result;
  }

  // =========================================================================
  // /transformed/pose 回调（map 系 base_link 位姿）
  // =========================================================================

  void meilin_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto& pose = msg->pose;
    blackboard_->set("meilin_pose_received", true);
    blackboard_->set("meilin_pose_x", pose.position.x);
    blackboard_->set("meilin_pose_y", pose.position.y);
    blackboard_->set("meilin_pose_yaw", yaw_from_quaternion(pose.orientation));
    blackboard_->set("meilin_pose_last_update_sec", now().seconds());
  }

  // =========================================================================
  // 暂存区轮询（替代直接订阅 /mf_action_seq）
  // =========================================================================

  void process_action_seq(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::string plan_id;
    std::string error;
    auto segments = parse_mf_action_seq(msg->data, plan_id, error);
    if (segments.empty())
    {
      RCLCPP_ERROR(get_logger(), "Meilin plan rejected: %s", error.c_str());
      blackboard_->set("last_error", error);
      blackboard_->set("execution_state", std::string{"PLAN_REJECTED"});
      return;
    }

    // 追加 PLAN_DONE 哨兵
    r2_bt::Segment done;
    done.index = static_cast<int>(segments.size());
    done.segment_type = "PLAN_DONE";
    done.debug_name = "meilin_plan_done";
    segments.push_back(done);

    // 原地更新现有队列，避免替换 shared_ptr 导致 SubTree 子 blackboard 持有旧引用
    r2_bt::SegmentQueuePtr queue;
    if (!blackboard_->get("segment_queue", queue) || !queue)
    {
      queue = std::make_shared<r2_bt::SegmentQueue>();
      blackboard_->set("segment_queue", queue);
    }
    {
      std::unique_lock<std::mutex> lock(queue->mtx);
      queue->items.assign(segments.begin(), segments.end());
    }

    blackboard_->set("plan_id", plan_id);
    blackboard_->set("current_segment_index", -1);
    blackboard_->set("segment_type", std::string{});
    blackboard_->set("segment_debug_name", std::string{});
    blackboard_->set("active_action", std::string{});
    blackboard_->set("retry_count", 0);
    blackboard_->set("last_error", std::string{});
    blackboard_->set("execution_state", std::string{"MF_PLAN_READY"});

    RCLCPP_INFO(get_logger(), "Meilin plan accepted: plan_id=%s, actions=%zu",
                plan_id.c_str(), segments.size() - 1);
  }

  void buffer_poll_callback()
  {
    // 只在等待新计划时才拉取暂存区
    std::string state;
    bool state_got = blackboard_->get("execution_state", state);
    if (state != "WAITING_MF_ACTION_SEQ" && state != "WAITING_PLAN" && state != "IDLE")
      return;

    if ((!state_got) || (!buffer_client_->service_is_ready()))
      return;

    auto request = std::make_shared<r2_interfaces::srv::GetActionSeq::Request>();
    buffer_client_->async_send_request(request,
        [this](rclcpp::Client<r2_interfaces::srv::GetActionSeq>::SharedFuture future) {
          auto response = future.get();
          if (!response->has_data) return;

          auto msg = std::make_shared<std_msgs::msg::Float32MultiArray>();
          msg->data = response->data.data;
          msg->layout = response->data.layout;
          process_action_seq(msg);
        });
  }

  // =========================================================================
  // /planning/segments 回调（旧 JSON 路径，保持兼容）
  // =========================================================================

  void segment_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string plan_id;
    auto segments = parse_segment_json(msg->data, plan_id);
    if (segments.empty())
    {
      RCLCPP_ERROR(get_logger(), "Segment plan rejected: no executable segments");
      blackboard_->set("last_error", std::string{"Segment plan rejected"});
      blackboard_->set("execution_state", std::string{"MISSION_FAILED"});
      return;
    }

    // 原地更新现有队列，避免替换 shared_ptr 导致 SubTree 子 blackboard 持有旧引用
    r2_bt::SegmentQueuePtr queue;
    if (!blackboard_->get("segment_queue", queue) || !queue)
    {
      queue = std::make_shared<r2_bt::SegmentQueue>();
      blackboard_->set("segment_queue", queue);
    }
    {
      std::unique_lock<std::mutex> lock(queue->mtx);
      queue->items.assign(segments.begin(), segments.end());
    }

    blackboard_->set("segment_json", msg->data);
    blackboard_->set("plan_id", plan_id);
    blackboard_->set("current_segment_index", -1);
    blackboard_->set("segment_type", std::string{});
    blackboard_->set("segment_debug_name", std::string{});
    blackboard_->set("active_action", std::string{});
    blackboard_->set("retry_count", 0);
    blackboard_->set("last_error", std::string{});
    blackboard_->set("execution_state", std::string{"WAITING_PLAN"});

    RCLCPP_INFO(get_logger(), "Segment plan received: plan_id=%s, segments=%zu",
                plan_id.c_str(), segments.size());
  }

  // =========================================================================
  // 旧 JSON segment 解析（保持兼容）
  // =========================================================================

  int parse_climb_mode(const std::string& value) const
  {
    if (value == "UP") return 1;
    if (value == "DOWN") return 2;
    if (value == "RECOVER") return 3;
    return 0;
  }

  int parse_climb_direction(const std::string& value) const
  {
    if (value == "LEFT") return 1;
    if (value == "RIGHT") return 2;
    if (value == "BACKWARD") return 3;
    return 0;
  }

  uint8_t parse_arm_command(const std::string& value) const
  {
    if (value == "grasp") return 1;
    if (value == "store_to_body") return 2;
    if (value == "store_on_arm") return 3;
    if (value == "get_body") return 4;
    if (value == "place_mid") return 5;
    if (value == "place_high") return 6;
    if (value == "safe_pose") return 7;
    if (value == "prepare_grasp") return 8;
    return 0;
  }

  uint8_t parse_spear_command(const std::string& value) const
  {
    if (value == "prepare") return 1;
    if (value == "grasp") return 2;
    if (value == "dock_extend") return 3;
    if (value == "dock_release") return 4;
    return 0;
  }

  void load_move_target(const nlohmann::json& item, r2_bt::Segment& segment)
  {
    if (!item.contains("move_target") || !item["move_target"].is_object())
      throw std::runtime_error(segment.segment_type + " requires move_target");
    const auto& target = item["move_target"];
    segment.target_x = target.at("x").get<double>();
    segment.target_y = target.at("y").get<double>();
    segment.target_yaw = target.value("yaw", 0.0);
  }

  std::vector<r2_bt::Segment> parse_segment_json(const std::string& json_str,
                                                 std::string& plan_id)
  {
    std::vector<r2_bt::Segment> result;
    try
    {
      const auto doc = nlohmann::json::parse(json_str);
      if (!doc.contains("segments") || !doc["segments"].is_array())
        throw std::runtime_error("top-level segments array is required");

      plan_id = doc.value("plan_id", "");

      int index = 0;
      for (const auto& item : doc["segments"])
      {
        if (!item.is_object())
          throw std::runtime_error("segment item must be an object");

        r2_bt::Segment segment;
        segment.index = index++;
        segment.segment_type = item.at("segment_type").get<std::string>();
        segment.debug_name = segment.segment_type + "#" + std::to_string(segment.index);

        if (segment.segment_type == "SPEAR_PREP")
        {
          load_move_target(item, segment);
          segment.max_speed = item.value("max_speed", 0.4);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.spear_command = 1;
        }
        else if (segment.segment_type == "SPEAR_GRASP")
        {
          segment.timeout_sec = item.value("timeout_sec", 5.0);
          segment.spear_command = 2;
        }
        else if (segment.segment_type == "ALIGN")
        {
          load_move_target(item, segment);
          segment.max_speed = item.value("max_speed", 0.4);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
        }
        else if (segment.segment_type == "DOCK")
        {
          segment.timeout_sec = item.value("timeout_sec", 10.0);
        }
        else if (segment.segment_type == "MOVE2")
        {
          load_move_target(item, segment);
          segment.max_speed = item.value("max_speed", 0.5);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
        }
        else if (segment.segment_type == "GRASP")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 1;
          segment.wait_result = true;
          segment.height_diff = item.value("height_diff", 0.0);
        }
        else if (segment.segment_type == "CLIMB")
        {
          load_move_target(item, segment);
          segment.climb_mode = parse_climb_mode(item.at("climb_mode").get<std::string>());
          segment.climb_direction =
              parse_climb_direction(item.at("climb_direction").get<std::string>());
          segment.climb_height = item.at("climb_height").get<double>();
          segment.max_speed = item.value("max_speed", 0.4);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = parse_arm_command(item.value("arm_command", ""));
          segment.wait_result = item.value("wait_result", true);
        }
        else if (segment.segment_type == "STORE1")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 2;
          segment.wait_result = item.value("wait_result", false);
        }
        else if (segment.segment_type == "STORE2")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 3;
          segment.wait_result = item.value("wait_result", false);
        }
        else if (segment.segment_type == "EXIT")
        {
          load_move_target(item, segment);
          segment.max_speed = item.value("max_speed", 0.6);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 7;
          segment.wait_result = true;
        }
        else if (segment.segment_type == "FINAL_MOVE2")
        {
          load_move_target(item, segment);
          segment.max_speed = item.value("max_speed", 0.4);
          segment.timeout_sec = item.value("timeout_sec", 30.0);
        }
        else if (segment.segment_type == "PLACE_MID")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 5;
          segment.wait_result = true;
        }
        else if (segment.segment_type == "PLACE_HIGH")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 6;
          segment.wait_result = true;
        }
        else if (segment.segment_type == "FINISH")
        {
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 7;
          segment.wait_result = true;
        }
        else
        {
          throw std::runtime_error("unsupported segment_type: " + segment.segment_type);
        }

        result.push_back(segment);
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse segment JSON: %s", e.what());
      result.clear();
    }
    return result;
  }

  // =========================================================================
  // BehaviorTree 管理
  // =========================================================================

  std::string resolve_tree_file() const
  {
    if (!tree_file_.empty() && tree_file_.front() == '/')
      return tree_file_;
    auto share_dir = ament_index_cpp::get_package_share_directory("r2_bt");
    return share_dir + "/trees/" + tree_file_;
  }

  void build_fixed_tree()
  {
    groot2_publisher_.reset();
    if (current_tree_)
    {
      current_tree_->haltTree();
      current_tree_.reset();
    }

    auto tree_path = resolve_tree_file();
    RCLCPP_INFO(get_logger(), "Loading fixed behavior tree: %s", tree_path.c_str());

    try
    {
      current_tree_ = std::make_unique<BT::Tree>(
          factory_.createTreeFromFile(tree_path, blackboard_));
      try
      {
        groot2_publisher_ = std::make_unique<BT::Groot2Publisher>(
            *current_tree_, groot2_port_);
        RCLCPP_INFO(get_logger(), "Groot2 publisher started on port %u", groot2_port_);
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(get_logger(), "Fixed tree loaded, but Groot2 publisher failed: %s",
                    e.what());
      }
      RCLCPP_INFO(get_logger(), "Fixed tree loaded successfully");
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load fixed tree: %s", e.what());
      current_tree_.reset();
    }
  }

  void tick_callback()
  {
    if (current_tree_)
    {
      blackboard_->set("ros_node", shared_from_this());
      auto status = current_tree_->tickOnce();
      if (status == BT::NodeStatus::FAILURE)
      {
        blackboard_->set("execution_state", std::string{"MISSION_FAILED"});
      }
    }
  }

  // =========================================================================
  // 成员变量
  // =========================================================================

  using MoveToPoseAction = action_of_motion_interfaces::action::MoveToPose;

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Tree> current_tree_;
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr segment_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr meilin_pose_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr buffer_poll_timer_;

  // 暂存区 Service Client
  rclcpp::Client<r2_interfaces::srv::GetActionSeq>::SharedPtr buffer_client_;
  rclcpp_action::Client<MoveToPoseAction>::SharedPtr move_to_pose_client_;

  unsigned groot2_port_ = 1667;
  std::string segment_topic_;
  std::string mf_action_topic_;
  std::string buffer_service_;
  std::string meilin_pose_topic_;
  std::string tree_file_;
  std::string match_config_;

  // 梅林区参数
  std::string meilin_side_ = "blue";
  int meilin_rows_ = 6;
  int meilin_cols_ = 3;
  int meilin_initial_row_ = 0;
  int meilin_initial_col_ = 0;
  double meilin_initial_height_ = 0.0;
  double meilin_initial_yaw_ = 0.0;
  double meilin_grid_size_ = 1.2;
  double meilin_origin_x_ = 0;
  double meilin_origin_y_ = -1.2;
  double meilin_grasp_distance_ = 0.4;
  double meilin_yaw_tolerance_ = 0.12;
  double meilin_height_tolerance_ = 1.0;
  double meilin_default_align_timeout_ = 30.0;
  double meilin_default_suspension_timeout_ = 10.0;
  double meilin_default_grasp_timeout_ = 30.0;
  double meilin_suspension_normal_height_ = 30.0;  // 正常行驶悬挂高度 (mm)，即 H_INIT
  double meilin_pose_timeout_sec_ = 1.0;
  double meilin_cell_center_tolerance_ = 0.15;
  uint64_t meilin_plan_counter_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BtEngineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
