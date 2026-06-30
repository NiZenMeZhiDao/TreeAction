#include <ament_index_cpp/get_package_share_directory.hpp>
#include <action_of_motion_interfaces/action/move_to_pose.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <pick_action_interfaces/action/pick_sequence.hpp>
#include <r2_interfaces/action/arm_action.hpp>
#include <r2_interfaces/action/spear_action.hpp>
#include <r2_interfaces/action/step_motion_control.hpp>
#include <r2_interfaces/action/suspension_control.hpp>
#include <r2_interfaces/srv/get_action_seq.hpp>
#include <r2_interfaces/srv/start_autonomy.hpp>
#include <r2_interfaces/srv/tool_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cctype>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "r2_bt/nodes/actions/ares_tool_action.hpp"
#include "r2_bt/nodes/actions/go_to_pose.hpp"
#include "r2_bt/nodes/actions/meilin_fetch.hpp"
#include "r2_bt/nodes/actions/meilin_move.hpp"
#include "r2_bt/nodes/actions/meilin_move_plan.hpp"
#include "r2_bt/nodes/actions/move_through_final_waypoints.hpp"
#include "r2_bt/nodes/actions/move_to_pose.hpp"
#include "r2_bt/nodes/actions/place_object_placeholder.hpp"
#include "r2_bt/nodes/actions/pick_action.hpp"
#include "r2_bt/nodes/actions/pop_next_meilin_segment.hpp"
#include "r2_bt/nodes/actions/pop_next_segment.hpp"
#include "r2_bt/nodes/actions/publish_chassis_height.hpp"
#include "r2_bt/nodes/actions/select_final_target.hpp"
#include "r2_bt/nodes/actions/step_motion_control.hpp"
#include "r2_bt/nodes/actions/suspension_control.hpp"
#include "r2_bt/nodes/actions/wait_arm_idle.hpp"
#include "r2_bt/nodes/actions/wait_for_int_signal.hpp"
#include "r2_bt/nodes/actions/wait_seconds.hpp"
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
    declare_parameter("param_config", "");
    declare_parameter("match_config", "");
    declare_parameter("autostart", true);
    declare_parameter("default_region", "full");
    declare_parameter("require_map_relocalization", false);
    declare_parameter("localization_timeout_sec", 1.0);

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
    declare_parameter("meilin_motion_mode", "single_axis");

    groot2_port_ = static_cast<unsigned>(get_parameter("groot2_port").as_int());
    double tick_freq = get_parameter("tick_frequency").as_double();
    segment_topic_ = get_parameter("segment_topic").as_string();
    mf_action_topic_ = get_parameter("mf_action_topic").as_string();
    buffer_service_ = get_parameter("buffer_service").as_string();
    tree_file_ = get_parameter("tree_file").as_string();
    param_config_ = get_parameter("param_config").as_string();
    match_config_ = get_parameter("match_config").as_string();
    autostart_ = get_parameter("autostart").as_bool();
    autonomy_started_ = autostart_;
    default_region_ = normalize_region(get_parameter("default_region").as_string());
    require_map_relocalization_ =
        get_parameter("require_map_relocalization").as_bool();
    localization_timeout_sec_ =
        get_parameter("localization_timeout_sec").as_double();
    load_meilin_parameters();

    // === 注册 BT 节点 ===
    factory_.registerNodeType<r2_bt::MoveToPose>("MoveToPose");
    factory_.registerNodeType<r2_bt::GoToPose>("GoToPose");
    factory_.registerNodeType<r2_bt::MeilinMove>("Move");
    factory_.registerNodeType<r2_bt::MeilinFetch>("Fetch");
    factory_.registerNodeType<r2_bt::MeilinPlanMove>("MeilinPlanMove");
    factory_.registerNodeType<r2_bt::MeilinCommitMove>("MeilinCommitMove");
    factory_.registerNodeType<r2_bt::MoveThroughFinalWaypoints>("MoveThroughFinalWaypoints");
    factory_.registerNodeType<r2_bt::PopNextSegment>("PopNextSegment");
    factory_.registerNodeType<r2_bt::PopNextMeilinSegment>("PopNextMeilinSegment");
    factory_.registerNodeType<r2_bt::SuspensionControl>("SuspensionControl");
    factory_.registerNodeType<r2_bt::StepMotionControl>("StepMotionControl");
    factory_.registerNodeType<r2_bt::AresToolAction>("AresToolAction");
    factory_.registerNodeType<r2_bt::PickAction>("PickAction");
    factory_.registerNodeType<r2_bt::PlaceObjectPlaceholder>("PlaceObjectPlaceholder");
    factory_.registerNodeType<r2_bt::PublishChassisHeight>("PublishChassisHeight");
    factory_.registerNodeType<r2_bt::SelectFinalTarget>("SelectFinalTarget");
    factory_.registerNodeType<r2_bt::WaitArmIdle>("WaitArmIdle");
    factory_.registerNodeType<r2_bt::WaitForIntSignal>("WaitForIntSignal");
    factory_.registerNodeType<r2_bt::WaitSeconds>("WaitSeconds");
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
    meilin_cfg->motion_mode = meilin_motion_mode_;
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
    blackboard_->set("meilin_pose_frame_id", std::string{});
    blackboard_->set("meilin_entry_move_pending", false);

    // 加载全区域 YAML 参数；match_config 仅作为旧 JSON 覆盖入口保留。
    load_param_config();
    if (!match_config_.empty())
    {
      load_match_config();
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "No legacy match_config provided; using param_config for area params.");
    }

    prewarm_ros_clients();

    start_autonomy_service_ = create_service<r2_interfaces::srv::StartAutonomy>(
        "/bt_engine/start_autonomy",
        std::bind(&BtEngineNode::start_autonomy_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

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
    if (!autonomy_started_)
    {
      blackboard_->set("execution_state", std::string{"IDLE"});
      blackboard_->set("active_action", std::string{});
      RCLCPP_INFO(get_logger(),
                  "Autonomy gated: waiting for /bt_engine/start_autonomy "
                  "(default_region=%s)",
                  default_region_.c_str());
    }

    auto tick_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / tick_freq));
    tick_timer_ = create_wall_timer(tick_period,
                                    std::bind(&BtEngineNode::tick_callback, this));

    RCLCPP_INFO(get_logger(),
                "BT Engine started: tick=%.1fHz, groot2_port=%u, segment_topic=%s, "
                "buffer_service=%s, meilin_pose_topic=%s, tree_file=%s, match_config=%s, "
                "autostart=%s",
                tick_freq, groot2_port_, segment_topic_.c_str(),
                buffer_service_.c_str(), meilin_pose_topic_.c_str(), tree_file_.c_str(),
                match_config_.empty() ? "(none)" : match_config_.c_str(),
                autostart_ ? "true" : "false");
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

  static std::string normalize_region(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
  }

  static std::string tree_file_for_region(const std::string& region)
  {
    if (region.empty() || region == "full" || region == "full_match")
      return "full_match.xml";
    if (region == "prepare" || region == "prepare_area")
      return "prepare_area.xml";
    if (region == "meilin" || region == "minimal_meilin" || region == "meilin_stage")
      return "meilin_stage.xml";
    if (region == "final" || region == "final_area")
      return "final_area.xml";
    return {};
  }

  static bool region_needs_prepare(const std::string& region)
  {
    return region == "prepare" || region == "prepare_area" ||
           region == "full" || region == "full_match";
  }

  static bool region_needs_meilin(const std::string& region)
  {
    return region == "meilin" || region == "minimal_meilin" ||
           region == "meilin_stage" || region == "full" || region == "full_match";
  }

  static bool region_needs_final(const std::string& region)
  {
    return region == "final" || region == "final_area" ||
           region == "full" || region == "full_match";
  }

  bool append_if_missing(bool ready, const std::string& name,
                         std::vector<std::string>& missing) const
  {
    if (!ready)
    {
      missing.push_back(name);
      return false;
    }
    return true;
  }

  void prewarm_ros_clients()
  {
    std::string pick_action_name = "/pick_action";
    const bool pick_action_name_loaded =
        blackboard_->get("prepare_pick_action_server_name", pick_action_name);
    (void)pick_action_name_loaded;

    std::string tool_service_name = "/ares_tool_node/tool_action";
    const bool tool_service_name_loaded =
        blackboard_->get("prepare_spear_tool_service_name", tool_service_name);
    (void)tool_service_name_loaded;
    const bool final_tool_service_name_loaded =
        blackboard_->get("final_place_action_service_name", tool_service_name);
    (void)final_tool_service_name_loaded;

    move_to_pose_client_ =
        rclcpp_action::create_client<MoveToPoseAction>(
            get_node_base_interface(),
            get_node_graph_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "move_to_pose");
    blackboard_->set("move_to_pose_client", move_to_pose_client_);

    pick_action_client_ = rclcpp_action::create_client<PickSequenceAction>(
        get_node_base_interface(),
        get_node_graph_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        pick_action_name);
    blackboard_->set("pick_action_client", pick_action_client_);
    blackboard_->set("pick_action_client_name", pick_action_name);

    tool_client_ = create_client<ToolActionSrv>(tool_service_name);
    blackboard_->set("tool_action_client", tool_client_);
    blackboard_->set("tool_action_client_name", tool_service_name);

    buffer_client_ = create_client<r2_interfaces::srv::GetActionSeq>(buffer_service_);
    step_motion_client_ = rclcpp_action::create_client<StepMotionAction>(
        get_node_base_interface(),
        get_node_graph_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        "step_motion_control");
    blackboard_->set("step_motion_client", step_motion_client_);

    RCLCPP_INFO(get_logger(),
                "Prewarmed shared clients: move_to_pose=/move_to_pose, "
                "pick_action=%s, tool_service=%s, step_motion=/step_motion_control, "
                "buffer_service=%s",
                pick_action_name.c_str(), tool_service_name.c_str(),
                buffer_service_.c_str());
  }

  std::vector<std::string> missing_start_dependencies(const std::string& region) const
  {
    std::vector<std::string> missing;

    const bool needs_prepare = region_needs_prepare(region);
    const bool needs_meilin = region_needs_meilin(region);
    const bool needs_final = region_needs_final(region);

    if (needs_prepare || needs_meilin || needs_final)
    {
      append_if_missing(move_to_pose_client_ && move_to_pose_client_->action_server_is_ready(),
                        "/move_to_pose action", missing);
    }

    if (needs_prepare)
    {
      append_if_missing(pick_action_client_ && pick_action_client_->action_server_is_ready(),
                        "/pick_action action", missing);
      append_if_missing(tool_client_ && tool_client_->service_is_ready(),
                        "/ares_tool_node/tool_action service", missing);
    }

    if (needs_meilin)
    {
      append_if_missing(step_motion_client_ && step_motion_client_->action_server_is_ready(),
                        "/step_motion_control action", missing);
      append_if_missing(tool_client_ && tool_client_->service_is_ready(),
                        "/ares_tool_node/tool_action service", missing);
      append_if_missing(buffer_client_ && buffer_client_->service_is_ready(),
                        buffer_service_ + " service", missing);

      if (require_map_relocalization_)
      {
        bool pose_received = false;
        const bool pose_received_got =
            blackboard_->get("meilin_pose_received", pose_received);

        double last_update_sec = 0.0;
        const bool last_update_got =
            blackboard_->get("meilin_pose_last_update_sec", last_update_sec);

        std::string frame_id;
        const bool frame_id_got =
            blackboard_->get("meilin_pose_frame_id", frame_id);

        const double age_sec = now().seconds() - last_update_sec;
        append_if_missing(pose_received_got && pose_received,
                          meilin_pose_topic_ + " pose", missing);
        if (pose_received_got && pose_received)
        {
          append_if_missing(last_update_got && age_sec <= localization_timeout_sec_,
                            meilin_pose_topic_ + " fresh pose", missing);
          append_if_missing(frame_id_got && frame_id == "map",
                            meilin_pose_topic_ + " frame_id=map", missing);
        }
      }
    }

    if (needs_final && !needs_prepare && !needs_meilin)
    {
      append_if_missing(tool_client_ && tool_client_->service_is_ready(),
                        "/ares_tool_node/tool_action service", missing);
    }

    return missing;
  }

  static std::string join_strings(const std::vector<std::string>& items,
                                  const std::string& delimiter)
  {
    std::ostringstream oss;
    for (size_t i = 0; i < items.size(); ++i)
    {
      if (i > 0) oss << delimiter;
      oss << items[i];
    }
    return oss.str();
  }

  void reset_runtime_blackboard(const std::string& region)
  {
    blackboard_->set("plan_id", std::string{});
    blackboard_->set("current_segment_index", -1);
    blackboard_->set("segment_type", std::string{});
    blackboard_->set("segment_debug_name", std::string{});
    blackboard_->set("active_action", std::string{});
    blackboard_->set("retry_count", 0);
    blackboard_->set("last_error", std::string{});
    blackboard_->set("meilin_entry_move_pending", false);
    blackboard_->set("execution_state",
                     std::string{"AUTONOMY_RUNNING:"} + region);

    r2_bt::SegmentQueuePtr queue;
    if (blackboard_->get("segment_queue", queue) && queue)
    {
      std::unique_lock<std::mutex> lock(queue->mtx);
      queue->items.clear();
    }
  }

  void start_autonomy_callback(
      const std::shared_ptr<r2_interfaces::srv::StartAutonomy::Request> request,
      std::shared_ptr<r2_interfaces::srv::StartAutonomy::Response> response)
  {
    const std::string requested_region = normalize_region(request->region);
    const std::string region =
        requested_region.empty() ? default_region_ : requested_region;
    const std::string requested_tree = tree_file_for_region(region);
    if (requested_tree.empty())
    {
      response->success = false;
      response->message = "Unsupported autonomy region: " + region;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    if (tree_file_ != requested_tree)
    {
      tree_file_ = requested_tree;
      build_fixed_tree();
      if (!current_tree_)
      {
        response->success = false;
        response->message = "Failed to load tree for region: " + region;
        return;
      }
    }
    else if (current_tree_)
    {
      current_tree_->haltTree();
    }
    else
    {
      build_fixed_tree();
      if (!current_tree_)
      {
        response->success = false;
        response->message = "Failed to load tree for region: " + region;
        return;
      }
    }

    const auto missing = missing_start_dependencies(region);
    if (!missing.empty())
    {
      response->success = false;
      response->message = "Autonomy start blocked; missing dependencies: " +
                          join_strings(missing, ", ");
      blackboard_->set("last_error", response->message);
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    reset_runtime_blackboard(region);
    autonomy_started_ = true;
    response->success = true;
    response->message = "Autonomy started: region=" + region +
                        ", tree_file=" + tree_file_;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
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
    meilin_motion_mode_ = get_parameter("meilin_motion_mode").as_string();
    if (meilin_motion_mode_ != "omni" && meilin_motion_mode_ != "single_axis")
    {
      RCLCPP_WARN(get_logger(),
                  "Invalid meilin_motion_mode=%s; falling back to single_axis",
                  meilin_motion_mode_.c_str());
      meilin_motion_mode_ = "single_axis";
    }

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

  void load_param_config()
  {
    prepare_config_loaded_ = false;
    if (param_config_.empty())
    {
      RCLCPP_WARN(get_logger(), "No param_config provided. Area params not loaded.");
      return;
    }

    const std::string config_path = resolve_param_config_file();
    RCLCPP_INFO(get_logger(), "Loading param config: %s", config_path.c_str());

    YAML::Node cfg;
    try
    {
      cfg = YAML::LoadFile(config_path);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse param config YAML: %s", e.what());
      blackboard_->set("execution_state", std::string{"CONFIG_ERROR"});
      blackboard_->set("last_error",
                       std::string{"Failed to parse param config YAML: "} + e.what());
      return;
    }

    load_match_section(cfg["match"]);
    load_meilin_area_config(cfg["meilin_area"]);
    if (!load_final_area_config(cfg["final_area"]))
    {
      return;
    }

    const auto prepare = cfg["prepare_area"];
    if (!prepare)
    {
      RCLCPP_ERROR(get_logger(), "Param config missing required field: prepare_area.");
      blackboard_->set("execution_state", std::string{"CONFIG_ERROR"});
      blackboard_->set("last_error",
                       std::string{"Missing required config field: prepare_area"});
      return;
    }

    const auto points = prepare["points"];
    const auto spear_pickup = points ? points["spear_pickup"] : YAML::Node{};
    const auto dock_standby = points ? points["dock_standby"] : YAML::Node{};
    const auto motion = prepare["motion"];
    const auto spear_tool = prepare["spear_tool"];
    const auto pick_action = prepare["pick_action"];
    if (!spear_pickup || !dock_standby || !motion || !spear_tool)
    {
      RCLCPP_ERROR(get_logger(),
                   "Param config missing one of: prepare_area.points.spear_pickup, "
                   "prepare_area.points.dock_standby, prepare_area.motion, "
                   "prepare_area.spear_tool.");
      blackboard_->set("execution_state", std::string{"CONFIG_ERROR"});
      blackboard_->set("last_error",
                       std::string{"Missing required prepare_area config section"});
      return;
    }

    load_prepare_point("prepare_spear_pickup_", spear_pickup);
    load_prepare_point("prepare_dock_standby_", dock_standby);

    blackboard_->set("prepare_motion_pid_profile",
                     yaml_value<int>(motion, "pid_profile", 1));
    blackboard_->set("prepare_motion_timeout_sec",
                     yaml_value<double>(motion, "timeout_sec", 10.0));
    blackboard_->set("prepare_motion_retry_attempts",
                     yaml_value<int>(motion, "retry_attempts", 3));

    blackboard_->set("prepare_spear_tool_service_name",
                     yaml_value<std::string>(
                         spear_tool, "service_name", "/ares_tool_node/tool_action"));
    blackboard_->set("prepare_spear_tool_dock_extend_action",
                     yaml_value<std::string>(
                         spear_tool, "dock_extend_action", "dock_extend"));
    blackboard_->set("prepare_spear_tool_dock_extend_timeout_sec",
                     yaml_value<double>(spear_tool, "dock_extend_timeout_sec", 60.0));
    blackboard_->set("prepare_spear_tool_retry_attempts",
                     yaml_value<int>(spear_tool, "retry_attempts", 3));

    std::array<double, 4> args{0.0, 0.0, 0.0, 0.0};
    const auto args_node = spear_tool["args"];
    if (args_node && args_node.IsSequence())
    {
      for (std::size_t i = 0; i < args.size() && i < args_node.size(); ++i)
      {
        args[i] = args_node[i].as<double>();
      }
    }
    blackboard_->set("prepare_spear_tool_arg0", args[0]);
    blackboard_->set("prepare_spear_tool_arg1", args[1]);
    blackboard_->set("prepare_spear_tool_arg2", args[2]);
    blackboard_->set("prepare_spear_tool_arg3", args[3]);

    blackboard_->set("prepare_pick_action_server_name",
                     yaml_value<std::string>(
                         pick_action, "server_name", "/pick_action"));
    blackboard_->set("prepare_pick_action_expected_count",
                     yaml_value<int>(pick_action, "expected_count", 3));
    blackboard_->set("prepare_pick_action_timeout_sec",
                     yaml_value<double>(pick_action, "timeout_sec", 45.0));
    blackboard_->set("prepare_pick_action_retry_attempts",
                     yaml_value<int>(pick_action, "retry_attempts", 1));

    prepare_config_loaded_ = true;
    blackboard_->set("execution_state", std::string{"CONFIG_LOADED"});
    RCLCPP_INFO(get_logger(),
                "PrepareArea params loaded: spear pickup -> grasp -> "
                "parallel(dock standby, dock_extend)");
  }

  void load_match_section(const YAML::Node& match)
  {
    if (!match)
    {
      RCLCPP_WARN(get_logger(), "Param config missing optional field: match.");
      return;
    }

    const auto side = yaml_value<std::string>(match, "side", meilin_side_);
    if (side == "red" || side == "blue")
    {
      meilin_side_ = side;
      if (auto cfg_ptr = blackboard_->get<r2_bt::MeilinConfigPtr>("meilin_config"))
      {
        cfg_ptr->side = side;
      }
      RCLCPP_INFO(get_logger(), "Param config match.side: %s", side.c_str());
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid param_config match.side: %s",
                  side.c_str());
    }
  }

  void load_meilin_area_config(const YAML::Node& meilin)
  {
    if (!meilin)
    {
      RCLCPP_WARN(get_logger(), "Param config missing optional field: meilin_area.");
      return;
    }

    meilin_grid_size_ = yaml_value<double>(meilin, "grid_size", meilin_grid_size_);
    meilin_initial_height_ =
        yaml_value<double>(meilin, "initial_height", meilin_initial_height_);
    meilin_initial_yaw_ = r2_bt::meilin_normalize_angle(
        yaml_value<double>(meilin, "initial_yaw", meilin_initial_yaw_));
    meilin_grasp_distance_ =
        yaml_value<double>(meilin, "grasp_distance", meilin_grasp_distance_);
    meilin_yaw_tolerance_ =
        yaml_value<double>(meilin, "yaw_tolerance", meilin_yaw_tolerance_);
    meilin_height_tolerance_ =
        yaml_value<double>(meilin, "height_tolerance", meilin_height_tolerance_);
    meilin_pose_timeout_sec_ =
        yaml_value<double>(meilin, "pose_timeout_sec", meilin_pose_timeout_sec_);
    meilin_cell_center_tolerance_ =
        yaml_value<double>(meilin, "cell_center_tolerance", meilin_cell_center_tolerance_);
    meilin_default_align_timeout_ =
        yaml_value<double>(meilin, "default_align_timeout", meilin_default_align_timeout_);
    meilin_default_suspension_timeout_ =
        yaml_value<double>(meilin, "default_suspension_timeout",
                           meilin_default_suspension_timeout_);
    meilin_default_grasp_timeout_ =
        yaml_value<double>(meilin, "default_grasp_timeout", meilin_default_grasp_timeout_);
    meilin_suspension_normal_height_ =
        yaml_value<double>(meilin, "suspension_normal_height",
                           meilin_suspension_normal_height_);
    meilin_motion_mode_ =
        yaml_value<std::string>(meilin, "motion_mode", meilin_motion_mode_);
    if (meilin_motion_mode_ != "omni" && meilin_motion_mode_ != "single_axis")
    {
      RCLCPP_WARN(get_logger(),
                  "Ignoring invalid meilin_area.motion_mode: %s",
                  meilin_motion_mode_.c_str());
      meilin_motion_mode_ = "single_axis";
    }

    const auto origin = yaml_double_vector(meilin["grid_origin"]);
    if (origin.size() >= 2)
    {
      meilin_origin_x_ = origin[0];
      meilin_origin_y_ = origin[1];
    }

    const auto initial_grid = yaml_int_vector(meilin["initial_grid"]);
    if (initial_grid.size() >= 2)
    {
      meilin_initial_row_ = static_cast<int>(initial_grid[0]);
      meilin_initial_col_ = static_cast<int>(initial_grid[1]);
    }

    sync_meilin_blackboard();
    RCLCPP_INFO(get_logger(),
                "Meilin params loaded from param_config: side=%s, grid=%.3f, "
                "origin=(%.3f, %.3f), initial=(%d, %d), motion_mode=%s",
                meilin_side_.c_str(), meilin_grid_size_,
                meilin_origin_x_, meilin_origin_y_,
                meilin_initial_row_, meilin_initial_col_,
                meilin_motion_mode_.c_str());
  }

  bool load_final_area_config(const YAML::Node& final_cfg)
  {
    if (!final_cfg)
    {
      RCLCPP_WARN(get_logger(), "Param config missing optional field: final_area.");
      return true;
    }

    const auto standby = final_cfg["standby"];
    const auto targets = final_cfg["targets"];
    if (!standby || !targets)
    {
      blackboard_->set("execution_state", std::string{"CONFIG_ERROR"});
      blackboard_->set("last_error",
                       std::string{"Param config missing final_area.standby/targets"});
      RCLCPP_ERROR(get_logger(),
                   "Param config missing one of: final_area.standby, final_area.targets.");
      return false;
    }

    blackboard_->set("final_deck_topic",
                     yaml_value<std::string>(
                         final_cfg, "deck_topic", "/aruco_comm/tx_id"));
    blackboard_->set("final_place_signal",
                     yaml_value<int>(final_cfg, "place_signal", 7));
    blackboard_->set("final_command_timeout_sec",
                     yaml_value<double>(final_cfg, "command_timeout_sec", 0.0));
    blackboard_->set("final_place_signal_timeout_sec",
                     yaml_value<double>(
                         final_cfg, "place_signal_timeout_sec", 0.0));
    blackboard_->set("final_post_place_wait_sec",
                     yaml_value<double>(final_cfg, "post_place_wait_sec", 3.0));
    load_final_chassis_height_config(final_cfg["chassis_height"]);
    const double standby_x = yaml_value<double>(standby, "target_x", 0.0);
    const double standby_y = yaml_value<double>(standby, "target_y", 0.0);
    const double standby_yaw = yaml_value<double>(standby, "target_yaw", 0.0);
    const int standby_pid_profile = yaml_value<int>(standby, "pid_profile", 1);
    const double standby_timeout_sec = yaml_value<double>(standby, "timeout_sec", 60.0);
    blackboard_->set("final_standby_target_x",
                     standby_x);
    blackboard_->set("final_standby_target_y",
                     standby_y);
    blackboard_->set("final_standby_target_yaw",
                     standby_yaw);
    blackboard_->set("final_standby_pid_profile",
                     standby_pid_profile);
    blackboard_->set("final_standby_timeout_sec",
                     standby_timeout_sec);

    const double side_sign = standby_y < 0.0 ? -1.0 : 1.0;
    const auto standby_waypoints = final_cfg["standby_waypoints"];
    const int configured_waypoint_count =
        yaml_value<int>(final_cfg, "standby_waypoint_count", 0);
    blackboard_->set("final_standby_waypoint_count", configured_waypoint_count);
    auto final_waypoints = std::make_shared<r2_bt::FinalWaypointList>();
    if (standby_waypoints && standby_waypoints.IsSequence() &&
        standby_waypoints.size() > 0)
    {
      std::size_t waypoint_count = standby_waypoints.size();
      if (configured_waypoint_count > 0)
      {
        const auto requested_count =
            static_cast<std::size_t>(configured_waypoint_count);
        if (requested_count > waypoint_count)
        {
          RCLCPP_WARN(get_logger(),
                      "final_area.standby_waypoint_count=%zu exceeds "
                      "standby_waypoints size=%zu; using all configured waypoints.",
                      requested_count, waypoint_count);
        }
        waypoint_count = std::min(waypoint_count, requested_count);
      }
      else if (configured_waypoint_count < 0)
      {
        RCLCPP_WARN(get_logger(),
                    "final_area.standby_waypoint_count=%d is invalid; "
                    "using all configured waypoints.",
                    configured_waypoint_count);
      }

      for (std::size_t i = 0; i < waypoint_count; ++i)
      {
        const bool is_wp1 = i == 0;
        const bool is_wp2 = i == 1;
        final_waypoints->push_back(make_final_waypoint(
            standby_waypoints[i],
            is_wp1 ? 7.80 : (is_wp2 ? 8.60 : standby_x),
            is_wp1 ? side_sign * 0.60 : (is_wp2 ? side_sign * 1.00 : standby_y),
            standby_yaw,
            is_wp1 || is_wp2 ? 1 : standby_pid_profile,
            is_wp1 || is_wp2 ? 30.0 : standby_timeout_sec));
      }
    }
    else
    {
      std::size_t waypoint_count = 3;
      if (configured_waypoint_count > 0)
      {
        const auto requested_count =
            static_cast<std::size_t>(configured_waypoint_count);
        if (requested_count > waypoint_count)
        {
          RCLCPP_WARN(get_logger(),
                      "final_area.standby_waypoint_count=%zu requested but "
                      "no standby_waypoints list is configured; using the "
                      "3 default waypoints.",
                      requested_count);
        }
        waypoint_count = std::min(waypoint_count, requested_count);
      }
      else if (configured_waypoint_count < 0)
      {
        RCLCPP_WARN(get_logger(),
                    "final_area.standby_waypoint_count=%d is invalid; "
                    "using the 3 default waypoints.",
                    configured_waypoint_count);
      }

      if (waypoint_count >= 1)
      {
        final_waypoints->push_back(
            make_final_waypoint(YAML::Node{}, 7.80, side_sign * 0.60,
                                standby_yaw, 1, 30.0));
      }
      if (waypoint_count >= 2)
      {
        final_waypoints->push_back(
            make_final_waypoint(YAML::Node{}, 8.60, side_sign * 1.00,
                                standby_yaw, 1, 30.0));
      }
      if (waypoint_count >= 3)
      {
        final_waypoints->push_back(
            make_final_waypoint(YAML::Node{}, standby_x, standby_y, standby_yaw,
                                standby_pid_profile, standby_timeout_sec));
      }
    }
    blackboard_->set("final_standby_waypoints", final_waypoints);
    RCLCPP_INFO(get_logger(), "FinalArea standby waypoints loaded: %zu",
                final_waypoints->size());

    const auto place_action = final_cfg["place_action"];
    blackboard_->set("final_place_action_service_name",
                     yaml_value<std::string>(
                         place_action, "service_name", "/ares_tool_node/tool_action"));
    blackboard_->set("final_place_mid_action",
                     yaml_value<std::string>(place_action, "mid_action", "arm_place_mid"));
    blackboard_->set("final_place_high_action",
                     yaml_value<std::string>(place_action, "high_action", "arm_place_high"));
    blackboard_->set("final_place_action_timeout_sec",
                     yaml_value<double>(place_action, "timeout_sec", 30.0));
    blackboard_->set("final_place_action_retry_attempts",
                     yaml_value<int>(place_action, "retry_attempts", 3));
    std::array<double, 4> place_args{0.0, 0.0, 0.0, 0.0};
    const auto place_args_node = place_action["args"];
    if (place_args_node && place_args_node.IsSequence())
    {
      for (std::size_t i = 0; i < place_args.size() && i < place_args_node.size(); ++i)
      {
        place_args[i] = place_args_node[i].as<double>();
      }
    }
    blackboard_->set("final_place_action_arg0", place_args[0]);
    blackboard_->set("final_place_action_arg1", place_args[1]);
    blackboard_->set("final_place_action_arg2", place_args[2]);
    blackboard_->set("final_place_action_arg3", place_args[3]);

    for (const auto& key : {"2_left", "2_mid", "2_right",
                            "3_left", "3_mid", "3_right"})
    {
      const auto target = targets[key];
      if (!target)
      {
        blackboard_->set("execution_state", std::string{"CONFIG_ERROR"});
        blackboard_->set("last_error",
                         std::string{"Missing final target config: "} + key);
        RCLCPP_ERROR(get_logger(), "Param config missing final_area.targets.%s", key);
        return false;
      }
      const std::string prefix = "final_target_" + std::string{key} + "_";
      blackboard_->set(prefix + "target_x",
                       yaml_value<double>(target, "target_x", 0.0));
      blackboard_->set(prefix + "target_y",
                       yaml_value<double>(target, "target_y", 0.0));
      blackboard_->set(prefix + "target_yaw",
                       yaml_value<double>(target, "target_yaw", 0.0));
      blackboard_->set(prefix + "pid_profile",
                       yaml_value<int>(target, "pid_profile", 1));
      blackboard_->set(prefix + "timeout_sec",
                       yaml_value<double>(target, "timeout_sec", 30.0));
    }

    blackboard_->set("execution_state", std::string{"CONFIG_LOADED"});
    RCLCPP_INFO(get_logger(), "FinalArea params loaded from param_config.");
    return true;
  }

  void sync_meilin_blackboard()
  {
    if (auto cfg_ptr = blackboard_->get<r2_bt::MeilinConfigPtr>("meilin_config"))
    {
      cfg_ptr->side = meilin_side_;
      cfg_ptr->grid_size = meilin_grid_size_;
      cfg_ptr->grid_origin_x = meilin_origin_x_;
      cfg_ptr->grid_origin_y = meilin_origin_y_;
      cfg_ptr->grasp_distance = meilin_grasp_distance_;
      cfg_ptr->yaw_tolerance = meilin_yaw_tolerance_;
      cfg_ptr->height_tolerance = meilin_height_tolerance_;
      cfg_ptr->align_timeout_sec = meilin_default_align_timeout_;
      cfg_ptr->suspension_timeout_sec = meilin_default_suspension_timeout_;
      cfg_ptr->arm_timeout_sec = meilin_default_grasp_timeout_;
      cfg_ptr->suspension_normal_height = meilin_suspension_normal_height_;
      cfg_ptr->pose_timeout_sec = meilin_pose_timeout_sec_;
      cfg_ptr->cell_center_tolerance = meilin_cell_center_tolerance_;
      cfg_ptr->motion_mode = meilin_motion_mode_;
      cfg_ptr->rows = meilin_rows_;
      cfg_ptr->cols = meilin_cols_;
    }

    blackboard_->set("meilin_current_row", meilin_initial_row_);
    blackboard_->set("meilin_current_col", meilin_initial_col_);
    blackboard_->set("meilin_current_height", meilin_initial_height_);
    blackboard_->set("meilin_current_yaw", meilin_initial_yaw_);
    blackboard_->set("meilin_pose_is_cell_center", true);
    blackboard_->set("meilin_suspension_offset", 0.0);
    blackboard_->set("meilin_entry_move_pending", false);
  }

  void load_prepare_point(const std::string& prefix, const YAML::Node& node)
  {
    blackboard_->set(prefix + "target_x", yaml_value<double>(node, "x", 0.0));
    blackboard_->set(prefix + "target_y", yaml_value<double>(node, "y", 0.0));
    blackboard_->set(prefix + "target_yaw", yaml_value<double>(node, "yaw", 0.0));
    blackboard_->set(prefix + "frame_id",
                     yaml_value<std::string>(node, "frame_id", "map"));
  }

  r2_bt::FinalWaypoint make_final_waypoint(const YAML::Node& node,
                                           double fallback_x,
                                           double fallback_y,
                                           double fallback_yaw,
                                           int fallback_pid_profile,
                                           double fallback_timeout_sec)
  {
    r2_bt::FinalWaypoint waypoint;
    waypoint.target_x = yaml_value<double>(node, "target_x", fallback_x);
    waypoint.target_y = yaml_value<double>(node, "target_y", fallback_y);
    waypoint.target_yaw = yaml_value<double>(node, "target_yaw", fallback_yaw);
    waypoint.pid_profile = yaml_value<int>(node, "pid_profile", fallback_pid_profile);
    waypoint.timeout_sec =
        yaml_value<double>(node, "timeout_sec", fallback_timeout_sec);
    return waypoint;
  }

  void load_final_chassis_height_config(const YAML::Node& node)
  {
    const auto topic =
        yaml_value<std::string>(node, "topic", "t0x0112_final");
    blackboard_->set("final_chassis_height_topic", topic);
    blackboard_->set("final_chassis_height_wp1",
                     yaml_value<double>(node, "waypoint1_height", 60.0));
    blackboard_->set("final_chassis_height_wp3",
                     yaml_value<double>(node, "waypoint3_height", 20.0));
    blackboard_->set("final_chassis_height_settle_sec",
                     yaml_value<double>(node, "settle_sec", 0.2));

    auto publisher =
        create_publisher<std_msgs::msg::Float32MultiArray>(topic, 10);
    blackboard_->set("final_chassis_height_publisher", publisher);
    RCLCPP_INFO(get_logger(),
                "FinalArea chassis height publisher initialized: topic=%s",
                topic.c_str());
  }

  template <typename T>
  static T yaml_value(const YAML::Node& node,
                      const std::string& key,
                      const T& fallback)
  {
    if (!node)
    {
      return fallback;
    }
    const auto child = node[key];
    if (!child)
    {
      return fallback;
    }
    return child.as<T>();
  }

  static std::vector<double> yaml_double_vector(const YAML::Node& node)
  {
    std::vector<double> result;
    if (!node || !node.IsSequence())
    {
      return result;
    }
    for (const auto& item : node)
    {
      result.push_back(item.as<double>());
    }
    return result;
  }

  static std::vector<int64_t> yaml_int_vector(const YAML::Node& node)
  {
    std::vector<int64_t> result;
    if (!node || !node.IsSequence())
    {
      return result;
    }
    for (const auto& item : node)
    {
      result.push_back(item.as<int64_t>());
    }
    return result;
  }

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

    // FinalArea 参数
    if (!cfg.contains("final") || !cfg["final"].is_object())
    {
      blackboard_->set("execution_state", std::string{"CONFIG_ERROR"});
      blackboard_->set("last_error", std::string{"Match config missing final section"});
      RCLCPP_ERROR(get_logger(), "Match config missing required section: final.");
      return;
    }

    const auto& final_cfg = cfg["final"];
    const auto standby = final_cfg.value("standby", nlohmann::json::object());
    const auto targets = final_cfg.value("targets", nlohmann::json::object());
    if (!standby.is_object() || !targets.is_object())
    {
      blackboard_->set("execution_state", std::string{"CONFIG_ERROR"});
      blackboard_->set("last_error",
                       std::string{"Match config missing final.standby/final.targets"});
      RCLCPP_ERROR(get_logger(),
                   "Match config missing one of: final.standby, final.targets.");
      return;
    }

    blackboard_->set("final_deck_topic",
                     final_cfg.value("deck_topic", "/aruco_comm/tx_id"));
    blackboard_->set("final_place_signal", final_cfg.value("place_signal", 7));
    blackboard_->set("final_command_timeout_sec",
                     final_cfg.value("command_timeout_sec", 0.0));
    blackboard_->set("final_place_signal_timeout_sec",
                     final_cfg.value("place_signal_timeout_sec", 0.0));
    blackboard_->set("final_post_place_wait_sec",
                     final_cfg.value("post_place_wait_sec", 3.0));
    load_final_chassis_height_config(YAML::Node{});

    const double standby_x = standby.value("target_x", 0.0);
    const double standby_y = standby.value("target_y", 0.0);
    const double standby_yaw = standby.value("target_yaw", 0.0);
    const int standby_pid_profile = standby.value("pid_profile", 1);
    const double standby_timeout_sec = standby.value("timeout_sec", 60.0);
    blackboard_->set("final_standby_target_x", standby_x);
    blackboard_->set("final_standby_target_y", standby_y);
    blackboard_->set("final_standby_target_yaw", standby_yaw);
    blackboard_->set("final_standby_pid_profile", standby_pid_profile);
    blackboard_->set("final_standby_timeout_sec", standby_timeout_sec);

    const double side_sign = standby_y < 0.0 ? -1.0 : 1.0;
    blackboard_->set("final_standby_waypoint_count", 0);
    auto final_waypoints = std::make_shared<r2_bt::FinalWaypointList>();
    final_waypoints->push_back(make_final_waypoint(
        YAML::Node{}, 7.80, side_sign * 0.60, standby_yaw, 1, 30.0));
    final_waypoints->push_back(make_final_waypoint(
        YAML::Node{}, 8.60, side_sign * 1.00, standby_yaw, 1, 30.0));
    final_waypoints->push_back(make_final_waypoint(
        YAML::Node{}, standby_x, standby_y, standby_yaw,
        standby_pid_profile, standby_timeout_sec));
    blackboard_->set("final_standby_waypoints", final_waypoints);

    blackboard_->set("final_place_action_service_name",
                     final_cfg.value("place_service_name", "/ares_tool_node/tool_action"));
    blackboard_->set("final_place_mid_action",
                     final_cfg.value("place_mid_action", "arm_place_mid"));
    blackboard_->set("final_place_high_action",
                     final_cfg.value("place_high_action", "arm_place_high"));
    blackboard_->set("final_place_action_timeout_sec",
                     final_cfg.value("place_action_timeout_sec", 30.0));
    blackboard_->set("final_place_action_retry_attempts",
                     final_cfg.value("place_action_retry_attempts", 3));
    blackboard_->set("final_place_action_arg0", 0.0);
    blackboard_->set("final_place_action_arg1", 0.0);
    blackboard_->set("final_place_action_arg2", 0.0);
    blackboard_->set("final_place_action_arg3", 0.0);

    const auto load_final_target =
        [this, &targets](const std::string& key) -> bool {
          if (!targets.contains(key) || !targets[key].is_object())
          {
            blackboard_->set("last_error",
                             std::string{"Missing final target config: "} + key);
            return false;
          }
          const auto& target = targets[key];
          const std::string prefix = "final_target_" + key + "_";
          blackboard_->set(prefix + "target_x", target.value("target_x", 0.0));
          blackboard_->set(prefix + "target_y", target.value("target_y", 0.0));
          blackboard_->set(prefix + "target_yaw", target.value("target_yaw", 0.0));
          blackboard_->set(prefix + "pid_profile", target.value("pid_profile", 1));
          blackboard_->set(prefix + "timeout_sec", target.value("timeout_sec", 30.0));
          return true;
        };

    for (const auto& key : {"2_left", "2_mid", "2_right",
                            "3_left", "3_mid", "3_right"})
    {
      if (!load_final_target(key))
      {
        blackboard_->set("execution_state", std::string{"CONFIG_ERROR"});
        RCLCPP_ERROR(get_logger(), "Match config missing final.targets.%s",
                     key);
        return;
      }
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

  std::string resolve_param_config_file() const
  {
    if (!param_config_.empty() && param_config_.front() == '/')
      return param_config_;
    auto share_dir = ament_index_cpp::get_package_share_directory("r2_bt");
    if (param_config_.find("config/") == 0)
      return share_dir + "/" + param_config_;
    return share_dir + "/config/" + param_config_;
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
                                   bool entry_move,
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

    // 3. 如果相邻且有高度差，校验方向（无后退）。
    // 新 web 路径的第一个 move 是梅林入口点，不从当前 blackboard 初始格爬过去。
    const double h_diff = map_height - state.height;
    if (!entry_move && std::abs(h_diff) > meilin_height_tolerance_)
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
    seg.debug_name = entry_move
        ? "entry_move#" + std::to_string(idx)
        : "move#" + std::to_string(idx);
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
      if (i == 0 && action_type != 0)
      {
        error = "first Meilin action must be move entry point";
        return {};
      }

      r2_bt::Segment seg;
      if (action_type == 0)
      {
        // move
        const bool entry_move = i == 0;
        seg = make_move_segment(i, row, col, arg3, yaw, sim, entry_move, error);
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
    blackboard_->set("meilin_pose_frame_id", msg->header.frame_id);
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
    blackboard_->set("meilin_entry_move_pending", true);
    blackboard_->set("execution_state", std::string{"MF_PLAN_READY"});

    RCLCPP_INFO(get_logger(), "Meilin plan accepted: plan_id=%s, actions=%zu",
                plan_id.c_str(), segments.size() - 1);
  }

  void buffer_poll_callback()
  {
    if (!autonomy_started_)
      return;

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
          segment.timeout_sec = item.value("timeout_sec", 30.0);
        }
        else if (segment.segment_type == "DOCK")
        {
          segment.timeout_sec = item.value("timeout_sec", 10.0);
        }
        else if (segment.segment_type == "MOVE2")
        {
          load_move_target(item, segment);
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
          segment.timeout_sec = item.value("timeout_sec", 30.0);
          segment.arm_command = 7;
          segment.wait_result = true;
        }
        else if (segment.segment_type == "FINAL_MOVE2")
        {
          load_move_target(item, segment);
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
    if (!autonomy_started_)
      return;

    if (current_tree_)
    {
      try
      {
        blackboard_->set("ros_node", shared_from_this());
        auto status = current_tree_->tickOnce();
        if (status == BT::NodeStatus::FAILURE)
        {
          std::string last_error;
          std::string active_action;
          std::string segment_debug_name;
          (void)blackboard_->get("last_error", last_error);
          (void)blackboard_->get("active_action", active_action);
          (void)blackboard_->get("segment_debug_name", segment_debug_name);
          blackboard_->set("execution_state", std::string{"MISSION_FAILED"});
          autonomy_started_ = false;
          RCLCPP_ERROR(get_logger(),
                       "BT returned FAILURE; segment=%s active_action=%s last_error=%s; "
                       "autonomy paused until /bt_engine/start_autonomy",
                       segment_debug_name.c_str(), active_action.c_str(),
                       last_error.c_str());
        }
        else if (status == BT::NodeStatus::SUCCESS)
        {
          std::string state;
          if (!blackboard_->get("execution_state", state) ||
              state.find("SUCCESS") == std::string::npos)
          {
            blackboard_->set("execution_state", std::string{"MISSION_SUCCESS"});
          }
          blackboard_->set("active_action", std::string{});
          autonomy_started_ = false;
          current_tree_->haltTree();
          RCLCPP_INFO(get_logger(),
                      "BT returned SUCCESS; autonomy paused with current success state");
        }
      }
      catch (const std::exception& e)
      {
        blackboard_->set("last_error", std::string{"BT tick exception: "} + e.what());
        blackboard_->set("execution_state", std::string{"MISSION_FAILED"});
        autonomy_started_ = false;
        RCLCPP_ERROR(get_logger(), "BT tick exception: %s", e.what());
      }
    }
  }

  // =========================================================================
  // 成员变量
  // =========================================================================

  using MoveToPoseAction = action_of_motion_interfaces::action::MoveToPose;
  using PickSequenceAction = pick_action_interfaces::action::PickSequence;
  using StepMotionAction = r2_interfaces::action::StepMotionControl;
  using ToolActionSrv = r2_interfaces::srv::ToolAction;

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Tree> current_tree_;
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr segment_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr meilin_pose_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr buffer_poll_timer_;
  rclcpp::Service<r2_interfaces::srv::StartAutonomy>::SharedPtr start_autonomy_service_;

  // 暂存区 Service Client
  rclcpp::Client<r2_interfaces::srv::GetActionSeq>::SharedPtr buffer_client_;
  rclcpp::Client<ToolActionSrv>::SharedPtr tool_client_;
  rclcpp_action::Client<MoveToPoseAction>::SharedPtr move_to_pose_client_;
  rclcpp_action::Client<PickSequenceAction>::SharedPtr pick_action_client_;
  rclcpp_action::Client<StepMotionAction>::SharedPtr step_motion_client_;

  unsigned groot2_port_ = 1667;
  std::string segment_topic_;
  std::string mf_action_topic_;
  std::string buffer_service_;
  std::string meilin_pose_topic_;
  std::string tree_file_;
  std::string param_config_;
  std::string match_config_;
  std::string default_region_ = "full";
  bool prepare_config_loaded_ = false;
  bool autostart_ = true;
  bool autonomy_started_ = true;
  bool require_map_relocalization_ = false;
  double localization_timeout_sec_ = 1.0;

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
  std::string meilin_motion_mode_ = "single_axis";
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
