cmake_minimum_required(VERSION 3.8)
project(r2_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ares_tool_interfaces REQUIRED)
find_package(action_of_motion_interfaces REQUIRED)
find_package(pick_action_interfaces REQUIRED)
find_package(r2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

#------------------------------------------------------------
# BT nodes library (C++)
#------------------------------------------------------------
add_library(r2_bt_nodes SHARED
  src/nodes/ares_tool_action.cpp
  src/nodes/initial_place_kfs.cpp
  src/nodes/is_segment_type.cpp
  src/nodes/is_string_empty.cpp
  src/nodes/is_string_non_empty.cpp
  src/nodes/meilin_fetch.cpp
  src/nodes/meilin_move.cpp
  src/nodes/meilin_move_plan.cpp
  src/nodes/move_through_final_waypoints.cpp
  src/nodes/move_to_pose.cpp
  src/nodes/pop_next_meilin_segment.cpp
  src/nodes/pop_next_segment.cpp
  src/nodes/publish_chassis_height.cpp
  src/nodes/retry_segment.cpp
  src/nodes/set_debug_status.cpp
  src/nodes/switch_segment_type.cpp
  src/nodes/for_each_segment.cpp
  src/nodes/go_to_pose.cpp
  src/nodes/place_object_placeholder.cpp
  src/nodes/place_kfs_action.cpp
  src/nodes/pick_action.cpp
  src/nodes/select_final_target.cpp
  src/nodes/suspension_control.cpp
  src/nodes/step_motion_control.cpp
  src/nodes/wait_arm_idle.cpp
  src/nodes/wait_for_int_signal.cpp
  src/nodes/wait_for_r1_signal.cpp
  src/nodes/wait_seconds.cpp
)

target_include_directories(r2_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(r2_bt_nodes
  rclcpp
  rclcpp_action
  behaviortree_cpp
  ares_tool_interfaces
  action_of_motion_interfaces
  pick_action_interfaces
  r2_interfaces
  geometry_msgs
  std_msgs
)

#------------------------------------------------------------
# BT 引擎节点
#------------------------------------------------------------
add_executable(r2_bt_engine
  src/bt_engine_node.cpp
)

target_include_directories(r2_bt_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(r2_bt_engine
  ament_index_cpp
  rclcpp
  rclcpp_action
  behaviortree_cpp
  ares_tool_interfaces
  action_of_motion_interfaces
  pick_action_interfaces
  r2_interfaces
  geometry_msgs
  std_msgs
)

target_link_libraries(r2_bt_engine r2_bt_nodes)
target_link_libraries(r2_bt_engine nlohmann_json::nlohmann_json)
target_link_libraries(r2_bt_engine yaml-cpp)

#------------------------------------------------------------
# 安装
#------------------------------------------------------------
install(TARGETS
  r2_bt_nodes
  r2_bt_engine
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(r2_bt_nodes)
ament_export_dependencies(ares_tool_interfaces pick_action_interfaces r2_interfaces)

ament_package()
