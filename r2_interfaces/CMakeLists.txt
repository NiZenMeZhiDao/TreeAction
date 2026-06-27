cmake_minimum_required(VERSION 3.8)
project(r2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmAction.action"
  "action/GoToPose.action"
  "action/SpearAction.action"
  "action/StepMotionControl.action"
  "action/SuspensionControl.action"
  "srv/GetActionSeq.srv"
  "srv/StartAutonomy.srv"
  "srv/ToolAction.srv"
  DEPENDENCIES geometry_msgs action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
