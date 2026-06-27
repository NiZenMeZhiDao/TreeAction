cmake_minimum_required(VERSION 3.8)
project(ares_tool_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
<<<<<<< Updated upstream
find_package(rclcpp_action REQUIRED)
find_package(r2_interfaces REQUIRED)
=======
find_package(ares_tool_interfaces REQUIRED)
>>>>>>> Stashed changes
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

add_executable(tool_node src/tool_node.cpp src/ares_usb.cpp)
target_include_directories(tool_node PRIVATE include)
<<<<<<< Updated upstream
ament_target_dependencies(tool_node rclcpp rclcpp_action r2_interfaces)
=======
ament_target_dependencies(tool_node rclcpp ares_tool_interfaces)
>>>>>>> Stashed changes
target_link_libraries(tool_node PkgConfig::LIBUSB)

install(TARGETS tool_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
