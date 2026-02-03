cmake_minimum_required(VERSION 3.8)
project(teleop2servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(teleop_keyboard_node src/teleop2servo_node.cpp)
ament_target_dependencies(
  teleop_keyboard_node
  rclcpp
  std_msgs
  control_msgs
  geometry_msgs)

# TESTING
add_executable(tester_sub src/tester_sub.cpp)
ament_target_dependencies(tester_sub rclcpp std_msgs)

install(TARGETS teleop_keyboard_node tester_sub DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
