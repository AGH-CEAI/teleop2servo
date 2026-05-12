cmake_minimum_required(VERSION 3.8)
project(teleop2servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(gamepad_teleop_node SHARED src/gamepad_teleop_node.cpp src/print_helper.cpp)

target_include_directories(
  gamepad_teleop_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>)

target_compile_definitions(gamepad_teleop_node PRIVATE "RCLCPP_COMPONENTS_BUILDING_DLL")

ament_target_dependencies(
  gamepad_teleop_node
  rclcpp
  rclcpp_components
  sensor_msgs
  control_msgs
  geometry_msgs)

rclcpp_components_register_nodes(gamepad_teleop_node "teleop2servo::GamepadTeleopNode")

install(
  TARGETS gamepad_teleop_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
