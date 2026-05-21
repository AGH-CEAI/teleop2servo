cmake_minimum_required(VERSION 3.8)
project(teleop2servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(magic_enum REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(teleop_gamepad_node SHARED src/teleop_gamepad_node.cpp src/teleop_publisher.cpp
                                       src/print_helper.cpp)

target_include_directories(
  teleop_gamepad_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>)

target_compile_definitions(teleop_gamepad_node PRIVATE "RCLCPP_COMPONENTS_BUILDING_DLL")

ament_target_dependencies(
  teleop_gamepad_node
  control_msgs
  geometry_msgs
  magic_enum
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs)

target_link_libraries(teleop_gamepad_node magic_enum::magic_enum)

rclcpp_components_register_node(
  teleop_gamepad_node
  PLUGIN
  "teleop2servo::TeleopGamepadNode"
  EXECUTABLE
  teleop_gamepad_node_exec)

install(
  TARGETS teleop_gamepad_node teleop_gamepad_node_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
