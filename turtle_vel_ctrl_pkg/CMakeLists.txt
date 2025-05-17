cmake_minimum_required(VERSION 3.8)
project(turtle_vel_ctrl_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtle_vel_ctrl_node src/turtle_vel_ctrl_node.cpp)
ament_target_dependencies(turtle_vel_ctrl_node rclcpp geometry_msgs)

add_executable(turtle_vel_rece_node src/turtle_vel_rece_node.cpp)
ament_target_dependencies(turtle_vel_rece_node rclcpp geometry_msgs)

install(TARGETS
  turtle_vel_ctrl_node
  turtle_vel_rece_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
