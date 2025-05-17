cmake_minimum_required(VERSION 3.8)
project(parameter_server_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(get_parameter_node src/get_parameter_node.cpp)
ament_target_dependencies(get_parameter_node rclcpp std_msgs)

install(TARGETS
  get_parameter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )  

ament_package()
