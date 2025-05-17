cmake_minimum_required(VERSION 3.8)
project(tf_test_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
add_executable(tf_listener src/tf_listener.cpp)

ament_target_dependencies(
   tf_broadcaster
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)

ament_target_dependencies(
    tf_listener
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
)


install(TARGETS
   tf_broadcaster 
   tf_listener
   DESTINATION lib/${PROJECT_NAME})

ament_package()
