cmake_minimum_required(VERSION 3.8)
project(service_example_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(service_client_pkg REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# 添加可执行文件（如果需要）
add_executable(service_example_node src/service_example_node.cpp)
ament_target_dependencies(service_example_node rclcpp std_msgs service_client_pkg)

add_executable(client_example_node src/client_example_node.cpp)
ament_target_dependencies(client_example_node rclcpp std_msgs service_client_pkg)

# 安装目标
install(TARGETS
  service_example_node
  client_example_node
  DESTINATION lib/${PROJECT_NAME})


# 导出依赖项
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)

ament_package()
