cmake_minimum_required(VERSION 3.8)
project(actionlib_pkg)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# 生成Action接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ActionlibExMsg.action"
  DEPENDENCIES action_msgs
)

# 导出依赖和头文件
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rosidl_default_runtime
)
ament_export_include_directories(include)

# 安装自动生成的接口库
install(
  TARGETS ${PROJECT_NAME}__rosidl_generator_c
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 生成包配置
ament_package()