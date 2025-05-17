cmake_minimum_required(VERSION 3.8)
project(actionlib_example_pkg)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(actionlib_pkg REQUIRED)  # 假设actionlib_pkg已正确导出接口

# 生成客户端可执行文件
add_executable(action_client_node
  src/actionlib_client_node.cpp
)
ament_target_dependencies(action_client_node
  rclcpp
  rclcpp_action
  actionlib_pkg
)

# 生成服务端可执行文件
add_executable(actionlib_server_node
  src/actionlib_server_node.cpp
)
ament_target_dependencies(actionlib_server_node
  rclcpp
  rclcpp_action
  actionlib_pkg
)

# 安装可执行文件
install(TARGETS
  action_client_node
  actionlib_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出包配置
ament_package()