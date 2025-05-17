cmake_minimum_required(VERSION 3.8)
project(service_client_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)  # 确保 geometry_msgs 被正确加载
find_package(rosidl_default_generators REQUIRED)


# 添加服务文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ServiceClientExMsg.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()
