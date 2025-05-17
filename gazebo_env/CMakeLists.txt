cmake_minimum_required(VERSION 3.8)
project(gazebo_env)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装URDF文件到正确位置
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 安装 worlds 文件夹到 share/gazebo_env/worlds/
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
ament_package()
