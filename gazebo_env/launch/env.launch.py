from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # 定义包名和文件路径
    package_name = "gazebo_env"
    xacro_file = "urdf/my_car.xacro"  # 统一使用xacro文件
    world_file = "worlds/model.world"

    # 获取完整路径
    pkg_path = get_package_share_directory(package_name)
    xacro_path = os.path.join(pkg_path, xacro_file)
    
    # 创建robot_description
    robot_description = ParameterValue(
        value=Command(['xacro ', xacro_path]),
        value_type=str
    )

    # 1. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            ])
        ]),
        launch_arguments={
            "world": PathJoinSubstitution([
                FindPackageShare(package_name),
                world_file
            ]),
            "verbose": "true",
            "pause": "false",
        }.items(),
    )

    # 2. 启动 robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description
        }],
    )

    # 3. ROS-Gazebo 桥接
    ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/joint_cmd@ros_gz_interfaces/msg/JointCmd[gz.msgs.JointCmd'
        ],
        output='screen'
    )

    # 4. 在 Gazebo 中生成模型
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "my_robot",
            "-topic", "robot_description",
            "-x", "0.9",  # X 坐标（单位：米）
            "-y", "5.0",  # Y 坐标
            "-z", "0.1",  # Z 坐标（离地高度）
        ],
        output="screen",
    )

    # 5. 关节状态发布器
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        ros_bridge,
        joint_state_publisher,
        spawn_entity,
    ])