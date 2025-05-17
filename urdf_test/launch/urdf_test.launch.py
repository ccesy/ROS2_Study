from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    package_name = 'urdf_test'
    pkg_path = get_package_share_directory('urdf_test')

    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'my_car.urdf')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    
    # with open(urdf_path, 'r') as file:
    #     robot_description = file.read()

    robot_description = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(value=Command(['xacro ', xacro_file]), value_type=str  # 明确指定为字符串类型
    )
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description  # 直接读取URDF内容
            }]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory(package_name),
            'urdf',
            'config.rviz'  # 可选：预配置的RViz配置
            )]
        )
    ])