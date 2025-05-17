# import sys
# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# sys.path.append(os.path.join(get_package_share_directory('parameter_server_pkg'), 'launch'))

# def generate_launch_description():
#     # 获取 YAML 文件的绝对路径
#     yaml_file_path = get_package_share_directory('parameter_server_pkg') + '/para_setting.yaml'

#     return LaunchDescription([
#         Node(
#             package='parameter_server_pkg',
#             executable='get_parameter_node',
#             name='get_parameter_node',
#             output='screen',
#             parameters=[yaml_file_path]  # 使用绝对路径
#         )
#     ])

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'parameter_server_pkg'  # 你的包名
    config_file = os.path.join(get_package_share_directory(package_name), 'config', 'para_setting.yaml')

    return LaunchDescription([
        Node(
            package=package_name,
            executable='get_parameter_node',
            name='get_parameter_node',
            output='screen',
            parameters=[config_file]
        )
    ])


