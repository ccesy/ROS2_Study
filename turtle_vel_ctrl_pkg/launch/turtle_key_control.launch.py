from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='turtle_teleop_key',
            prefix='gnome-terminal --'  # 在新终端中运行键盘控制节点,
        ),
        # Node(
        #     package='turtle_vel_crtl_pkg',
        #     executable='turtle_vel_ctrl_node',
        #     name='turtle_teleop_key'
        # )
    ])
