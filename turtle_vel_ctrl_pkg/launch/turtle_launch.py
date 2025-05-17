from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # turtle1
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle1'
        ),
        Node(
            package='turtle_vel_ctrl_pkg',
            executable='turtle_vel_ctrl_node',
            name='turtle_vel_ctrl_node',
        ),

        # turtle2
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle2',
            remappings=[
                ('/turtle1/cmd_vel', '/turtle2/cmd_vel')  # 将控制命令重映射到 turtle2
            ],
            # parameters=[{'background_r': 255, 'background_g': 255, 'background_b': 255}]  # 可选：设置背景颜色
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='turtle_teleop_key',
            remappings=[
                ('/turtle1/cmd_vel', '/turtle2/cmd_vel')  # 将控制命令重映射到 turtle2
            ],
            prefix='gnome-terminal --'  # 在新终端中运行键盘控制节点
        ),
        
    ])