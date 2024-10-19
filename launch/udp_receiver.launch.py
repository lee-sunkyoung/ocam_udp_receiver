from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='udp_receiver',
            executable='udp_receiver',
            name='udp_receiver_node',
            output='screen',
        ),
    ])
