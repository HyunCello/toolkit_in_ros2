from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='custom_zed_projection',
            executable='custom_zed_projection_node',
            output='screen',
            name='custom_zed_projection_node'
        )
    ])
