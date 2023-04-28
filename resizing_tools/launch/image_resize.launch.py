from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription(
    [
      Node(
        package="resizing_tools",
        name="image_resizer",
        executable="image_resizer",
        output="screen",
        parameters=[{'width':640}, {'height':480}]
      )
    ]
  )
