from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='id_detection',
            executable='person_detection_executable',
            name='person_detection_node',
            output='screen',
        ),
    ])
