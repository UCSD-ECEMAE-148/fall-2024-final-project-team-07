from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to the vesc_twist.launch.py
    vesc_twist_launch_path = os.path.join(
        get_package_share_directory('ucsd_robocar_actuator2_pkg'),
        'launch',
        'vesc_twist.launch.py'
    )

    return LaunchDescription([
        # Launch the camera_control node
        Node(
            package='camera_control',
            executable='camera_executable',
            name='camera_control_node',
            output='screen',
        ),

        # Include the vesc_twist launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vesc_twist_launch_path),
            launch_arguments={}.items(),  # Add any arguments here if required
        ),
    ])
