from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_share = os.path.join(os.getenv('HOME'), 'ros2_ws', 'src', 'rplidar_ros', 'launch')

    return LaunchDescription([
        # CmdVel bridge + odometry
        Node(
            package='new_bot',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen'
        ),

        # Robot state publisher (URDF must be fully expanded)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': '<insert expanded URDF path here>'}]
        ),

        # Include LiDAR launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'rplidar_a1_launch.py'))
        )
    ])