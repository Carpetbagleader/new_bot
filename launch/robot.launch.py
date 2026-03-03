from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():

    # ----- Robot Description via XACRO (same as your old working version) -----
    xacro_file = PathJoinSubstitution([
        FindPackageShare('new_bot'),
        'description',
        'robot.urdf.xacro'
    ])

    robot_description = {
        'robot_description': Command(['xacro', xacro_file])
    }

    # ----- Include RPLidar launch -----
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            ])
        )
    )

    return LaunchDescription([

        # Robot state publisher (needed for TF tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # Your motor + odometry bridge
        Node(
            package='new_bot',
            executable='cmd_vel_bridge',
            output='screen'
        ),

        # LiDAR
        rplidar_launch,
    ])