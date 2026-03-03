from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    xacro_file = PathJoinSubstitution([
        FindPackageShare('new_bot'),
        'description',
        'robot.urdf.xacro'
    ])

    robot_description = {
        'robot_description': Command(['xacro', xacro_file])
    }

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])