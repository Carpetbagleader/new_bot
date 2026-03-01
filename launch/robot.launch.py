from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to your URDF
    urdf_file = LaunchConfiguration('urdf_file', default='$(find new_bot)/description/robot.urdf.xacro')

    return LaunchDescription([
        # robot_state_publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}]
        ),

        # Teleop / cmd_vel bridge node
        Node(
            package='new_bot',
            executable='cmd_vel_bridge',
            name='cmd_vel_bridge',
            output='screen'
        ),

        # RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config', default='')]
        ),
    ])