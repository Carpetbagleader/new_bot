from setuptools import setup

package_name = 'new_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=['new_bot'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/new_bot']),
        ('share/new_bot', ['package.xml']),

        # Install URDF / Xacro files
        ('share/new_bot/description', [
            'description/robot.urdf.xacro',
            'description/robot_core.xacro',
            'description/gazebo_control.xacro',
            'description/inertial_macros.xacro',
        ]),

        # Install mesh files into proper subfolder
        ('share/new_bot/description/meshes', [
            'description/meshes/base_link.STL',
            'description/meshes/camera_link.STL',
            'description/meshes/camera_collision.STL',
            'description/meshes/caster_wheel.STL',
            'description/meshes/laser_frame.STL',
            'description/meshes/lidar_collision.STL',
            'description/meshes/left_wheel.STL',
            'description/meshes/right_wheel.STL',
        ]),

        # Launch files
        ('share/new_bot/launch', [
            'launch/robot.launch.py',
            'launch/dev.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jagroop',
    maintainer_email='jagroop18randhawa@gmail.com',
    description='ROS2 package for new_bot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = new_bot.cmd_vel_bridge_node:main',
        ],
    },
)