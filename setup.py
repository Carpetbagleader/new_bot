from setuptools import find_packages, setup

package_name = 'new_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Standard ROS2 package registration
        ('share/ament_index/resource_index/packages',
            ['resource/new_bot']),
        ('share/new_bot', ['package.xml']),
        
        # Launch files (if you make any later)
        ('share/new_bot/launch', [
            # add your launch files here if you create them, e.g.
            # 'launch/launch_sim.launch.py'
        ]),
        
        # URDF / Xacro files
        ('share/new_bot/description', [
            'description/robot.urdf.xacro',
            'description/robot_core.xacro',
            'description/gazebo_control.xacro',
            'description/inertial_macros.xacro'
        ]),
        
        # Mesh files
        ('share/new_bot/description/meshes', [
            'description/meshes/base_link.STL',
            'description/meshes/camera_link.STL',
            'description/meshes/camera_collision.STL',
            'description/meshes/caster_wheel.STL',
            'description/meshes/laser_frame.STL',
            'description/meshes/lidar_collision.STL',
            'description/meshes/left_wheel.STL',
            'description/meshes/right_wheel.STL'
        ]),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jagroop',
    maintainer_email='jagroop18randhawa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add ROS2 nodes here if you have any
        ],
    },
)
