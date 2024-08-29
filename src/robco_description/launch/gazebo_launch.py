from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to your package
    package_name = 'robco_description'
    world_file_name = 'my_world.sdf'

    # Find the package path
    package_path = os.popen(f'ros2 pkg prefix {package_name}').read().strip()

    # Full path to the world file
    world_path = os.path.join(package_path, 'share', package_name, 'worlds', world_file_name)

    # Full path to the URDF file
    urdf_file = os.path.join(package_path, 'share', package_name, 'urdf', 'demo_robot.xacro')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                os.popen('ros2 pkg prefix gazebo_ros').read().strip(),
                'share', 'gazebo_ros', 'launch', 'gazebo.launch.py'
            )]),
            launch_arguments={'world': world_path}.items()
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'demo_robot', '-file', urdf_file],
            output='screen'),
    ])

