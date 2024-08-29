from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robco_description_share = FindPackageShare(package='robco_description').find('robco_description')
    urdf_file = PathJoinSubstitution([robco_description_share, 'urdf', 'demo_robot.xacro'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments={'extra_gazebo_args': '--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'}.items()
    )

    set_gazebo_model_database_uri = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')
    set_gazebo_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', '/home/yo/robco_ws/src/robco_description:/usr/share/gazebo-11/models')


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robco',
        namespace='/',        
        arguments=['-entity', 'robco', '-topic', 'robot_description'],
        output='screen',
    )

    return LaunchDescription([
        set_gazebo_model_database_uri,
        set_gazebo_model_path,
        gazebo,
        robot_state_pub_node,
        TimerAction(
            period=5.0,  # Delay to ensure Gazebo is fully started
            actions=[spawn_robot]
        )
    ])

