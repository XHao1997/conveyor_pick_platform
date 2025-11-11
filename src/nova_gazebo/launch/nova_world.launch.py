import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world', default_value='test2_world.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_nova_gazebo = get_package_share_directory('nova_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_nova_gazebo,
            'worlds',
            LaunchConfiguration('world')
        ]),
        #TextSubstitution(text=' -r -v -v1')],
        TextSubstitution(text=' -r -v -v1 --render-engine ogre --render-engine-gui-api-backend opengl')],
        'on_exit_shutdown': 'true'}.items()
    )

    gz_bridge_params_path = os.path.join(
        get_package_share_directory('nova_description'),
        'config',
        'gz_bridge.yaml'
    )
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )
    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(gz_bridge_node)

    launchDescriptionObject.add_action(gazebo_launch)

    return launchDescriptionObject