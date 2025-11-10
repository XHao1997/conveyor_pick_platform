import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world', default_value='test_world.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_nova_gazebo = get_package_share_directory('nova_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nova_description = get_package_share_directory('nova_description')
    # Add your own gazebo library path here
    gazebo_models_path = os.getcwd() + "/src/nova_gazebo/custom_models"
    # os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    # os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + pkg_nova_description
    SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[LaunchConfiguration("existing_path"), ":", gazebo_models_path, ":", pkg_nova_description],
    )


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

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(gazebo_launch)

    return launchDescriptionObject