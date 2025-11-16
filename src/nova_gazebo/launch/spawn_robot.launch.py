#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


# ------------------------- Package paths -------------------------
pkg_nova_description = get_package_share_directory('nova_description')
pkg_nova_gazebo = get_package_share_directory('nova_gazebo')
custom_models = os.path.join(pkg_nova_gazebo, 'custom_models')


# ------------------------- Gazebo resource paths -------------------------
def _append_env(var: str, path: str):
    cur = os.environ.get(var, '')
    parts = [p for p in cur.split(os.pathsep) if p]
    if path not in parts:
        parts.append(path)
    os.environ[var] = os.pathsep.join(parts)

for var in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH", "GAZEBO_MODEL_PATH"):
    _append_env(var, pkg_nova_description)
    _append_env(var, custom_models)


# ------------------------- Robots & poses -------------------------
robot_names = ["nova2_robot0", "nova2_robot1", "nova2_robot2", "nova2_robot3"]

robot_poses = {
    "nova2_robot0": (-0.14,  1.65, 0.351,  3.14/2.0),
    "nova2_robot1": (-0.14,  0.65, 0.351,  3.14/2.0),
    "nova2_robot2": ( 0.14, -1.65, 0.351, -3.14/2.0),
    "nova2_robot3": ( 0.14, -0.65, 0.351, -3.14/2.0),
}


# ------------------------- Helpers -------------------------
def make_robot_description_cmd(ns: str):
    return Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("nova_description"),
            "urdf",
            "Nova2.urdf.xacro",
        ]),
        " ",
        "use_gazebo:=true ",
        f"ns:={ns}",
    ])


def rsp_node(ns: str, robot_description_cmd):
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_cmd,
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )


def spawn_entity_node(ns: str):
    x, y, z, yaw = robot_poses[ns]
    return Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", ns,
            "-topic", f"/{ns}/robot_description",
            "-x", str(x),
            "-y", str(y),
            "-z", str(z),
            "-Y", str(yaw),
        ],
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )


def jsb_spawner(ns: str, timeout_sec: float = 60.0):
    return Node(
        package='controller_manager',
        executable='spawner',
        namespace=ns,
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', f'{timeout_sec}',
            '--controller-manager', f'/{ns}/controller_manager',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )


def arm_spawner(ns: str, controllers_yaml, timeout_sec: float = 10.0):
    return Node(
        package='controller_manager',
        executable='spawner',
        namespace=ns,
        arguments=[
            'arm_controller',
            '--param-file', controllers_yaml,
            '--controller-manager-timeout', f'{timeout_sec}',
            '--controller-manager', f'/{ns}/controller_manager',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )


def staged_robot_actions(ns: str,
                         controllers_yaml,
                         offset_sec: float,
                         wait_before_spawn: float = 2.0,
                         wait_before_controllers: float = 4.0,
                         cm_timeout: float = 10.0):
    desc_cmd = make_robot_description_cmd(ns)
    return GroupAction([
        TimerAction(period=offset_sec + 0.0, actions=[rsp_node(ns, desc_cmd)]),
        TimerAction(period=offset_sec + wait_before_spawn, actions=[spawn_entity_node(ns)]),
        TimerAction(period=offset_sec + wait_before_controllers, actions=[jsb_spawner(ns, cm_timeout)]),
        TimerAction(period=offset_sec + wait_before_controllers + 0.2,
                    actions=[arm_spawner(ns, controllers_yaml, cm_timeout)]),
    ])


# ------------------------- Main -------------------------
def generate_launch_description() -> LaunchDescription:
    world_arg = DeclareLaunchArgument(
        'world', default_value='test2_world.sdf', description='Gazebo world file'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='use_sim_time flag'
    )
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='false', description='Open RViz'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='rviz.rviz', description='RViz config file'
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nova_gazebo, 'launch', 'nova_world.launch.py'),
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_nova_description, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("nova_description"),
        "config",
        "nova2_controllers.yaml",
    ])

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(rviz_launch_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(world_launch)
    ld.add_action(gazebo_bridge)
    ld.add_action(rviz_node)

    # Stagger each robot by 3 seconds
    stagger = 3.0
    for i, ns in enumerate(robot_names):
        ld.add_action(staged_robot_actions(
            ns=ns,
            controllers_yaml=controllers_yaml,
            offset_sec=i * stagger,
            wait_before_spawn=2.0,
            wait_before_controllers=4.0,
            cm_timeout=60.0,
        ))

    return ld
