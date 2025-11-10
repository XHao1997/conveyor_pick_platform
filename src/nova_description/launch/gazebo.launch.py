#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # -------- Args --------
    model_file = DeclareLaunchArgument(
        'model_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nova_description'), 'urdf', 'Nova2.urdf.xacro'  # 若是 .xacro 也可，xacro 会展开
        ]),
        description='Path to Nova2 URDF/Xacro'
    )
    controllers_file = DeclareLaunchArgument(
        'controllers_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nova_description'), 'config', 'ros2_controllers.yaml'
        ]),
        description='ros2_control YAML (used by controller spawners)'
    )
    world_file = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'worlds', 'empty.sdf'
        ]),
        description='SDF world file for gz sim'
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    model_path       = LaunchConfiguration('model_file')
    controllers_yaml = LaunchConfiguration('controllers_file')
    world            = LaunchConfiguration('world')
    use_sim_time     = LaunchConfiguration('use_sim_time')

    # -------- robot_description (xacro/urdf -> urdf) --------
    robot_description = Command([
        'xacro ', model_path,
        ' controllers_yaml:=', controllers_yaml,    # 仅当你的 xacro 接收该参数
        ' use_ros2_control:=true'                  # 仅当你的 xacro 接收该参数
    ])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # -------- gz sim --------
    # 官方 launch 接收 gz_args：例如 "-r -v 3 <world_path>"
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            # 这里直接拼成一串传进去
            'gz_args': [TextSubstitution(text='-r -v 3 ') , world]
        }.items()
    )

    # 可选：静态 TF（如需要 base_footprint）
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_footprint_base',
        arguments=['0','0','0','0','0','0','base_link','base_footprint']
    )

    # -------- Spawn entity from /robot_description --------
    # gz 实体名可按需修改
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_nova2',
        output='screen',
        arguments=[
            '-name', 'Nova2',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0',
            '-allow_renaming', 'true'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # -------- Controllers (可选，需 gz_ros2_control 支持) --------
    # 前提：你的 URDF/Xacro 里已经集成了 **gz_ros2_control**（不是 gazebo_ros2_control）
    # 典型顺序：joint_state_broadcaster -> 具体的 arm_controller（名字按你的 YAML 修改）
    js_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    arm_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--param-file', controllers_yaml],  # ← 改为你的控制器名
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 顺序：spawn -> JS -> arm_controller
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[js_broadcaster])
    )
    after_js = RegisterEventHandler(
        OnProcessExit(target_action=js_broadcaster, on_exit=[arm_controller])
    )

    return LaunchDescription([
        model_file, controllers_file, world_file, use_sim_time_arg,
        gz,
        static_tf,
        rsp,
        spawn,
        after_spawn,
        after_js,
    ])
