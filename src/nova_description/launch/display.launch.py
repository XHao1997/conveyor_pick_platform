#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def _launch_setup(context, *args, **kwargs):
    # 解析参数
    model_path = LaunchConfiguration('model').perform(context)
    rviz_cfg  = LaunchConfiguration('rviz_config').perform(context)
    use_gui   = LaunchConfiguration('use_gui').perform(context).lower() in ('1','true','yes')

    # 读取 URDF 文本（ROS2 没有 <param textfile=...>，需要手动读进来）
    with open(model_path, 'r') as f:
        robot_description = f.read()

    nodes = []

    if use_gui:
        nodes.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui'
            )
        )
    else:
        # 不需要 GUI 时也可以使用无界面的 joint_state_publisher（可选）
        nodes.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher'
            )
        )

    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        )
    )

    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen'
        )
    )

    return nodes

def generate_launch_description():
    default_model = PathJoinSubstitution([
        FindPackageShare('nova_description'),
        'urdf', 'Nova2.urdf.xacro'   # 如果你改成 .xacro，把文件名换掉即可
    ])

    default_rviz = PathJoinSubstitution([
        FindPackageShare('nova_description'),
        'urdf.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Path to the Nova2 URDF/Xacro file'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Path to RViz2 config'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui if true, else joint_state_publisher'
        ),
        OpaqueFunction(function=_launch_setup)
    ])
