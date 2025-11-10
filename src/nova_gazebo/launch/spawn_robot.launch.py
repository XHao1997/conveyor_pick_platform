import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
pkg_nova_description = get_package_share_directory('nova_description')
pkg_nova_gazebo = get_package_share_directory('nova_gazebo')
custom_models = os.path.join(pkg_nova_gazebo, 'custom_models')



robot_positions = {"robot_0": (-0.14, 1.65, 0.351),
                  "robot_1": (-0.14, 0.65, 0.351),
                  "robot_2": (0.14, -1.65, 0.351),
                  "robot_3": (0.14, -0.65, 0.351),}
def _append_env(var: str, path: str):
    cur = os.environ.get(var, '')
    parts = [p for p in cur.split(os.pathsep) if p]
    if path not in parts:
        parts.append(path)
    os.environ[var] = os.pathsep.join(parts)

# 兼容 gz-sim / ignition / classic 三种变量名
for var in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH", "GAZEBO_MODEL_PATH"):
    _append_env(var, pkg_nova_description)  # 让 model://nova_description/... 可解析（目录在 share/nova_description）
    _append_env(var, custom_models)         # 你的自定义模型目录（有 model.config 的那批）

print("GZ_SIM_RESOURCE_PATH=", os.environ.get("GZ_SIM_RESOURCE_PATH", ""))
print("IGN_GAZEBO_RESOURCE_PATH=", os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""))
print("GAZEBO_MODEL_PATH=", os.environ.get("GAZEBO_MODEL_PATH", ""))



def spawn_robot(ld, robot_name: str, x: float, y: float, z: float, yaw: float):
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robot_name,
            "-topic", "robot_description",
            "-x", str(x),
            "-y", str(y),
            "-z", str(z),
            "-Y", str(yaw),
        ],
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    ld.add_action(spawn_node)



def generate_launch_description():
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("nova_description"),
            "urdf",
            "Nova2.urdf.xacro"
        ]),
        " ",
        "use_gazebo:=true ",
        # "robot_name:=nova2_robot0",
    ])

    robot_description = {"robot_description": robot_description_content}
    # ---------- Launch 参数 ----------
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='rviz.rviz', description='RViz config file'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value='test_world.sdf', description='Gazebo world file'
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='Nova2.urdf', description='URDF or Xacro file name'
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='spawn x')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='spawn y')
    z_arg = DeclareLaunchArgument('z', default_value='1.04', description='spawn z')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='spawn yaw')
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='use_sim_time flag'
    )

    # ---------- URDF/Xacro 路径 ----------
    urdf_file_path = PathJoinSubstitution([
        pkg_nova_description, "urdf", LaunchConfiguration('model')
    ])

    # ---------- 启世界 ----------
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nova_gazebo, 'launch', 'nova_world.launch.py'),
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # ---------- RViz（可选） ----------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_nova_description, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    robot_controllers = PathJoinSubstitution(
    [
        FindPackageShare("nova_description"),
        "config",
        "nova2_controllers.yaml",
    ]
    )
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--param-file',
            robot_controllers,
            ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # ---------- 机器人描述（RSP） ----------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )



    # ---------- 控制器（如果你有 ros2_control 配置） ----------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    ld = LaunchDescription()
    for a in (gazebo_bridge,rviz_launch_arg, rviz_config_arg, world_arg, model_arg, x_arg, y_arg, z_arg, yaw_arg, sim_time_arg):
        ld.add_action(a)
    ld.add_action(world_launch)
    ld.add_action(rviz_node)               # 如果不想开 RViz，可以注释掉
    ld.add_action(robot_state_publisher_node)

    x0, y0, z0 = robot_positions["robot_0"]
    x1, y1, z1 = robot_positions["robot_1"]
    x2, y2, z2 = robot_positions["robot_2"]
    x3, y3, z3 = robot_positions["robot_3"]
    spawn_robot(ld, "nova2_robot0", x0, y0, z0, 3.14/2)
    # spawn_robot(ld, "nova2_robot1", x1, y1, z1, 3.14/2)
    # spawn_robot(ld, "nova2_robot2", x2, y2, z2, -3.14/2)
    # spawn_robot(ld, "nova2_robot3", x3, y3, z3, -3.14/2)

    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_trajectory_controller_spawner)

    return ld
