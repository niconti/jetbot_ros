#
# Launch NVIDIA JetBot motor controller and camera nodes.
# This is for the original NVIDIA JetBot.
#

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'jetbot_ros'


def generate_launch_description():

    jetbot_config_file_arg = DeclareLaunchArgument(
        name='jetbot_config_file', 
        default_value=[
            PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'config', 'jetbot.config.yaml'])
        ])

    joy_config_arg = DeclareLaunchArgument(
        name='joy_config', 
        default_value='logitech')

    joy_config_file_arg = DeclareLaunchArgument(
        name='joy_config_file', 
        default_value=[
            PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'config', '']), LaunchConfiguration('joy_config'), '.config.yaml'
        ])

    monitor_battery = Node(
        package='jetbot_ros', 
        executable='monitor_battery',
        parameters=[
            {"warning_level": 20},
            {"critical_level": 10},
        ],
        ros_arguments=[
            '--log-level', 'info'
        ],
        emulate_tty=True)

    motor_controller = Node(
        package='jetbot_ros',
        executable='motors_nvidia',
        parameters=[LaunchConfiguration('jetbot_config_file')],
        ros_arguments=[
            '--log-level', 'debug',
            '--log-level', 'rcl:=info'
        ],
        emulate_tty=True)              

    teleop_robot = Node(
        name='teleop_robot',
        package='teleop_twist_joy', 
        executable='teleop_node',
        parameters=[
            LaunchConfiguration('joy_config_file')
        ],
        remappings=[
            ("cmd_vel", "jetbot/cmd_vel"),
        ],
        ros_arguments=[
            '--log-level', 'debug',
            '--log-level', 'rcl:=info'
        ],
        emulate_tty=True)

    teleop_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('jetbot_ros'), 'launch', 'teleop_camera.launch.py'])
        ]))

    wireless_watcher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('wireless_watcher'), 'launch', 'watcher.launch.py'])
        ]),
        launch_arguments={
            'dev': 'wlan0'
        }.items())

    return LaunchDescription([
        joy_config_arg,
        joy_config_file_arg,
        jetbot_config_file_arg,
        motor_controller,
        monitor_battery,
        teleop_robot,
        teleop_camera_launch,
        wireless_watcher_launch
    ])