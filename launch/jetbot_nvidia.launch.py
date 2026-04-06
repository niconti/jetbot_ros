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
    
    teleop_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('jetbot_ros'), 'launch', 'teleop_robot.launch.py'])
        ]))

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
        monitor_battery,
        teleop_robot_launch,
        teleop_camera_launch,
        wireless_watcher_launch
    ])