#
# Launch NVIDIA JetBot motor controller and camera nodes.
# This is for the original NVIDIA JetBot.
#
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    motor_controller = Node(package='jetbot_ros', executable='motors_nvidia',
                            remappings=[
                                ("/jetbot/cmd_vel", "/cmd_vel"),
                            ],
                            output='screen', emulate_tty=True)              
     
    # oled_controller = Node(package='jetbot_ros', executable='oled_ssd1306',
    #                         output='screen', emulate_tty=True)  
 
    teleop_camera_imu = Node(package='jetbot_ros', executable='teleop_camera_imu',
                             output='screen', emulate_tty=True, arguments=[('__log_level:=debug')])

    # teleop_camera_joy = Node(package='jetbot_ros', executable='teleop_camera_joy',
    #                         output='screen', emulate_tty=True, arguments=[('__log_level:=debug')])

    # teleop_robot_joy = Node(package='teleop_twist_joy', executable='teleop_node',
    #                     parameters=[
    #                         {"require_enable_button": "true"},
    #                     ],
    #                     remappings=[
    #                             ("/cmd_vel", "/jetbot/cmd_vel"),
    #                     ],
    #                     output='screen', emulate_tty=True)

    video_source = Node(package='ros_deep_learning', executable='video_source',
                        parameters=[
                            {"resource": "csi://0"},
                            {"width": 320},
                            {"height": 240},
                            {"framerate": 15.0},
                            {"flip": "rotate-180"},
                        ],
                        remappings=[
                            ("/video_source/raw", "/jetbot/camera/image_raw"),
                        ],
                        output='screen', emulate_tty=True)
    
    rtp_output = DeclareLaunchArgument('rtp_output', default_value="DUSTINF-LT1.fios-router.home:1234")
    
    video_output = Node(package='ros_deep_learning', executable='video_output',
                        parameters=[
                            {"resource": ["rtp://", LaunchConfiguration('rtp_output')]},
                            {"codec": "h264"},
                        ],
                        remappings=[
                            ("/video_output/image_in", "/jetbot/camera/image_raw"),
                        ],
                        output='screen', emulate_tty=True)
                        
    return LaunchDescription([
        motor_controller,
        # oled_controller,
        teleop_camera_imu,
        # teleop_camera_joy,
        rtp_output,
        video_source,
        video_output
    ])