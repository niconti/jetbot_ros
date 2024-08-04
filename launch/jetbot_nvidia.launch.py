#
# Launch NVIDIA JetBot motor controller and camera nodes.
# This is for the original NVIDIA JetBot.
#
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    joy_config_arg = DeclareLaunchArgument('joy_config', default_value='logitech')

    joy_config_file_arg = DeclareLaunchArgument('joy_config_file', default_value=[
        TextSubstitution(text=os.path.join(get_package_share_directory('jetbot_ros'), 'config', '')), LaunchConfiguration('joy_config'), TextSubstitution(text='.config.yaml')
    ])

    jetbot_config_file_arg = DeclareLaunchArgument('jetbot_config_file', default_value=[
        TextSubstitution(text=os.path.join(get_package_share_directory('jetbot_ros'), 'config', '')), 'jetbot', TextSubstitution(text='.config.yaml')
    ])

    rtp_output_arg = DeclareLaunchArgument('rtp_output', default_value="DUSTINF-LT1.fios-router.home:1234")
    

    motor_controller = Node(package='jetbot_ros', executable='motors_nvidia',
                            parameters=[LaunchConfiguration('jetbot_config_file')],
                            output='screen', emulate_tty=True)              

    # oled_controller = Node(package='jetbot_ros', executable='oled_ssd1306',
    #                         output='screen', emulate_tty=True)  
 
    monitor_battery = Node(package='jetbot_ros', executable='monitor_battery',
                        parameters=[
                            {"warning_level": 20},
                            {"critical_level": 10},
                        ],
                        output='screen', emulate_tty=True, arguments=[('__log_level:=debug')])

    teleop_camera = Node(package='jetbot_ros', executable='teleop_camera',
                         parameters=[
                            {"pan_scale": 2},
                            {"tilt_scale": 2},
                         ],
                         output='screen', emulate_tty=True, arguments=[('__log_level:=debug')])

    teleop_robot = Node(package='teleop_twist_joy', executable='teleop_node',
                        name='teleop_robot',
                        parameters=[
                            LaunchConfiguration('joy_config_file')
                        ],
                        remappings=[
                            ("/cmd_vel", "/jetbot/cmd_vel"),
                        ],
                        output='screen', emulate_tty=True)


    video_source = Node(package='ros_deep_learning', executable='video_source',
                        parameters=[
                            {"resource": "csi://0"},
                            {"width": 640},
                            {"height": 480},
                            {"framerate": 15.0}
                        ],
                        remappings=[
                            ("raw", "/jetbot/camera/image_raw"),
                        ],
                        output='screen', emulate_tty=True)

    detectenet = Node(package='ros_deep_learning', executable='detectnet',
                        parameters=[
                            {"model_name": "ssd-mobilenet-v2"},
                            {"overlay_flags": "box,labels,conf"},
                            {"mean_pixel_value": 0.00},
                            {"threshold": 0.5}
                        ],
                        remappings=[
                            ("image_in", "/jetbot/camera/image_raw"),
                        ],
                        output='screen', emulate_tty=True)

    image_transport = Node(package='image_transport', executable='republish',
                        arguments=[
                            ('raw'),
                            ('compressed')
                        ],
                        remappings=[
                            ("in", "/jetbot/camera/image_raw"),
                            ("out/compressed", "/jetbot/camera/image_raw/compressed")
                        ],
                        output='screen', emulate_tty=True)  

    video_output = Node(package='ros_deep_learning', executable='video_output',
                        parameters=[
                            {"resource": ["rtp://", LaunchConfiguration('rtp_output')]},
                            {"codec": "h264"},
                        ],
                        remappings=[
                            ("image_in", "/jetbot/camera/image_raw"),
                        ],
                        output='screen', emulate_tty=True)
    
    wireless_watcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('wireless_watcher'), 'launch', 'watcher.launch.py')
        ]),
        launch_arguments={'dev': 'wlan0'}.items(),
    )

    return LaunchDescription([
        joy_config_arg,
        joy_config_file_arg,
        jetbot_config_file_arg,
        rtp_output_arg,
        motor_controller,
        # oled_controller,
        monitor_battery,
        teleop_camera,
        teleop_robot,
        video_source,
        # detectenet,
        # video_output,
        image_transport,
        wireless_watcher
    ])