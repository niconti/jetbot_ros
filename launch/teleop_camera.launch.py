from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # rtp_output_arg = DeclareLaunchArgument(
    #     'rtp_output', 
    #     default_value="DUSTINF-LT1.fios-router.home:1234")

    teleop_camera = Node(
                    package='jetbot_ros', 
                    executable='teleop_camera',
                    parameters=[
                        { "pan_scale": 2.0 },
                        { "tilt_scale": 2.0 },
                    ],
                    ros_arguments=[
                        '--log-level', 'debug'
                    ],
                    emulate_tty=True)

    # v4l2_camera = Node(
    #                 package='v4l2_camera', 
    #                 namespace='jetbot/camera',
    #                 executable='v4l2_camera_node',
    #                 parameters=[
    #                     { "video_device": "/dev/video0" },
    #                     # { "image_size": [640, 480] },
    #                     { "image_size": [1920, 1080] },
    #                     { "output_encoding": "bgr8" }
    #                 ],
    #                 emulate_tty=True)

    # video_source = Node(package='ros_deep_learning', executable='video_source',
    #                 parameters=[
    #                     {"resource": "csi://0"},
    #                     {"width": 640},
    #                     {"height": 480},
    #                     {"framerate": 15.0}
    #                 ],
    #                 remappings=[
    #                     ("raw", "/jetbot/camera/image_raw"),
    #                 ],
    #                 emulate_tty=True)

    # detectenet = Node(package='ros_deep_learning', executable='detectnet',
    #                 parameters=[
    #                     {"model_name": "ssd-mobilenet-v2"},
    #                     {"overlay_flags": "box,labels,conf"},
    #                     {"mean_pixel_value": 0.00},
    #                     {"threshold": 0.5}
    #                 ],
    #                 remappings=[
    #                     ("image_in", "/jetbot/camera/image_raw"),
    #                 ],
    #                 emulate_tty=True)

    # video_output = Node(package='ros_deep_learning', executable='video_output',
    #                 parameters=[
    #                     {"resource": ["rtp://", LaunchConfiguration('rtp_output')]},
    #                     {"codec": "h264"},
    #                 ],
    #                 remappings=[
    #                     ("image_in", "/jetbot/camera/image_raw"),
    #                 ],
    #                 emulate_tty=True)

    image_container = ComposableNodeContainer(
                    namespace='jetbot/camera',
                    name='image_proc_container',
                    package='rclcpp_components',
                    executable='component_container',
                    composable_node_descriptions=[
                        ComposableNode(
                            namespace='jetbot/camera',
                            name='resize',
                            package='image_proc',
                            plugin='image_proc::ResizeNode',
                            parameters=[
                                { 'use_scale': False },
                                { 'width': 640 },
                                { 'height': 480 },
                                { 'image_transport': 'compressed' }
                            ],
                            remappings=[
                                ("image/image_raw", "/left/image_raw"),
                                ("image/camera_info", "/left/camera_info")
                            ])
                    ],
                    emulate_tty=True)

    image_transport = Node(
                    namespace='jetbot/camera',
                    package='image_transport', 
                    executable='republish',
                    arguments=[
                        ('raw'),
                        ('compressed')
                    ],
                    remappings=[
                        ("in", "resized/image_raw"),
                        ("out/compressed", "image_raw/compressed")
                    ],
                    emulate_tty=True)

    return LaunchDescription([
        # rtp_output_arg,
        teleop_camera,
        # v4l2_camera
        # video_source,
        # detectenet,
        # video_output,
        image_container,
        # image_transport
    ])