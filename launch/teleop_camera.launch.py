from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'jetbot_ros'


def generate_launch_description():

    module_id_arg = DeclareLaunchArgument(
        name='module_id', 
        default_value='-1')

    camera_id_arg = DeclareLaunchArgument(
        name='camera_id', 
        default_value='0')

    camera_info_url_arg = DeclareLaunchArgument(
        name='camera_info_url', 
        default_value=[
            TextSubstitution(text='file://'), 
            PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'config', 'camera_info.yaml'])
        ])

    camera_container = ComposableNodeContainer(
        namespace='jetbot/camera',
        name='camera_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                namespace='jetbot/camera',
                name='argus_mono',
                package='isaac_ros_argus_camera',
                plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
                parameters=[
                    { 'module_id': LaunchConfiguration('module_id') },
                    { 'camera_id': LaunchConfiguration('camera_id') },
                    { 'camera_info_url': LaunchConfiguration('camera_info_url') }
                ],
                remappings=[
                    ("left/image_raw", "image_raw"),
                    ("left/camera_info", "camera_info")
                ]
            ),
            ComposableNode(
                namespace='jetbot/camera',
                name='resize',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                parameters=[
                    { 'input_width': 3280 },
                    { 'input_height': 2464 },
                    { 'output_width': 640 },
                    { 'output_height': 480 }
                ],
                remappings=[
                    ("image", "image_raw"),
                    ("camera_info", "camera_info"),
                    ("resize/image", "resize/image_raw"),
                    ("resize/camera_info", "resize/camera_info")
                ]
            ),
            ComposableNode(
                namespace='jetbot/camera',
                name='flip',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFlipNode',
                parameters=[
                    { 'flip_mode': 'BOTH' }
                ],
                remappings=[
                    ("image", "resize/image_raw"),
                    ("image_flipped", "flip/image_raw"),
                ]
            )
        ],
        ros_arguments=[
            '--log-level', 'info'
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
            ("in", "flip/image_raw"),
            ("out/compressed", "image_raw/compressed")
        ],
        emulate_tty=True)
    
    teleop_camera = Node(
        package='jetbot_ros', 
        executable='teleop_camera',
        parameters=[
            { "pan_scale": 1.0 },
            { "tilt_scale": 1.0 }
        ],
        ros_arguments=[
            '--log-level', 'info'
        ],
        emulate_tty=True)

    return LaunchDescription([
        module_id_arg,
        camera_id_arg,
        camera_info_url_arg,
        camera_container,
        image_transport,
        teleop_camera
    ])