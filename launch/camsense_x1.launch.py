from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        Node(
            package='camsense_x1',
            executable='camsense_x1_node',
            name='camsense_x1',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
        ),
    ])
