from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('offsets_file', default_value=os.path.join(
            get_package_share_directory('so101_aruco'), 'config', 'marker_offsets.yaml')),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('marker_size', default_value='0.02'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_info'),
        Node(package='v4l2_camera', executable='v4l2_camera_node',
             condition=IfCondition(arg('start_camera')), parameters=[{
                 'video_device': arg('video_device'),
                 'camera_frame_id': 'camera_optical_frame',
                 'camera_info_url': arg('camera_info_url'),
             }]),
        Node(package='so101_aruco', executable='detector', output='screen', parameters=[arg('offsets_file'), {
            'marker_size': ParameterValue(arg('marker_size'), value_type=float),
            'marker_ids': [0, 1],
            'marker_sizes': ['[', arg('marker_size'), ', ', arg('marker_size'), ']'],
            'image_topic': arg('image_topic'), 'camera_info_topic': arg('camera_info_topic'),
        }]),
    ])
