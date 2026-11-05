import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('so101_aruco')
    robot_share = get_package_share_directory('so101_follower_description')
    description = xacro.process_file(os.path.join(robot_share, 'urdf', 'so101_follower.urdf.xacro')).toxml()
    return LaunchDescription([
        DeclareLaunchArgument('offsets_file', default_value=os.path.join(share, 'config', 'marker_offsets.yaml')),
        DeclareLaunchArgument('preview_file', default_value=os.path.join(share, 'config', 'preview.yaml')),
        DeclareLaunchArgument('start_detector', default_value='true'),
        DeclareLaunchArgument('use_fake_joints', default_value='true', description='Display-only joint values; sends no robot commands'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        Node(package='so101_aruco', executable='detector', output='screen',
             condition=IfCondition(Arg('start_detector')), parameters=[Arg('offsets_file'), {
                 'publish_camera_tf': True,
                 'base_prediction_frame': 'base_link_predict'}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}],
             remappings=[('/joint_states', Arg('joint_states_topic'))]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=IfCondition(Arg('use_fake_joints')), parameters=[{'robot_description': description}],
             remappings=[('/joint_states', Arg('joint_states_topic'))]),
        Node(package='so101_aruco', executable='preview', output='screen', parameters=[Arg('preview_file')]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'rviz', 'aruco_preview.rviz')]),
    ])
