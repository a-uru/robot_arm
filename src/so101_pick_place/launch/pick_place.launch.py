"""One terminal: USB camera, ArUco, MoveIt/controllers, RViz and Enter worker."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def default_workspace():
    for parent in Path(__file__).resolve().parents:
        if (parent / 'scripts/start_usb_camera.sh').is_file():
            return str(parent)
    return os.path.expanduser('~/robot_arm')


def setup(context):
    workspace = Path(Arg('workspace').perform(context)).resolve()
    camera_script = workspace / 'scripts/start_usb_camera.sh'
    calibration = workspace / 'config/camera/ost.yaml'
    if Arg('start_camera').perform(context).lower() == 'true':
        if not camera_script.is_file() or not calibration.is_file():
            raise RuntimeError('USB camera script/calibration is missing under workspace')
    camera_profile = workspace / 'config/camera/fastdds_camera.xml'
    if not camera_profile.is_file():
        raise RuntimeError('Camera DDS profile missing under workspace')
    fake = Arg('use_fake_hardware').perform(context)
    moveit_share = get_package_share_directory('so101_follower_moveit')
    aruco_share = get_package_share_directory('so101_aruco')
    moveit = MoveItConfigsBuilder('so101_follower', package_name='so101_follower_moveit').robot_description(
        file_path='config/so101_follower.urdf.xacro', mappings={'use_fake_hardware': fake}).to_moveit_configs()
    return [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'DEFAULT'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', str(camera_profile)),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', str(camera_profile)),
        ExecuteProcess(cmd=['bash', str(camera_script), '-p', 'camera_info_url:=file://' + str(calibration)],
                       additional_env={'VIDEO_DEVICE': Arg('video_device')},
                       condition=IfCondition(Arg('start_camera')), output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(moveit_share, 'launch', 'demo.launch.py')),
            launch_arguments={'use_fake_hardware': fake, 'port': Arg('port'), 'use_rviz': Arg('use_rviz'),
                              'rviz_config': os.path.join(aruco_share, 'rviz', 'aruco_preview.rviz')}.items()),
        Node(package='so101_aruco', executable='detector', name='aruco_detector', output='screen',
             condition=IfCondition(Arg('start_perception')), parameters=[Arg('offsets_file'), {'publish_camera_tf': True}]),
        Node(package='so101_aruco', executable='preview', output='screen',
             condition=IfCondition(Arg('start_perception')), parameters=[os.path.join(aruco_share, 'config', 'preview.yaml')]),
        Node(package='so101_pick_place', executable='pick_place', name='so101_pick_place', output='screen',
             parameters=[Arg('pick_config'), moveit.robot_description,
                         {'use_fake_hardware': ParameterValue(Arg('use_fake_hardware'), value_type=bool)}]),
    ]


def generate_launch_description():
    share = get_package_share_directory('so101_pick_place')
    aruco_share = get_package_share_directory('so101_aruco')
    return LaunchDescription([
        DeclareLaunchArgument('workspace', default_value=default_workspace()),
        DeclareLaunchArgument('use_fake_hardware', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/v4l/by-id/usb-XWF-1080p6_Camera_XWF-1080p6_Camera_202403191545002-video-index0'),
        DeclareLaunchArgument('offsets_file', default_value=os.path.join(aruco_share, 'config', 'marker_offsets.yaml')),
        DeclareLaunchArgument('pick_config', default_value=os.path.join(share, 'config', 'pick_place.yaml')),
        OpaqueFunction(function=setup),
    ])
