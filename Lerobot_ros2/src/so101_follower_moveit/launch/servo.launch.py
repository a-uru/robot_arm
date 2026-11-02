"""SO-101 position tracking; settings come from servo_tracking.yaml."""
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

TRACKING_KEYS = (
    "tip_frame", "tip_offset_x", "tip_offset_y", "tip_offset_z", "max_linear_speed",
    "max_joint_speed", "max_joint_acceleration", "gain", "target_timeout",
)


def launch_setup(context):
    share = get_package_share_directory("so101_follower_moveit")
    path = LaunchConfiguration("tracking_config").perform(context)
    with open(path, encoding="utf-8") as stream:
        settings = yaml.safe_load(stream)
    if not isinstance(settings, dict) or set(settings) != set(TRACKING_KEYS):
        raise ValueError(f"{path}: expected settings: {', '.join(TRACKING_KEYS)}")
    for key in TRACKING_KEYS:
        override = LaunchConfiguration(key).perform(context)
        if override:
            settings[key] = override
        if key == "tip_frame":
            if not isinstance(settings[key], str) or not settings[key].strip():
                raise ValueError("tip_frame must be a non-empty frame name")
        else:
            settings[key] = float(settings[key])
            if not math.isfinite(settings[key]):
                raise ValueError(f"{key} must be finite")
            if not key.startswith("tip_offset_") and settings[key] <= 0:
                raise ValueError(f"{key} must be positive")
    hardware_keys = ("use_fake_hardware", "use_rviz", "port", "start_motor_bridge", "calib_file")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "demo.launch.py")),
        launch_arguments={**{key: LaunchConfiguration(key) for key in hardware_keys},
                          **{key: str(value) for key, value in settings.items()},
                          "start_move_group": "false", "start_servo": "true",
                          "controllers_config": os.path.join(share, "config", "servo_controllers.yaml"),
                          "rviz_config": os.path.join(share, "config", "servo.rviz")}.items(),
    ), Node(
        package="so101_servo", executable="interactive_target",
        parameters=[{key: settings[key] for key in ("tip_frame", "tip_offset_x", "tip_offset_y", "tip_offset_z")}],
        condition=IfCondition(LaunchConfiguration("interactive_target")),
        output="screen",
    )]


def generate_launch_description():
    share = get_package_share_directory("so101_follower_moveit")
    args = {"use_fake_hardware": "true", "use_rviz": "true", "port": "/dev/ttyACM0",
            "start_motor_bridge": "true",
            "tracking_config": os.path.join(share, "config", "servo_tracking.yaml"),
            "calib_file": os.path.join(get_package_share_directory("so101_hw_interface"), "config", "so101_calibration.yaml")}
    declarations = [DeclareLaunchArgument("interactive_target", default_value="false", choices=["true", "false"],
                                          description="Drag a target marker in RViz to control the tool position")]
    declarations += [DeclareLaunchArgument(key, default_value=value) for key, value in args.items()]
    declarations += [DeclareLaunchArgument(key, default_value="", description="Optional override of servo_tracking.yaml")
                     for key in TRACKING_KEYS]
    return LaunchDescription(declarations + [OpaqueFunction(function=launch_setup)])
