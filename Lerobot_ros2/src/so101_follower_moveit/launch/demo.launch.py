"""Launch MoveIt with mock hardware or the SO-101 serial motor bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def launch_setup(context):
    if (LaunchConfiguration("start_servo").perform(context).lower() == "true"
            and LaunchConfiguration("start_move_group").perform(context).lower() == "true"):
        raise RuntimeError("Servo and MoveGroup execution cannot own arm_controller simultaneously; use servo.launch.py")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    moveit_config = (
        MoveItConfigsBuilder("so101_follower", package_name="so101_follower_moveit")
        .robot_description(
            file_path="config/so101_follower.urdf.xacro",
            mappings={"use_fake_hardware": use_fake_hardware},
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[LaunchConfiguration("controllers_config")],
            remappings=[("~/robot_description", "/robot_description")],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
            condition=IfCondition(LaunchConfiguration("start_move_group")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="log",
        ),
    ]
    real_hardware = use_fake_hardware.lower() not in ("true", "1")
    if real_hardware and LaunchConfiguration("start_motor_bridge").perform(context).lower() == "true":
        nodes.append(Node(
            package="so101_hw_interface",
            executable="so101_motor_bridge",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "calib_file": LaunchConfiguration("calib_file"),
                "move_to_midpoint": False,
            }],
            output="screen",
        ))
    broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
        output="screen",
    )
    trajectories = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "gripper_controller", "--controller-manager-timeout", "60"],
        output="screen",
    )
    if LaunchConfiguration("start_servo").perform(context).lower() == "true":
        with open(os.path.join(moveit_config.package_path, "config", "servo.yaml")) as stream:
            servo_params = yaml.safe_load(stream)
        servo = Node(
            package="moveit_servo", executable="servo_node", name="servo_node",
            parameters=[servo_params, moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics, moveit_config.joint_limits],
            output="screen",
        )
        tracker = Node(
            package="so101_servo", executable="point_tracker",
            parameters=[moveit_config.robot_description, {
                "tip_frame": LaunchConfiguration("tip_frame"),
                "target_timeout": float(LaunchConfiguration("target_timeout").perform(context)),
                **{name: float(LaunchConfiguration(name).perform(context)) for name in
                   ("max_linear_speed", "max_joint_speed", "max_joint_acceleration", "gain",
                    "tip_offset_x", "tip_offset_y", "tip_offset_z")},
            }], output="screen",
        )

        def after_controllers(event, context):
            if event.returncode == 0:
                return [servo, tracker]
            return [EmitEvent(event=Shutdown(reason="Trajectory controllers failed to start"))]

        nodes.append(RegisterEventHandler(OnProcessExit(target_action=trajectories, on_exit=after_controllers)))

    # The real trajectory controllers must initialize from measured positions.
    readiness = Node(
        package="so101_hw_interface",
        executable="so101_wait_for_joint_states",
        output="screen",
    ) if real_hardware else None

    def after_ready(event, context):
        if event.returncode == 0:
            return [trajectories]
        return [EmitEvent(event=Shutdown(reason="Motor feedback was not received"))]

    if real_hardware:
        nodes.append(RegisterEventHandler(OnProcessExit(target_action=readiness, on_exit=after_ready)))
    nodes.extend([
        RegisterEventHandler(OnProcessExit(
            target_action=broadcaster,
            on_exit=[readiness] if real_hardware else [trajectories],
        )),
        broadcaster,
    ])
    return nodes


def generate_launch_description():
    calibration = os.path.join(
        get_package_share_directory("so101_hw_interface"), "config", "so101_calibration.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true",
                              choices=["true", "false"], description="Use mock hardware instead of the real arm"),
        DeclareLaunchArgument("start_motor_bridge", default_value="true",
                              choices=["true", "false"], description="Start the serial bridge; false uses an external bridge"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Start RViz"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0", description="SO-101 serial port"),
        DeclareLaunchArgument("calib_file", default_value=calibration, description="Motor calibration YAML"),
        DeclareLaunchArgument("start_move_group", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_servo", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("controllers_config", default_value=os.path.join(
            get_package_share_directory("so101_follower_moveit"), "config", "ros2_controllers.yaml")),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(
            get_package_share_directory("so101_follower_moveit"), "config", "moveit.rviz")),
        DeclareLaunchArgument("tip_frame", default_value="gripper_link"),
        DeclareLaunchArgument("target_timeout", default_value="0.3"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.08"),
        DeclareLaunchArgument("max_joint_speed", default_value="0.8"),
        DeclareLaunchArgument("max_joint_acceleration", default_value="1.0"),
        DeclareLaunchArgument("gain", default_value="4.0"),
        DeclareLaunchArgument("tip_offset_x", default_value="0.0"),
        DeclareLaunchArgument("tip_offset_y", default_value="0.0"),
        DeclareLaunchArgument("tip_offset_z", default_value="0.0"),
        OpaqueFunction(function=launch_setup),
    ])
