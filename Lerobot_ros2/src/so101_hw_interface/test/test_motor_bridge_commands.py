"""Verify command initialization without opening a serial port."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState

from so101_hw_interface.motor_bridge import JOINTS, MotorBridge


def make_bridge(move_to_midpoint=False, calibrated=True):
    raw = {name: 2100 + index * 10 for index, name in enumerate(JOINTS)}
    bus = Mock()
    bus.sync_read.return_value = raw
    bridge = SimpleNamespace(
        bus=bus,
        _home_offsets={name: 2048 for name in JOINTS} if calibrated else None,
        _limits={name: (1000, 3000) for name in JOINTS} if calibrated else None,
        _commands_initialized=False,
        _initial_move_done=False,
        _steps_per_rad=4096.0 / (2 * math.pi),
        current_commands={name: 0.0 for name in JOINTS},
        joint_states_pub=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        get_logger=lambda: Mock(),
        get_parameter=lambda name: SimpleNamespace(value=move_to_midpoint),
    )
    return bridge, raw


@pytest.mark.parametrize("calibrated", [True, False])
def test_first_read_holds_measured_pose_and_partial_command_preserves_other_joints(calibrated):
    bridge, raw = make_bridge(calibrated=calibrated)
    # A controller command arriving before feedback must not move the robot.
    MotorBridge._command_cb(bridge, JointState(name=["gripper"], position=[0.5]))
    MotorBridge._timer_cb(bridge)
    written = bridge.bus.sync_write.call_args.args[1]
    assert all(abs(written[name] - raw[name]) <= 1 for name in JOINTS)

    MotorBridge._command_cb(bridge, JointState(name=["gripper"], position=[0.15]))
    MotorBridge._timer_cb(bridge)
    written = bridge.bus.sync_write.call_args.args[1]
    assert all(abs(written[name] - raw[name]) <= 1 for name in JOINTS if name != "gripper")
    home = bridge._home_offsets["gripper"]
    assert written["gripper"] == int(home + 0.15 * bridge._steps_per_rad)


def test_uninitialized_topic_commands_are_ignored():
    bridge, _ = make_bridge()
    MotorBridge._timer_cb(bridge)
    before = dict(bridge.current_commands)
    MotorBridge._command_cb(bridge, JointState(
        name=["shoulder_pan", "gripper", "unknown"],
        position=[float("nan"), float("inf"), 0.1],
    ))
    assert bridge.current_commands == before


def test_standalone_midpoint_option_remains_available():
    bridge, _ = make_bridge(move_to_midpoint=True)
    MotorBridge._timer_cb(bridge)
    written = bridge.bus.sync_write.call_args.args[1]
    assert bridge._initial_move_done
    assert all(abs(value - 2000) <= 1 for value in written.values())
