#!/usr/bin/env python3
"""Read-only recording of controller and motor-bridge tracking data."""
import argparse
import csv
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=180.)
    parser.add_argument('--output', default='log/arm_tracking/' + time.strftime('%Y%m%d_%H%M%S'))
    args = parser.parse_args()
    folder = Path(args.output)
    folder.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('arm_tracking_recorder')
    start = time.monotonic()
    samples = []
    streams = []
    subscriptions = []

    def csv_file(name, header):
        stream = (folder / name).open('w', newline='')
        streams.append(stream)
        writer = csv.writer(stream)
        writer.writerow(header)
        return writer

    controller = csv_file('controller.csv', ['elapsed_s', 'stamp_s', 'joint', 'reference_rad', 'feedback_rad', 'error_rad', 'reference_velocity_rad_s', 'feedback_velocity_rad_s'])
    bridge = csv_file('bridge.csv', ['elapsed_s', 'source', 'stamp_s', 'joint', 'position_rad'])
    events = csv_file('events.csv', ['elapsed_s', 'status'])
    diagnostics = (folder / 'motor_diagnostics.jsonl').open('w')
    streams.append(diagnostics)

    def on_diagnostics(msg):
        record = json.loads(msg.data)
        record['elapsed_s'] = time.monotonic() - start
        diagnostics.write(json.dumps(record) + '\n')

    def at(values, i):
        return values[i] if i < len(values) else ''

    def on_controller(msg):
        elapsed = time.monotonic() - start
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for i, joint in enumerate(msg.joint_names):
            row = [elapsed, stamp, joint, at(msg.reference.positions, i), at(msg.feedback.positions, i), at(msg.error.positions, i), at(msg.reference.velocities, i), at(msg.feedback.velocities, i)]
            controller.writerow(row)
            samples.append(row)

    def on_bridge(msg, source):
        elapsed = time.monotonic() - start
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for joint, position in zip(msg.name, msg.position):
            bridge.writerow([elapsed, source, stamp, joint, position])

    subscriptions.append(node.create_subscription(JointTrajectoryControllerState, '/arm_controller/controller_state', on_controller, qos_profile_sensor_data))
    for topic in ('/so101_follower/joint_commands', '/so101_follower/joint_states', '/joint_states'):
        subscriptions.append(node.create_subscription(JointState, topic, lambda msg, source=topic: on_bridge(msg, source), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(String, '/pick_place/status', lambda msg: events.writerow([time.monotonic() - start, msg.data]), 10))
    subscriptions.append(node.create_subscription(String, '/so101_follower/motor_diagnostics', on_diagnostics, 10))
    print(f'Recording for {args.seconds:g}s into {folder}. No commands are published.', flush=True)
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.2)
            for stream in streams:
                stream.flush()
    except KeyboardInterrupt:
        pass
    finally:
        summary = {}
        for joint in sorted({row[2] for row in samples}):
            rows = [row for row in samples if row[2] == joint]
            valid = [row for row in rows if isinstance(row[5], (float, int))]
            if not valid:
                continue
            tail = [row for row in valid if row[0] >= valid[-1][0] - 2.]
            summary[joint] = {'samples': len(rows), 'max_abs_error_rad': max(abs(row[5]) for row in valid),
                              'last_error_rad': valid[-1][5],
                              'last_2s_reference_range_rad': max(row[3] for row in tail) - min(row[3] for row in tail),
                              'last_2s_feedback_change_rad': tail[-1][4] - tail[0][4]}
        (folder / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        for stream in streams:
            stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'Saved: {folder}; controller joints recorded: {len(summary)}', flush=True)


if __name__ == '__main__':
    main()
