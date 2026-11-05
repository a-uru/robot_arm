"""Read-only incremental register sampling, sharing the bridge's serial owner."""
import json
import time
from std_msgs.msg import String

SETTINGS = ('P_Coefficient', 'I_Coefficient', 'D_Coefficient', 'CW_Dead_Zone',
            'CCW_Dead_Zone', 'Minimum_Startup_Force', 'Max_Torque_Limit',
            'Torque_Limit', 'Protection_Current', 'Protective_Torque',
            'Overload_Torque', 'Operating_Mode', 'Torque_Enable')
LIVE = ('Present_Voltage', 'Present_Current', 'Present_Load',
        'Present_Temperature', 'Goal_Position', 'Present_Position', 'Status')


class MotorDiagnostics:
    def __init__(self, node, bus, joints):
        self.node, self.bus = node, bus
        self.pending = [(joint, reg) for joint in joints for reg in SETTINGS]
        self.index = 0
        self.publisher = node.create_publisher(String, '/so101_follower/motor_diagnostics', 10)
        node.declare_parameter('motor_diagnostics_enabled', False)
        # Single-threaded bridge executor serializes this timer with all motor I/O.
        self.timer = node.create_timer(.2, self.sample)

    def sample(self):
        if not self.node.get_parameter('motor_diagnostics_enabled').value:
            return
        record = {'stamp_ns': self.node.get_clock().now().nanoseconds}
        started = time.monotonic()
        try:
            if self.pending:
                joint, reg = self.pending.pop(0)
                record.update(kind='setting', register=reg,
                              values={joint: self.bus.read(reg, joint, normalize=False)})
            else:
                reg = LIVE[self.index % len(LIVE)]
                self.index += 1
                record.update(kind='live', register=reg,
                              values=self.bus.sync_read(reg, normalize=False))
        except Exception as exc:
            record.update(register=reg, error=str(exc))
        record['read_duration_s'] = time.monotonic() - started
        self.publisher.publish(String(data=json.dumps(record)))
