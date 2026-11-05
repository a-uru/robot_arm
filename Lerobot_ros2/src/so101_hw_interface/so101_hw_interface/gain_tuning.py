"""Small, reversible P-gain changes through the existing serial owner."""
import json
import time
from pathlib import Path
from rcl_interfaces.msg import SetParametersResult


class GainTuning:
    def __init__(self, node, bus):
        self.node, self.bus = node, bus
        self.original = {}
        self.names = {f'tuning.{joint}.p': joint for joint in ('shoulder_lift', 'elbow_flex')}
        node.declare_parameter('gain_tuning_log', '/tmp/so101_gain_changes.jsonl')
        for name in self.names:
            node.declare_parameter(name, -1)
        node.add_on_set_parameters_callback(self.change)

    def change(self, parameters):
        selected = [p for p in parameters if p.name in self.names]
        if not selected:
            return SetParametersResult(successful=True)
        if len(parameters) != 1:
            return SetParametersResult(successful=False, reason='Change one joint gain at a time')
        p = selected[0]
        if type(p.value) is not int or (p.value != -1 and not 1 <= p.value <= 32):
            return SetParametersResult(successful=False, reason='P must be integer 1..32; -1 restores baseline')
        joint = self.names[p.name]
        old = None
        try:
            old = self.bus.read('P_Coefficient', joint, normalize=False)
            if joint not in self.original:
                self.original[joint] = old
            desired = self.original[joint] if p.value == -1 else p.value
            if p.value != -1 and abs(desired - old) > 4:
                return SetParametersResult(successful=False, reason='Limit each P adjustment to 4')
            log = Path(self.node.get_parameter('gain_tuning_log').value)
            # Save baseline BEFORE any write; failure to save prevents tuning.
            with log.open('a') as stream:
                stream.write(json.dumps({'time_ns': time.time_ns(), 'joint': joint,
                                         'baseline_p': self.original[joint], 'old_p': old,
                                         'requested_p': desired}) + '\n')
            self.bus.write('Lock', joint, 1, normalize=False)
            self.bus.write('P_Coefficient', joint, desired, normalize=False)
            actual = self.bus.read('P_Coefficient', joint, normalize=False)
            if actual != desired:
                raise RuntimeError(f'Readback {actual} differs from requested {desired}')
            self.node.get_logger().info(f'{joint}: P {old} -> {actual}; I/D unchanged; baseline={self.original[joint]}; RAM-only')
            return SetParametersResult(successful=True)
        except Exception as exc:
            if old is not None:
                try:
                    self.bus.write('Lock', joint, 1, normalize=False)
                    self.bus.write('P_Coefficient', joint, old, normalize=False)
                except Exception as rollback:
                    self.node.get_logger().error(f'Gain rollback failed: {rollback}')
            return SetParametersResult(successful=False, reason=str(exc))
