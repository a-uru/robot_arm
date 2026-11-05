from types import SimpleNamespace
from unittest.mock import Mock
import json
from so101_hw_interface.motor_diagnostics import MotorDiagnostics, SETTINGS


def test_disabled_diagnostics_performs_no_bus_io_then_reads_without_writes():
    node, bus = Mock(), Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 1
    enabled = SimpleNamespace(value=False)
    node.get_parameter.return_value = enabled
    diag = MotorDiagnostics(node, bus, ['shoulder_lift'])
    diag.sample()
    assert not bus.mock_calls
    enabled.value = True
    bus.read.return_value = 32
    for _ in SETTINGS:
        diag.sample()
    bus.sync_read.return_value = {'shoulder_lift': 74}
    diag.sample()
    assert all(call[0] in ('read', 'sync_read') for call in bus.mock_calls)
    record = json.loads(node.create_publisher.return_value.publish.call_args.args[0].data)
    assert record['register'] == 'Present_Voltage'
    assert record['values']['shoulder_lift'] == 74
