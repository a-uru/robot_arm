from types import SimpleNamespace
from unittest.mock import Mock
from so101_hw_interface.gain_tuning import GainTuning


def test_small_gain_change_backup_and_restore(tmp_path):
    node, bus = Mock(), Mock()
    node.get_parameter.return_value.value = str(tmp_path / 'gain.jsonl')
    values = {'P_Coefficient': 16}
    bus.read.side_effect = lambda key, joint, **kw: values[key]
    bus.write.side_effect = lambda key, joint, value, **kw: values.update({key: value})
    tuning = GainTuning(node, bus)
    name = 'tuning.shoulder_lift.p'
    assert tuning.change([SimpleNamespace(name=name, value=20)]).successful
    assert values['P_Coefficient'] == 20
    assert values['Lock'] == 1
    assert (tmp_path / 'gain.jsonl').is_file()
    assert not tuning.change([SimpleNamespace(name=name, value=28)]).successful
    assert values['P_Coefficient'] == 20
    assert tuning.change([SimpleNamespace(name=name, value=-1)]).successful
    assert values['P_Coefficient'] == 16
    assert all(call.args[0] in ('P_Coefficient', 'Lock') for call in bus.write.call_args_list)


def test_failed_readback_restores_previous_gain(tmp_path):
    node, bus = Mock(), Mock()
    node.get_parameter.return_value.value = str(tmp_path / 'gain.jsonl')
    bus.read.side_effect = [16, 17]
    tuning = GainTuning(node, bus)
    assert not tuning.change([SimpleNamespace(name='tuning.elbow_flex.p', value=20)]).successful
    assert bus.write.call_args.args == ('P_Coefficient', 'elbow_flex', 16)
