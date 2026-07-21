from pinkk_usb_insertion.configuration import execution_gate, insertion_gate


def test_execution_is_disabled_by_default() -> None:
    control = {
        'execution': {
            'execution_enabled': False,
            'insertion_enabled': False,
            'require_calibrated_tool': True,
        }
    }
    tool = {'tool': {'calibrated': False}}
    assert execution_gate(control, tool) == (False, 'execution_enabled=false')
    assert insertion_gate(control, tool) == (False, 'execution_enabled=false')


def test_insertion_requires_its_own_switch() -> None:
    control = {
        'execution': {
            'execution_enabled': True,
            'insertion_enabled': False,
            'require_calibrated_tool': True,
        }
    }
    tool = {'tool': {'calibrated': True}}
    assert execution_gate(control, tool)[0]
    assert insertion_gate(control, tool) == (False, 'insertion_enabled=false')
