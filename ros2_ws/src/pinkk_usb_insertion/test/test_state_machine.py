from pinkk_usb_insertion.state_machine.insertion_state_machine import (
    InsertionStateMachine,
)
from pinkk_usb_insertion.state_machine.states import InsertionState
import pytest


def test_dry_run_path() -> None:
    machine = InsertionStateMachine()
    machine.transition(InsertionState.ACQUIRE_PORT)
    machine.transition(InsertionState.ESTIMATE_POSE)
    machine.transition(InsertionState.CALCULATE_APPROACH)
    machine.transition(InsertionState.DRY_RUN_COMPLETE)
    assert machine.state == InsertionState.DRY_RUN_COMPLETE


def test_invalid_transition_is_rejected() -> None:
    machine = InsertionStateMachine()
    with pytest.raises(ValueError):
        machine.transition(InsertionState.INSERT)
