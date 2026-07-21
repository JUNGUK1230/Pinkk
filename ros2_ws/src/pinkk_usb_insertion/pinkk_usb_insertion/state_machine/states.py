"""USB 삽입 상태와 허용 전이를 정의한다."""

from __future__ import annotations

from enum import Enum


class InsertionState(str, Enum):
    IDLE = 'IDLE'
    ACQUIRE_PORT = 'ACQUIRE_PORT'
    ESTIMATE_POSE = 'ESTIMATE_POSE'
    CALCULATE_APPROACH = 'CALCULATE_APPROACH'
    DRY_RUN_COMPLETE = 'DRY_RUN_COMPLETE'
    MOVE_PRE_APPROACH = 'MOVE_PRE_APPROACH'
    VISUAL_ALIGN = 'VISUAL_ALIGN'
    INSERT = 'INSERT'
    VERIFY = 'VERIFY'
    COMPLETE = 'COMPLETE'
    RETREAT = 'RETREAT'
    ERROR = 'ERROR'


ALLOWED_TRANSITIONS: dict[InsertionState, set[InsertionState]] = {
    InsertionState.IDLE: {InsertionState.ACQUIRE_PORT},
    InsertionState.ACQUIRE_PORT: {InsertionState.ESTIMATE_POSE, InsertionState.ERROR},
    InsertionState.ESTIMATE_POSE: {InsertionState.CALCULATE_APPROACH, InsertionState.ERROR},
    InsertionState.CALCULATE_APPROACH: {
        InsertionState.DRY_RUN_COMPLETE,
        InsertionState.MOVE_PRE_APPROACH,
        InsertionState.ERROR,
    },
    InsertionState.DRY_RUN_COMPLETE: {InsertionState.IDLE},
    InsertionState.MOVE_PRE_APPROACH: {
        InsertionState.VISUAL_ALIGN,
        InsertionState.RETREAT,
        InsertionState.ERROR,
    },
    InsertionState.VISUAL_ALIGN: {
        InsertionState.INSERT,
        InsertionState.RETREAT,
        InsertionState.ERROR,
    },
    InsertionState.INSERT: {
        InsertionState.VERIFY,
        InsertionState.RETREAT,
        InsertionState.ERROR,
    },
    InsertionState.VERIFY: {
        InsertionState.COMPLETE,
        InsertionState.RETREAT,
        InsertionState.ERROR,
    },
    InsertionState.COMPLETE: {InsertionState.IDLE},
    InsertionState.RETREAT: {InsertionState.IDLE, InsertionState.ERROR},
    InsertionState.ERROR: {InsertionState.IDLE},
}
