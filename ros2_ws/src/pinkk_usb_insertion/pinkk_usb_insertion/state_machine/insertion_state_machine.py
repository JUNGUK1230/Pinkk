"""ROS 통신과 분리된 결정론적 USB 삽입 상태 머신."""

from __future__ import annotations

from dataclasses import dataclass, field

from .states import ALLOWED_TRANSITIONS, InsertionState


@dataclass
class InsertionStateMachine:
    state: InsertionState = InsertionState.IDLE
    history: list[InsertionState] = field(default_factory=lambda: [InsertionState.IDLE])

    def transition(self, target: InsertionState) -> None:
        if target not in ALLOWED_TRANSITIONS[self.state]:
            raise ValueError(f'허용되지 않은 상태 전이: {self.state.value} -> {target.value}')
        self.state = target
        self.history.append(target)

    def reset(self) -> None:
        if self.state != InsertionState.IDLE:
            self.transition(InsertionState.IDLE)
