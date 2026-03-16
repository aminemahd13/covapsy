#!/usr/bin/env python3
from dataclasses import dataclass
from enum import IntEnum


class RecoveryStage(IntEnum):
    NONE = 0
    BRAKE = 1
    REVERSE = 2
    REASSESS = 3
    ESCALATE = 4
    FAILSAFE_STOP = 5


@dataclass
class RecoveryCommand:
    speed_mps: float
    steer_rad: float
    stage: RecoveryStage
    reason: str


class RecoveryFSM:
    def __init__(
        self,
        max_attempts: int,
        brake_steps: int,
        reverse_steps: int,
        reassess_steps: int,
        forward_reorient_steps: int = 8,
    ):
        self.max_attempts = max_attempts
        self.brake_steps = brake_steps
        self.reverse_steps = reverse_steps
        self.reassess_steps = reassess_steps
        self.forward_reorient_steps = forward_reorient_steps
        self.stage = RecoveryStage.NONE
        self.attempt = 0
        self.counter = 0
        self.reverse_blocked = False
        self.forward_reorient_remaining = 0

    def reset(self):
        self.stage = RecoveryStage.NONE
        self.counter = 0
        self.attempt = 0
        self.reverse_blocked = False
        self.forward_reorient_remaining = 0

    def reset_runtime(self):
        self.stage = RecoveryStage.NONE
        self.counter = 0
        self.reverse_blocked = False
        self.forward_reorient_remaining = 0

    def mark_reverse_blocked(self, boxed_in: bool):
        if boxed_in:
            self.stage = RecoveryStage.FAILSAFE_STOP
            self.counter = 0
            self.reverse_blocked = True
            self.forward_reorient_remaining = 0
            return
        self.reverse_blocked = True
        self.forward_reorient_remaining = self.forward_reorient_steps
        self.stage = RecoveryStage.BRAKE
        self.counter = self.brake_steps

    def trigger(self):
        if self.stage == RecoveryStage.NONE:
            self.stage = RecoveryStage.BRAKE
            self.counter = self.brake_steps

    def step(self, open_side_sign: float, allow_forward_reorient: bool = False) -> RecoveryCommand:
        if self.stage == RecoveryStage.NONE:
            return RecoveryCommand(0.0, 0.0, self.stage, 'none')

        if self.stage == RecoveryStage.BRAKE:
            self.counter -= 1
            if self.counter <= 0:
                if self.reverse_blocked:
                    self.stage = RecoveryStage.REASSESS
                    self.counter = self.reassess_steps
                else:
                    self.stage = RecoveryStage.REVERSE
                    self.counter = self.reverse_steps
            return RecoveryCommand(0.0, 0.0, RecoveryStage.BRAKE, 'brake_pulse')

        if self.stage == RecoveryStage.REVERSE:
            self.counter -= 1
            steer = 0.25 * open_side_sign
            if self.counter <= 0:
                self.stage = RecoveryStage.REASSESS
                self.counter = self.reassess_steps
            return RecoveryCommand(-0.35, steer, RecoveryStage.REVERSE, 'reverse_to_open_side')

        if self.stage == RecoveryStage.REASSESS:
            if self.reverse_blocked and allow_forward_reorient and self.forward_reorient_remaining > 0:
                self.forward_reorient_remaining -= 1
                if self.forward_reorient_remaining <= 0:
                    self.attempt += 1
                    if self.attempt >= self.max_attempts:
                        self.stage = RecoveryStage.FAILSAFE_STOP
                    else:
                        self.reverse_blocked = False
                        self.stage = RecoveryStage.ESCALATE
                return RecoveryCommand(0.18, 0.20 * open_side_sign, RecoveryStage.REASSESS, 'forward_reorient')

            self.counter -= 1
            if self.counter <= 0:
                self.attempt += 1
                if self.attempt >= self.max_attempts:
                    self.stage = RecoveryStage.FAILSAFE_STOP
                else:
                    self.reverse_blocked = False
                    self.stage = RecoveryStage.ESCALATE
            return RecoveryCommand(0.0, 0.0, RecoveryStage.REASSESS, 'reassess')

        if self.stage == RecoveryStage.ESCALATE:
            # Hand off to nominal mode for retry; next trigger starts a new cycle.
            self.stage = RecoveryStage.NONE
            self.counter = 0
            return RecoveryCommand(0.0, 0.0, RecoveryStage.ESCALATE, 'retry')

        return RecoveryCommand(0.0, 0.0, RecoveryStage.FAILSAFE_STOP, 'failsafe_stop')
