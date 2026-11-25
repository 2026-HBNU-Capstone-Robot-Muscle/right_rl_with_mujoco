"""Isolated retimed vertical trial; original controller files stay untouched."""
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path

import numpy as np

import leap_measured_grip_hold_20261117 as pose
from vertical_direct_grip_plan_20261118 import VerticalDirectGrip, CHECKPOINTS
from vertical_speed_retime_20261125 import select_rows

ROOT = Path(__file__).resolve().parent
BASE = ROOT/'right_hand_rotation_20261125/vertical_3x_separate_trial'
PERIOD = .075
FACTOR = 3
FAST_STEP = 6
ORIGINAL_VALIDATOR = pose.validate_goal_return


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class VerticalSpeedTrial:
    def __init__(self):
        self.manifest = json.loads((BASE/'trial_manifest.json').read_bytes())
        for name, expected in self.manifest['source_hashes'].items():
            if digest(ROOT/name) != expected:
                raise RuntimeError('Fast trial source changed: '+name)
        if digest(BASE/'candidate.npz') != self.manifest['candidate_sha256']:
            raise RuntimeError('Fast trial candidate changed.')
        self.original = VerticalDirectGrip()
        self.start = self.original.start.copy()
        if self.manifest['baseline_candidate_sha256'] != self.original.manifest['candidate_sha256']:
            raise RuntimeError('Baseline vertical candidate changed.')
        self.moving = False
        self.index = 0
        self.labels = []
        self.original_rows = None
        self.actual_rows = None
        self.indices = None

    def authorized(self):
        return self.manifest.get('operator_authorized_maximum_step_ticks') == FAST_STEP

    def rows(self, current):
        self.moving = False
        self.original_rows = self.original.rows(current)
        rows, labels, indices, mandatory = select_rows(current, self.original_rows,
                                                     self.original.labels, FACTOR)
        self.labels, self.indices = labels.tolist(), indices
        self.actual_rows, self.mandatory = rows, mandatory
        self.entry = np.asarray(current).copy()
        if not np.array_equal(rows[-1], self.start):
            raise RuntimeError('Fast trial must finish at the same G.')
        return rows

    def phase(self, index, moving):
        self.index, self.moving = index, moving
        return self.labels[index] if moving else 'fast_trial_fixed_G_hold'

    def _limit(self):
        if not self.moving:
            return 2
        phase = self.labels[self.index]
        return FAST_STEP if phase.startswith(('direct_forward_', 'direct_return_')) else 2

    def validate_packet(self, value, previous, entry):
        target = pose.encoder_vector(value)
        old = pose.encoder_vector(previous)
        step = int(np.max(abs(target-old)))
        if step <= 2:
            return ORIGINAL_VALIDATOR(target, old, entry)
        if step > self._limit() or self.actual_rows is None or not self.moving:
            raise RuntimeError('Goal exceeds this isolated fast-trial step scope.')
        if (not np.array_equal(target, self.actual_rows[self.index])
                or not np.array_equal(old, self.actual_rows[self.index-1] if self.index else self.entry)):
            raise RuntimeError('Fast packet is not the exact next source-selected target.')
        left = int(self.indices[self.index-1]) if self.index else -1
        right = int(self.indices[self.index])
        checked = old
        for source_index in range(left+1, right+1):
            # Validation only; no hidden motor writes. Each intermediate source
            # point must still pass the unchanged range/return/2tick function.
            checked = ORIGINAL_VALIDATOR(self.original_rows[source_index], checked, entry)
        if not np.array_equal(checked, target):
            raise RuntimeError('Fast packet source proof did not reach its target.')
        return target

    @contextmanager
    def runtime_scope(self):
        import goal_readback_confirmation as readback
        if pose.validate_goal_return is not ORIGINAL_VALIDATOR or readback.MAX_STEP_TICKS != 2:
            raise RuntimeError('Unexpected global packet scope; no devices should be opened.')
        old = pose.validate_goal_return
        pose.validate_goal_return = self.validate_packet
        # Same exact-preceding-packet recovery,150ms/3 reads/2 exact reads and
        # repeated-mismatch trip. Only its packet-size eligibility follows6ticks.
        readback.MAX_STEP_TICKS = FAST_STEP
        try:
            yield
        finally:
            pose.validate_goal_return = old
            readback.MAX_STEP_TICKS = 2

    def metadata(self):
        value = self.original.metadata()
        value.update(workflow='isolated_vertical_speed_trial',
            phase_checkpoint_indices={p:self.labels.index(p) for p in CHECKPOINTS},
            period_seconds=PERIOD, maximum_step_ticks=FAST_STEP,
            entry_maximum_step_ticks=2, original_full_health_transport=True,
            original_source_goals=len(self.original_rows), selected_source_indices=self.indices.tolist(),
            retained_mandatory_indices=self.mandatory.tolist(),
            theoretical_total_speed_ratio=len(self.original_rows)/len(self.actual_rows),
            actual_speed_and_cube_stability_verified=False,
            readback_recovery_exact_preceding_packet_maximum_ticks=FAST_STEP,
            original_health_tracking_current_PID_and_recovery_budgets_unchanged=True)
        return value

    @staticmethod
    def period(index, moving):
        return PERIOD

    @staticmethod
    def ready(index):
        return True

    def message(self, phase):
        if phase == 'fast_trial_fixed_G_hold':
            return '속도 시험 경로 끝. G 유지 중. 큐브를 받친 뒤 SPACE로 종료/OFF.'
        return self.original.message(phase)
