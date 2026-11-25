"""Isolated3x source selection with the original bounds and same-goal recovery."""
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path

import numpy as np

import leap_measured_grip_hold_20261117 as pose
from build_vertical_direct_grip_20261118 import grip_entry
from build_taught_palm_transfer_20261118 import interpolate
from two_axis_cycle_plan_20261124 import TwoAxisCycle, ORIGINAL_BOUNDS
from vertical_speed_retime_20261125 import select_rows
from vertical_speed_trial_plan_20261125 import VerticalSpeedTrial

ROOT = Path(__file__).resolve().parent
BASE = ROOT/'right_hand_rotation_20261125/rotation_3x_with_recovery'
EXTRA_TIP_CURL_TICKS = 64  # 5.625 degrees; no joint-limit expansion.
MIDDLE_ROOT_LOWER_TICKS = 46  # 4.043 degrees below the index-like support.
TIP_CURL_PHASE = 'return_tips_curl_before_lower_push'
INDEX_FIRST_PHASES = ('return_grip_index_middle_seat', 'return_grip_lower_push', 'return_fixed_grip_close')
VERTICAL_INDEX_FIRST_PHASES = ('direct_return_final_taught_1', 'direct_return_final_taught_2',
                             'direct_return_final_taught_to_G')
SLOW_SEATING_PHASES = frozenset('trial_horizontal_'+phase for phase in (
    'return_thumb_lower_before_support', 'return_middle_root_lead', 'return_grip_index_middle_seat',
    TIP_CURL_PHASE, 'return_grip_lower_push', 'return_fixed_grip_close',
    *(p+'_index_lead' for p in INDEX_FIRST_PHASES)))


def horizontal_middle_index_support(rows, phases):
    """Replace excessive middle withdrawal with index-like flexion support.

    Match corresponding bend joints (9/11/12 <- 5/7/8), then lower ID12 by
    about4degrees as requested. Preserve the
    middle spread target and all non-middle stage endpoints. Hold the support
    through thumb pushing; retain the later tip curl and exact final G.
    """
    rows, phases = np.asarray(rows, dtype=int), np.asarray(phases)
    first = int(np.flatnonzero(phases == 'return_transfer_middle_clear')[0])
    last = int(np.flatnonzero(phases == 'return_fixed_grip_close')[-1])+1
    supported = rows[first-1].copy()
    supported[[8, 10, 11]] = supported[[4, 6, 7]]
    supported[11] -= MIDDLE_ROOT_LOWER_TICKS
    rebuilt, labels = [rows[:first]], list(phases[:first])
    previous = rows[first-1].copy()
    span = phases[first:last]
    boundaries = np.r_[0, np.flatnonzero(span[1:] != span[:-1])+1, len(span)]
    for left, right in zip(boundaries[:-1], boundaries[1:]):
        name = str(span[left])
        target = rows[first+right-1].copy()
        if name in ('return_transfer_middle_clear', 'return_transfer_thumb_prepare',
                    'return_transfer_thumb_push'):
            target[[8, 10, 11]] = supported[[8, 10, 11]]
        elif name in ('return_transfer_thumb_wrist_and_tips', 'return_middle_root_lead'):
            target[[10, 11]] = supported[[10, 11]]
        else:
            # Do not lower the root back to the old over-withdrawn target
            # during the following seat/lower-push stages.
            target[11] = max(target[11], supported[11])
        count = max(int(right-left), int(np.ceil(np.max(abs(target-previous))/2)), 1)
        segment = np.rint(previous+(target-previous)*np.arange(1, count+1)[:, None]/count).astype(int)
        rebuilt.append(segment)
        labels.extend([name]*len(segment))
        previous = target
    rebuilt.append(rows[last:])
    labels.extend(phases[last:])
    return np.vstack(rebuilt), np.asarray(labels)


def horizontal_thumb_first_return(rows, phases, grip):
    """Keep the taught transfer, then lower the thumb before seating fingers.

    The ring clears with the thumb, as in the approved vertical return. Keep
    the revised index/middle waypoints and hold the lowered thumb/ring at G
    throughout the remaining seating and closing motion.
    """
    rows = np.asarray(rows, dtype=int)
    phases = np.asarray(phases)
    begin = int(np.flatnonzero(phases == 'return_middle_root_lead')[0])
    support = rows[begin-1].copy()
    lowered = support.copy()
    lowered[:4], lowered[12:] = grip[:4], grip[12:]
    lower_rows = interpolate(support, lowered)
    suffix = rows[begin:].copy()
    suffix[:, :4], suffix[:, 12:] = grip[:4], grip[12:]
    return (np.vstack([rows[:begin], lower_rows, suffix]),
            np.asarray([*phases[:begin],
                        *(['return_thumb_lower_before_support']*len(lower_rows)),
                        *phases[begin:]]))


def horizontal_tips_before_lower_push(rows, phases):
    """Curl only IDs5/9 before lower-push, then return smoothly to exact G."""
    rows, phases = np.asarray(rows, dtype=int), np.asarray(phases)
    push = np.flatnonzero(phases == 'return_grip_lower_push')
    close = np.flatnonzero(phases == 'return_fixed_grip_close')
    begin, push_end = int(push[0]), int(push[-1])+1
    close_begin, close_end = int(close[0]), int(close[-1])+1
    if close_begin != push_end:
        raise RuntimeError('Expected lower-push directly before final closing')
    before = rows[begin-1].copy()
    curled = before.copy()
    tips = [4, 8]
    curled[tips] += EXTRA_TIP_CURL_TICKS
    prepare = interpolate(before, curled)
    pushed = rows[begin:push_end].copy()
    pushed[:, tips] += EXTRA_TIP_CURL_TICKS
    closing = rows[close_begin:close_end].copy()
    # Preserve every other joint and the final G. The tips relax from the
    # deeper curl during the existing closing interval, without a step back.
    final_tips = closing[-1, tips].copy()
    closing[:, tips] = np.rint(pushed[-1, tips]+(final_tips-pushed[-1, tips])*
        np.arange(1, len(closing)+1)[:, None]/len(closing)).astype(int)
    return (np.vstack([rows[:begin], prepare, pushed, closing, rows[close_end:]]),
            np.asarray([*phases[:begin], *([TIP_CURL_PHASE]*len(prepare)),
                        *phases[begin:]]))


def index_first_then_middle(rows, phases, *, phases_to_split=INDEX_FIRST_PHASES):
    """Finish the index path, then move the middle, for each seating stage.

    Index follows every existing goal first. Middle initially holds its
    previous target until the index reaches that stage's endpoint.
    The index holds its endpoint while the middle finishes. No angles or
    final waypoints are added; thumb/ring remain fixed during these stages.
    """
    rows, phases = np.asarray(rows, dtype=int), np.asarray(phases)
    bounds = np.r_[0, np.flatnonzero(phases[1:] != phases[:-1])+1, len(phases)]
    chunks, labels = [], []
    for left, right in zip(bounds[:-1], bounds[1:]):
        name = str(phases[left])
        segment = rows[left:right]
        if name not in phases_to_split:
            chunks.append(segment)
            labels.extend([name]*len(segment))
            continue
        if left == 0:
            raise RuntimeError('Index-first seating must have a preceding support goal')
        delay = len(segment)
        extended = segment[np.minimum(np.arange(len(segment)+delay), len(segment)-1)].copy()
        extended[:delay, 8:12] = rows[left-1, 8:12]
        extended[delay:, 8:12] = segment[:, 8:12]
        chunks.append(extended)
        labels.extend([name+'_index_lead']*delay+[name]*len(segment))
    return np.vstack(chunks), np.asarray(labels)


class RotationSpeedTrial(VerticalSpeedTrial):
    def __init__(self, axis, *, verify=True):
        if axis not in ('horizontal', 'vertical'):
            raise ValueError('Choose horizontal or vertical')
        self.axis = axis
        self.manifest = {}
        if verify:
            base = BASE/axis
            self.manifest = json.loads((base/'manifest.json').read_bytes())
            for name, expected in self.manifest['source_hashes'].items():
                if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
                    raise RuntimeError('New3x trial source changed: '+name)
            if hashlib.sha256((base/'candidate.npz').read_bytes()).hexdigest() != self.manifest['candidate_sha256']:
                raise RuntimeError('New3x trial candidate changed')
        self.reference = TwoAxisCycle()
        self.start = self.reference.start.copy()
        leg = self.reference.horizontal if axis == 'horizontal' else self.reference.vertical
        self.core, self.core_phases = leg.goals.copy(), leg.phases.copy()
        self.baseline_core_count = len(self.core)
        self.index_first_phases = INDEX_FIRST_PHASES if axis=='horizontal' else VERTICAL_INDEX_FIRST_PHASES
        if axis == 'horizontal':
            self.core, self.core_phases = horizontal_middle_index_support(self.core, self.core_phases)
            self.core, self.core_phases = horizontal_thumb_first_return(
                self.core, self.core_phases, self.start)
            self.core, self.core_phases = horizontal_tips_before_lower_push(
                self.core, self.core_phases)
        self.core, self.core_phases = index_first_then_middle(
            self.core, self.core_phases, phases_to_split=self.index_first_phases)
        self.moving, self.index, self.labels = False, 0, []
        self.original_rows = self.actual_rows = self.indices = None
        if verify and self.manifest['baseline_candidate_sha256'] != self.reference.manifest['candidate_sha256']:
            raise RuntimeError('Original two-axis source candidate changed')

    def rows(self, current):
        self.moving = False
        self.reference.active_leg = 'entry'
        intro, labels = grip_entry(current, self.start)
        self.reference.check_rows(current, intro, ORIGINAL_BOUNDS)
        bounds = self.reference.horizontal_bounds if self.axis == 'horizontal' else ORIGINAL_BOUNDS
        self.reference.check_rows(self.start, self.core, bounds)
        # G entry and forward rotation retain the baseline. Both returns now
        # complete each final index path before moving the middle finger.
        # No legacy horizontal SPACE gates or500s cutoff are introduced.
        dense = np.asarray([*intro, *np.tile(self.start, (40, 1)), self.start.copy(),
                            *self.core, self.start.copy()], dtype=int)
        phases = [*labels, *(['direct_entry_settle']*40), 'trial_start_G_check',
                  *('trial_'+self.axis+'_'+str(p) for p in self.core_phases), 'trial_final_G_check']
        # The3x trial showed large ring-finger lag and repeated3.6V dips in
        # final closing. Keep the original rate through thumb lowering and
        # receiving/seating. Joint limits and the final G are unchanged.
        selector = ['direct_entry_slow_seating_'+p if p in SLOW_SEATING_PHASES else p for p in phases]
        chosen, _, indices, mandatory = select_rows(current, dense, selector, 3)
        selected_phases = np.asarray(phases)[indices]
        self.original_rows, self.actual_rows = dense, chosen
        self.labels, self.indices, self.mandatory = selected_phases.tolist(), indices, mandatory
        self.entry = np.asarray(current).copy()
        if not np.array_equal(chosen[-1], self.start):
            raise RuntimeError('Experimental endpoint differs from fixed G')
        return chosen

    def phase(self, index, moving):
        self.index, self.moving = index, moving
        phase = self.labels[index] if moving else 'trial_fixed_G_hold'
        self.reference.active_leg = self.axis if phase.startswith('trial_'+self.axis+'_') else 'entry'
        return phase

    def _limit(self):
        return (6 if self.moving and self.labels[self.index].startswith('trial_'+self.axis+'_')
                and self.labels[self.index] not in SLOW_SEATING_PHASES else 2)

    @contextmanager
    def runtime_scope(self):
        with self.reference.runtime_limits(), super().runtime_scope():
            yield

    def metadata(self):
        return dict(workflow='experimental3x_'+self.axis+'_with_READ_recovery',
            experimental_speed_factor=3, maximum_step_ticks=6, entry_maximum_step_ticks=2,
            period_seconds=.075, original_source_goals=len(self.original_rows),
            selected_source_indices=self.indices.tolist(), retained_mandatory_indices=self.mandatory.tolist(),
            theoretical_total_speed_ratio=len(self.original_rows)/len(self.actual_rows),
            nominal_original_seconds=len(self.original_rows)*.075,
            nominal_selected_seconds=len(self.actual_rows)*.075,
            fixed_grip_target_ticks=self.start.tolist(), phase_gate_indices={},
            automatic_stage_transitions_enabled=True, maximum_ON_seconds=None,
            elapsed_time_limit_enabled=False, original_joint_bounds_unchanged=True,
            original_current_PID_and_health_limits_unchanged=True,
            horizontal_final_seating_maximum_step_ticks=2,
            horizontal_slow_seating_phases=sorted(SLOW_SEATING_PHASES) if self.axis=='horizontal' else [],
            baseline_core_goals=self.baseline_core_count,
            horizontal_return_order_modified=self.axis=='horizontal',
            vertical_return_order_modified=self.axis=='vertical',
            thumb_G_before_index_middle_seating=True,
            ring_clears_with_thumb_then_held_at_G=True,
            source_selection_uses_revised_dense_baseline=True,
            middle_index_like_support_enabled=self.axis=='horizontal',
            middle_support_matched_motor_ids=[9, 11, 12] if self.axis=='horizontal' else [],
            middle_spread_matching_changed=False,
            middle_root_lower_than_index_ticks=MIDDLE_ROOT_LOWER_TICKS if self.axis=='horizontal' else 0,
            middle_root_lower_than_index_degrees=MIDDLE_ROOT_LOWER_TICKS*360/4096 if self.axis=='horizontal' else 0,
            extra_tip_curl_before_lower_push_ticks=EXTRA_TIP_CURL_TICKS if self.axis=='horizontal' else 0,
            extra_tip_curl_motor_ids=[5, 9] if self.axis=='horizontal' else [],
            index_first_seating_phases=list(self.index_first_phases),
            middle_follows_after_index_stage_complete=True,
            middle_follow_delay_goals_by_phase={p:int(np.sum(self.core_phases==p+'_index_lead'))
                for p in self.index_first_phases},
            actual_speed_and_cube_stability_verified=False)

    def message(self, phase):
        if phase.startswith('direct_entry_'):
            return self.reference.vertical.message(phase)
        if phase == 'trial_start_G_check':
            return 'G 준비 끝. 별도3배속 회전/복귀로 자동 진행합니다. SPACE/Q/Esc: 중단/OFF.'
        if phase in ('trial_final_G_check', 'trial_fixed_G_hold'):
            return '실험 경로 끝. G 유지 중입니다. 큐브를 받친 뒤 SPACE로 종료/OFF.'
        prefix = 'trial_'+self.axis+'_'
        if phase.startswith(prefix):
            core = phase[len(prefix):]
            if core in tuple(p+'_index_lead' for p in self.index_first_phases):
                return ('중지는 현재 받침을 유지하고 검지가 이 단계 끝까지 먼저 움직입니다. '
                        '엄지·약지는 유지합니다. SPACE/Q/Esc: 중단/OFF.')
            if core in self.index_first_phases:
                return ('검지는 이 단계 끝 자세를 유지하고 중지만 이어서 움직입니다. '
                        'SPACE/Q/Esc: 중단/OFF.')
            if self.axis == 'horizontal' and core == TIP_CURL_PHASE:
                return ('엄지·약지와 나머지 마디는 유지하고 검지·중지 끝마디만 먼저64틱(약5.6도) 더 굽힙니다. '
                        '이 모양을 유지하며 아래를 받쳐 밉니다. 기존1배속/2틱. SPACE/Q/Esc: 중단/OFF.')
            if self.axis == 'horizontal' and core == 'return_transfer_middle_clear':
                return ('복귀: 엄지·검지·약지는 유지하고 중지를 검지와 비슷한 받침으로 맞추되 '
                        '중지 맨 아랫마디는 약4도 더 내립니다. SPACE/Q/Esc: 중단/OFF.')
            if self.axis == 'horizontal' and core == 'return_middle_root_lead':
                return ('중지 아랫마디를 검지와 비슷한 받침 높이에 유지하고 파지로 이어집니다. '
                        '기존1배속/2틱. SPACE/Q/Esc: 중단/OFF.')
            if core == 'return_thumb_lower_before_support':
                return ('검지·중지는 그대로 유지하고 엄지를 먼저 확정 파지 위치로 내립니다. '
                        '약지는 엄지 길을 함께 비킵니다. 기존1배속/2틱. SPACE/Q/Esc: 중단/OFF.')
            if phase in SLOW_SEATING_PHASES:
                return '마지막 받침/파지는 기존1배속으로 진행합니다. '+self.reference.message('chain_h_'+core)
            if self.axis == 'horizontal' and core == 'return_fixed_grip_hold':
                return '가로 복귀 G에서 기존 약10초 정착 시간을 유지합니다. SPACE/Q/Esc: 중단/OFF.'
            return self.reference.message(('chain_h_' if self.axis == 'horizontal' else 'chain_v_')+core)
        return phase+' / SPACE/Q/Esc: 중단/OFF.'
