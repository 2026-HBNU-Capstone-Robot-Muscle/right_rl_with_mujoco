"""One torque session: SPACE prepares G; A/B select a checked rotation."""
import argparse
from contextlib import contextmanager
import datetime as dt
import hashlib
import json
import time

import numpy as np

import calibrated_two_full_health_20261028 as calibrated
import leap_measured_grip_hold_20261117 as pose
import leap_vertical_direct_grip_recovery_20261125 as recovery_entry
import leap_vertical_direct_grip_resume_20261125 as resume_entry
from build_vertical_direct_grip_20261118 import grip_entry
from grip_pause_resume_10s_trial_20261125 import sample
from rotation_speed_resume_plan_20261125 import ROOT, RotationSpeedTrial, SLOW_SEATING_PHASES
from speed_trial_state_observer_20261125 import ObservedPauseBus as PauseBus
from two_axis_cycle_plan_20261124 import ORIGINAL_BOUNDS
from vertical_speed_trial_plan_20261125 import VerticalSpeedTrial

BASE = ROOT/'right_hand_rotation_20261125/keyboard_rotate_grip'
G_HOLD = 'keyboard_checked_G_hold'


class KeyboardRotationSequence(VerticalSpeedTrial):
    def __init__(self, report, *, verify=True):
        if verify:
            manifest = json.loads((BASE/'manifest.json').read_bytes())
            for name, digest in manifest['source_hashes'].items():
                if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest:
                    raise RuntimeError('Keyboard rotation source changed: '+name)
        self.report = report
        self.plans = {axis:RotationSpeedTrial(axis) for axis in ('horizontal', 'vertical')}
        self.reference = self.plans['horizontal'].reference
        self.start = self.plans['horizontal'].start.copy()
        self.compiled = {}
        for axis, plan in self.plans.items():
            if not np.array_equal(plan.start, self.start):
                raise RuntimeError('A/B paths must share the exact same G')
            rows = plan.rows(self.start)
            self.compiled[axis] = dict(rows=rows.copy(), dense=plan.original_rows.copy(),
                indices=plan.indices.copy(), labels=list(plan.labels))
        self.bus = None
        self.moving, self.index = False, 0
        self.ready_for_input = False
        self.active = self.pending = self.previous_hold_state = None
        self.confirmed = 0
        self.actual_rows, self.original_rows, self.indices, self.labels, self.legs = [], [], [], [], []

    def authorized(self):
        return all(plan.authorized() for plan in self.plans.values())

    def rows(self, current):
        self.reference.active_leg = 'entry'
        intro, labels = grip_entry(current, self.start)
        self.reference.check_rows(current, intro, ORIGINAL_BOUNDS)
        initial = [*intro, *np.tile(self.start, (40, 1)), self.start.copy()]
        self.actual_rows = [q.copy() for q in initial]
        self.original_rows = [q.copy() for q in initial]
        self.indices = list(range(len(initial)))
        self.labels = [*labels, *(['direct_entry_settle']*40), 'keyboard_initial_G_check']
        self.legs = ['entry']*len(initial)
        self.entry = np.asarray(current).copy()
        self.moving, self.index, self.confirmed = False, 0, 0
        self.active = self.pending = self.previous_hold_state = None
        self.ready_for_input = False
        self.report['keyboard_ready'] = False
        return self.actual_rows

    def request_axis(self, axis):
        if axis not in self.plans:
            raise ValueError('Unknown keyboard rotation axis')
        if (not self.ready_for_input or self.pending is not None or self.active is not None
                or self.bus is None or getattr(self.bus, '_motion_paused', False)):
            self.report['ignored_rotation_keys'] = self.report.get('ignored_rotation_keys', 0)+1
            return False
        self.pending = axis
        self.ready_for_input = False
        self.report['keyboard_ready'] = False
        return True

    def _confirm_hold(self):
        state = getattr(self.bus, 'last_reviewed_state', None)
        if self.report.get('phase') != G_HOLD or state is None or state is self.previous_hold_state:
            return
        self.previous_hold_state = state
        good = (np.array_equal(state['goal'], self.start) and np.all(state['torque'] == 1)
                and not np.any(state['error']) and np.all(state['watchdog'] == 25)
                and np.max(abs(state['position']-self.start))*2*np.pi/4096 <= .12)
        self.confirmed = self.confirmed+1 if good else 0
        if self.confirmed < 3:
            self.ready_for_input = False
            self.report['keyboard_ready'] = False
            return
        if self.active is not None:
            self.report.setdefault('completed_commands', []).append(dict(
                **self.active, checked_G_samples=3))
            self.active = None
        if self.pending is None and not self.ready_for_input:
            print('G 복귀/유지 확인. A: 가로 / B: 세로 / SPACE·Q·Esc: 종료/OFF.', flush=True)
        self.ready_for_input = self.pending is None
        self.report['keyboard_ready'] = self.ready_for_input

    def _append_pending(self):
        axis, self.pending = self.pending, None
        compiled = self.compiled[axis]
        offset, first = len(self.original_rows), len(self.actual_rows)
        self.original_rows.extend(q.copy() for q in compiled['dense'])
        self.actual_rows.extend(q.copy() for q in compiled['rows'])
        self.indices.extend(int(i)+offset for i in compiled['indices'])
        self.labels.extend(compiled['labels'])
        self.legs.extend(axis if p.startswith('trial_'+axis+'_') else 'entry'
                         for p in compiled['labels'])
        self.active = dict(axis=axis, start_index=first, end_index=len(self.actual_rows),
            source_candidate_sha256=self.plans[axis].manifest['candidate_sha256'])
        self.report.setdefault('requested_commands', []).append(self.active.copy())
        self.report.update(planned_goals=len(self.actual_rows), holding=False, keyboard_ready=False,
                           selected_source_indices=list(self.indices))
        self.ready_for_input, self.confirmed = False, 0
        self.previous_hold_state = None

    def phase(self, index, moving):
        self.index, self.moving = index, moving
        if moving:
            self.reference.active_leg = self.legs[index]
            self.report.update(holding=False, keyboard_ready=False)
            return self.labels[index]
        self.reference.active_leg = 'entry'
        self._confirm_hold()
        if self.pending is not None and self.confirmed >= 3:
            self._append_pending()
            # run_pose computed moving=False before this append. This cycle
            # still checks G; the first appended target starts next cycle.
            return 'keyboard_next_command_G_boundary'
        return G_HOLD

    def _limit(self):
        if (self.moving and self.legs[self.index] in ('horizontal', 'vertical')
                and self.labels[self.index] not in SLOW_SEATING_PHASES):
            return 6
        return 2

    @contextmanager
    def runtime_scope(self):
        with self.reference.runtime_limits(), super().runtime_scope():
            yield

    def metadata(self):
        return dict(workflow='keyboard_A_horizontal_B_vertical_return_G',
            axis_candidate_sha256={k:p.manifest['candidate_sha256'] for k,p in self.plans.items()},
            keyboard_mapping=dict(a='horizontal', b='vertical'),
            completed_commands=[], requested_commands=[], maximum_ON_seconds=None,
            elapsed_time_limit_enabled=False, intermediate_OFF_included=False,
            automatic_stage_transitions_enabled=True, maximum_step_ticks=6,
            entry_maximum_step_ticks=2, period_seconds=.075, phase_gate_indices={},
            communication_pause_maximum_seconds=10., input_requires_three_checked_G_samples=True,
            physical_task_success_verified=False)

    @staticmethod
    def period(index, moving):
        return .075

    @staticmethod
    def ready(index):
        return True

    def message(self, phase):
        if phase == G_HOLD:
            return 'G 유지 상태를 확인 중입니다. 확인 후 A/B 입력을 받습니다. SPACE/Q/Esc: 종료/OFF.'
        if phase == 'keyboard_next_command_G_boundary':
            return ('가로' if self.active['axis']=='horizontal' else '세로')+' 회전 요청 수락. G에서 자동 시작합니다.'
        if phase == 'keyboard_initial_G_check':
            return 'G 준비 끝. 상태 확인 후 A:가로 / B:세로.'
        axis = 'horizontal' if phase.startswith('trial_horizontal_') else 'vertical'
        if phase == 'trial_final_G_check':
            return '회전 경로 끝. G 복귀를 확인한 뒤 다음 A/B 입력을 받습니다.'
        return self.plans[axis].message(phase)


class KeyboardTerminal:
    def __init__(self, sequence, stop):
        self.sequence, self.stop = sequence, stop

    def handle(self, key):
        if key in (' ', 'q', 'Q', '\x1b', '\x03'):
            raise pose.OperatorStop()
        if key.lower() in ('a', 'b'):
            return self.sequence.request_axis('horizontal' if key.lower()=='a' else 'vertical')
        return False

    def wait(self):
        import msvcrt
        while True:
            self.stop.check()
            if msvcrt.kbhit():
                key = msvcrt.getwch()
                if key == ' ':
                    return
                self.handle(key)
            time.sleep(.02)

    def check(self):
        import msvcrt
        self.stop.check()
        while msvcrt.kbhit():
            self.handle(msvcrt.getwch())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    report = dict(status='started', holding=False, all_torque_off_verified=False, settings_restored=False)
    try:
        resume_entry.validate_sources()
        pose.validate_sources()
        sequence = KeyboardRotationSequence(report)
        preview = sequence.rows(sequence.start)
        if not sequence.authorized():
            raise RuntimeError('Current rotation experiment authorization missing')
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print('NOT READY:', exc, 'No devices opened.')
        return 2
    if not args.execute:
        print(json.dumps(dict(status='device_free_check_passed', devices_opened=False,
            initial_G_goals=len(preview), **sequence.metadata()), ensure_ascii=False))
        return 0
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    folder = ROOT/('keyboard_rotate_grip_trial_'+dt.datetime.now(
        dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    stop = recovery_entry.StopRequest()
    terminal = KeyboardTerminal(sequence, stop)
    bus = live = None
    previous_sample = calibrated.sample
    try:
        with SingleInstance(ROOT/'.final_grip_pose.lock'):
            stop.install()
            print('한 창에서 A:가로 회전 / B:세로 회전 → 같은 G 복귀/유지.', flush=True)
            print('현재 실험 속도와 최근 가로 수정(중지4도 낮춤·끝마디 추가 굽힘)을 사용합니다.', flush=True)
            print('손을 떼고 SPACE로 G 준비. G 확인 안내 뒤 A/B를 한 번 누르세요.', flush=True)
            print('회전 중 A/B는 무시하며 예약하지 않습니다. 중간 확인 SPACE·토크OFF·500초 제한 없음.', flush=True)
            print('진행/유지 중 SPACE/Q/Esc: 중단/OFF. 종료 전 큐브를 받쳐주세요.', flush=True)
            terminal.wait()
            folder.mkdir()
            live = LiveState(folder, report)
            bus = PauseBus(report, terminal.check)
            sequence.bus = bus
            with sequence.runtime_scope():
                calibrated.sample = sample
                initial, start, _ = pose.preflight(bus, report, sequence.start)
                pose.run_pose(bus, initial, start, sequence.rows(start), report, terminal.check,
                              sequence=sequence, limit=float('inf'))
    except (pose.OperatorStop, KeyboardInterrupt):
        report['status'] = 'operator_finished' if sequence.ready_for_input else 'operator_stopped'
    except BaseException as exc:
        report.update(status='stopped', error=str(exc))
        print('STOP:', exc, flush=True)
    finally:
        calibrated.sample = previous_sample
        try:
            if bus is not None:
                bus.close()
        except BaseException as exc:
            report['port_close_error'] = str(exc)
        finally:
            stop.close()
        if live is not None:
            report['live_state_writer'] = live.close()
        report['serial_port_closed'] = 'port_close_error' not in report
        if folder.exists():
            (folder/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            print('Saved:', folder, flush=True)
    return 0 if (report['status'] in ('operator_finished', 'operator_stopped')
                 and not report.get('cleanup_error') and not report.get('port_close_error')) else 1


if __name__ == '__main__':
    raise SystemExit(main())
