"""Continue the exact executed stage-2 command lineage, without device access.

Saved simulation states are not reconstructions of the real cube state. Native
guards and all failures are retained; no new motion file is authorized here.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
WORK = ROOT / 'right_hand_rotation_20261109/prehardware_policy_cycle'
sys.path.insert(0, str(WORK))
import mujoco
from native_cycle import assets, restore, Guard, BOUND, motor, ticks_to_sim, save_json, sha, angle
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate
from work_grip_stage2_plan_20261112 import validate_candidate
from screen_direct_work_grip_20261112 import transition

BASE = WORK / 'next_cycle/handoff_refinement_20261110'
OUT = ROOT / 'right_hand_rotation_20261112/stage3_preparation'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--tag', required=True)
    p.add_argument('--gain', type=float, default=1., choices=(.5, 1., 1.5))
    p.add_argument('--mu', type=float, default=.3, choices=(.2, .3, .5))
    p.add_argument('--cases', nargs='+', default=['reached_tail', 'aligned_second', 'repeat_first', 'reverse_first'])
    p.add_argument('--hold', type=int, default=54)
    a = p.parse_args()
    if not a.tag.replace('_', '').isalnum():
        raise ValueError('Local screen tag required')
    folder = OUT / a.tag
    if folder.exists():
        raise RuntimeError('Preserve previous stage-3 screen')
    folder.mkdir(parents=True)
    (folder / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    source = ROOT / f'right_hand_rotation_20261112/stage2_work_grip/loaded_models/gain{a.gain:g}/mu{a.mu:g}_end_state.npz'
    model, initial, _, _, provenance = assets('active_ccd50')
    model.actuator_gainprm[:, 0] *= a.gain
    model.actuator_biasprm[:, 1] *= a.gain
    model.geom_friction[:, 0] = a.mu
    z = dict(np.load(source, allow_pickle=False)); d0 = restore(model, z)
    old = validate_candidate(); start = old['goals'][-1]
    assert np.array_equal(motor(d0.ctrl), start)
    q = np.asarray(np.load(BASE / 'initial_second_turn_hold_replay/research_goals.npz', allow_pickle=False)['goals'], dtype=int)
    first = dict(np.load(ROOT / 'right_hand_rotation_20261104/forward_hold/candidate.npz', allow_pickle=False))
    first_move = first['goals'][np.asarray(first['phases']) == 'stage_7_horizontal_yaw']
    target180 = upright_yaw_target(initial['initial_cube_rotation'], initial['up'], 180.)
    cube = model.geom('cube').id
    cases = {
        'reached_tail': q[3015:-54],
        'aligned_second': np.vstack([transition(start, q[3404], 'all'), q[3405:-54]]),
        'repeat_first': np.vstack([transition(start, first_move[0], 'TH_MF_RF_IF'), first_move[1:],
                                  transition(first['goals'][-1], start, 'TH_MF_RF_IF')]),
        'repeat_first_all': np.vstack([transition(start, first_move[0], 'all'), first_move[1:],
                                      transition(first['goals'][-1], start, 'all')]),
        'repeat_first_all_then_observed': np.vstack([transition(start, first_move[0], 'all'), first_move[1:],
                                      transition(first['goals'][-1], start, 'TH_MF_RF_IF')]),
        'repeat_first_observed_then_all': np.vstack([transition(start, first_move[0], 'TH_MF_RF_IF'), first_move[1:],
                                      transition(first['goals'][-1], start, 'all')]),
        'reverse_first': np.vstack([transition(start, first['goals'][-1], 'TH_MF_RF_IF'), first_move[-2::-1],
                                   transition(first_move[0], start, 'TH_MF_RF_IF')]),
    }
    rows = []
    for name in a.cases:
        moving = cases[name]
        goals = np.vstack([moving, np.tile(moving[-1], (a.hold, 1))])
        assert np.all(np.isfinite(goals)) and np.max(abs(np.diff(np.vstack([start, goals]), axis=0))) <= 2
        data = restore(model, z); guard = Guard(model)
        if np.any(goals < guard.lo) or np.any(goals > guard.hi):
            raise ValueError('Unchanged native goals violated')
        Bbegin = data.geom_xmat[cube].reshape(3, 3).copy()
        cert = SweepCertificate(model, guard.hand, .3)
        stop = []; samples = supported = clear = 0; minimum = None; shift = drift = 0.; C0 = B0 = None
        done = 0; checkpoints = []
        for i, command in enumerate(goals):
            data.ctrl[:] = ticks_to_sim(command)
            for ms in range(75):
                mujoco.mj_step(model, data); stop = guard.check(data)
                if np.linalg.norm(data.qpos[-7:-4] - initial['initial_cube_center']) > .08:
                    stop = list(stop) + ['cube_translation_over80mm']
                if i >= len(moving):
                    C, B = data.geom_xpos[cube], data.geom_xmat[cube].reshape(3, 3)
                    if C0 is None: C0, B0 = C.copy(), B.copy()
                    samples += 1; supported += sample_model(model, data, guard.hand, cube)['supported_sample']
                    lower, bad = cert.query(data, C, B)
                    minimum = lower if minimum is None else min(minimum, lower)
                    clear += not bad and lower > .3
                    shift = max(shift, float(np.linalg.norm(C - C0) * 1000))
                    drift = max(drift, orientation_error_deg(B, B0))
                if stop: break
            done = i + 1
            if done % 250 == 0 or stop or done == len(goals):
                B = data.geom_xmat[cube].reshape(3, 3)
                t = {'case': name, 'done': done, 'stop': stop, 'relative_yaw_deg': angle(B, Bbegin, initial['up'])}
                checkpoints.append(t); print(json.dumps(t), flush=True)
            if stop: break
        sub = folder / name; sub.mkdir()
        np.savez_compressed(sub / 'research_goals.npz', start=start, goals=goals[:done])
        np.savez_compressed(sub / 'end_state.npz', **{k: getattr(data, k) for k in BOUND}, time=data.time)
        B = data.geom_xmat[cube].reshape(3, 3)
        supported_hold = bool(not stop and samples == a.hold * 75 and supported / max(1, samples) > .8 and shift < 2 and drift < 3)
        row = {'case': name, 'goals': done, 'planned_goals': len(goals), 'moving_goals': len(moving), 'stop': stop,
               'native_safety_passed': not stop and done == len(goals), 'relative_yaw_deg': angle(B, Bbegin, initial['up']),
               'relative_orientation_change_deg': orientation_error_deg(B, Bbegin),
               'absolute_target180_error_deg': orientation_error_deg(B, target180),
               'tilt_deg': face_up_error_deg(B, 5, initial['up']), 'hold_samples': samples, 'supported_samples': supported,
               'upper_certified_samples': clear, 'minimum_upper_lower_mm': minimum, 'hold_shift_mm': shift,
               'hold_drift_deg': drift, 'stable_supported_hold': supported_hold,
               'supported_clear_hold': supported_hold and clear == samples and minimum > .3,
               'worst': guard.worst, 'terminal_guard': guard.last, 'checkpoints': checkpoints}
        save_json(sub / 'review.json', row); rows.append(row)
        save_json(folder / 'progress.json', {'rows': rows})
        print(json.dumps({k: v for k, v in row.items() if k not in ('worst', 'terminal_guard', 'checkpoints')}), flush=True)
    save_json(folder / 'review.json', {'status': 'exact_stage2_model_continuation_screen', 'rows': rows,
              'source_state': source.relative_to(ROOT).as_posix(), 'source_state_sha256': sha(source),
              'model_provenance': provenance, 'gain': a.gain, 'friction_hypothesis': a.mu,
              'every_dynamic_sample_checked_ms': 1, 'physical_dynamic_state_reconstructed': False,
              'native_guards_and_limits_unchanged': True, 'devices_opened': 0, 'hardware_execution_allowed': False})


if __name__ == '__main__':
    main()
