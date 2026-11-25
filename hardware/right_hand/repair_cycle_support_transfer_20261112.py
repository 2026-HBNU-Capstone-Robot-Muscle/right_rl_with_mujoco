"""Offline investigation of the observed first-transfer fall; no device APIs.

Keep the executed candidate immutable. Saved simulation is a hypothesis, not a
reconstruction of the physical cube. Native collision/range/transport guards stay.
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
from native_cycle import assets, restore, Guard, BOUND, ticks_to_sim, motor, save_json, sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target, orientation_error_deg, face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate

SOURCE = ROOT / 'right_hand_rotation_20261112/nominal_cycle_trial_preparation/rf16_2177_v2/candidate.npz'
OUT = ROOT / 'right_hand_rotation_20261112/nominal_cycle_actual_review/support_transfer_fix_v3'


def snapshot(d):
    return {**{k: getattr(d, k).copy() for k in BOUND}, 'time': d.time}


def inspect():
    out = OUT / 'baseline'; out.mkdir(parents=True, exist_ok=False)
    z = dict(np.load(SOURCE, allow_pickle=False)); q = z['goals']
    m, initial, _, _, provenance = assets('active_ccd50'); d = restore(m, initial); g = Guard(m)
    checkpoints = {1417, 1700, 1800, 1900, 1950, 2000, 2050, 2100, 2150, 2188, 2202, 2250, 2300, 2400, 2600, 2800, 3015, 3351, 3405}
    rows = []
    for i, command in enumerate(q[:3405], 1):
        d.ctrl[:] = ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(m, d); stop = g.check(d)
            if stop: break
        if i in checkpoints or stop:
            support = sample_model(m, d, g.hand, m.geom('cube').id)
            row = {'goal': i, 'stop': stop, 'goal_ticks': command.tolist(),
                   'model_actual_ticks': np.round(motor(d.qpos[:16]), 2).tolist(),
                   'cube_center': d.qpos[-7:-4].tolist(), 'support': support}
            rows.append(row)
            np.savez_compressed(out / f'state_{i}.npz', **snapshot(d))
            print(json.dumps({'goal': i, 'stop': stop, 'contact': support['loaded_hand_geoms'],
                              'support_N': round(support['upward_support_N'], 3),
                              'actual_ticks': row['model_actual_ticks']}), flush=True)
        if stop: break
    save_json(out / 'review.json', {'rows': rows, 'source_sha256': sha(SOURCE),
              'model_provenance': provenance, 'worst': g.worst, 'devices_opened': 0,
              'physical_cube_reconstructed': False, 'native_guards_changed': False})


def delayed_thumb(delay):
    z = dict(np.load(SOURCE, allow_pickle=False)); old = z['goals']
    begin, end = 2050, 2600
    segment = old[begin - 1:end]
    indices = np.arange(len(segment) + delay)
    moving = segment[np.minimum(indices, len(segment) - 1)].copy()
    moving[:, :4] = segment[np.clip(indices - delay, 0, len(segment) - 1), :4]
    goals = np.vstack([old[:begin], moving, old[end:]])
    assert np.max(abs(np.diff(np.vstack([z['start'], goals]), axis=0))) <= 2
    return z, goals, len(goals) - len(old), begin


def search():
    folder = OUT / 'thumb_delay_screen'; folder.mkdir(exist_ok=False)
    m, initial, _, _, provenance = assets('active_ccd50')
    saved = dict(np.load(OUT / 'baseline/state_2050.npz', allow_pickle=False))
    rows = []
    for delay in (80, 160, 240):
        z, goals, extra, begin = delayed_thumb(delay)
        sub = folder / f'delay{delay}'; sub.mkdir()
        d = restore(m, saved); g = Guard(m); cube = m.geom('cube').id
        cert = SweepCertificate(m, g.hand, .3)
        holds = []; active = None; sparse = []
        for i in range(begin, len(goals)):
            hold_id = 0 if 3351 + extra <= i < 3405 + extra else 1 if i >= len(goals) - 54 else None
            if hold_id is not None and active != hold_id:
                active = hold_id; C0 = d.geom_xpos[cube].copy(); B0 = d.geom_xmat[cube].reshape(3, 3).copy()
                h = dict(samples=0, supported=0, clear=0, shift_mm=0., drift_deg=0., minimum_upper_mm=float('inf'))
                holds.append(h)
            d.ctrl[:] = ticks_to_sim(goals[i])
            for _ in range(75):
                mujoco.mj_step(m, d); stop = g.check(d)
                if np.linalg.norm(d.qpos[-7:-4] - initial['initial_cube_center']) > .08:
                    stop = list(stop) + ['cube_translation_over80mm']
                if hold_id is not None:
                    C, B = d.geom_xpos[cube], d.geom_xmat[cube].reshape(3, 3)
                    lower, bad = cert.query(d, C, B); h['samples'] += 1
                    h['supported'] += sample_model(m, d, g.hand, cube)['supported_sample']
                    h['clear'] += not bad and lower > .3
                    h['minimum_upper_mm'] = min(h['minimum_upper_mm'], lower)
                    h['shift_mm'] = max(h['shift_mm'], float(np.linalg.norm(C - C0) * 1000))
                    h['drift_deg'] = max(h['drift_deg'], orientation_error_deg(B, B0))
                if stop: break
            if hold_id is not None:
                B = d.geom_xmat[cube].reshape(3, 3)
                h['target_error_deg'] = orientation_error_deg(B, upright_yaw_target(initial['initial_cube_rotation'], initial['up'], 90 if hold_id == 0 else 180))
                h['tilt_deg'] = face_up_error_deg(B, 5, initial['up'])
            if (i + 1) % 200 == 0 or stop:
                sup = sample_model(m, d, g.hand, cube)
                row = {'goal': i + 1, 'stop': stop, 'support_N': sup['upward_support_N'], 'loaded': sup['loaded_hand_geoms']}
                sparse.append(row); print(json.dumps(dict(case=delay, **row)), flush=True)
            if stop: break
        passed = bool(not stop and i + 1 == len(goals) and len(holds) == 2 and all(
            h['samples'] == h['clear'] == 4050 and h['supported'] / 4050 > .8 and h['target_error_deg'] < 5
            and h['tilt_deg'] < 5 and h['shift_mm'] < 2 and h['drift_deg'] < 3 for h in holds))
        np.savez_compressed(sub / 'candidate.npz', start=z['start'], goals=goals)
        np.savez_compressed(sub / 'end_state.npz', **snapshot(d))
        row = dict(delay=delay, added_goals=extra, done=i + 1, total=len(goals), passed=passed, stop=stop,
                   holds=holds, worst=g.worst, terminal_guard=g.last, sparse=sparse,
                   prefix_goals_unchanged=begin, prefix_state_reset_is_cached_exact_model_state=True,
                   physical_cube_reconstructed=False, devices_opened=0, model_provenance=provenance,
                   source_sha256=sha(SOURCE), candidate_sha256=sha(sub / 'candidate.npz'))
        save_json(sub / 'review.json', row); rows.append(row)
        save_json(folder / 'review.json', {'rows': rows, 'guards_changed': False})
        print(json.dumps({k: row[k] for k in ('delay', 'done', 'total', 'passed', 'stop', 'holds')}), flush=True)


def compensated_index(offset):
    z = dict(np.load(SOURCE, allow_pickle=False)); old = z['goals']
    proposed = old.copy()
    ramp = np.clip((np.arange(len(old)) - 1799) / 200., 0., 1.)
    proposed[:, 7] += np.rint(offset * ramp).astype(int)
    rows = []; indices = []; previous = z['start']
    for i, target in enumerate(proposed):
        steps = max(1, int(np.ceil(np.max(abs(target - previous)) / 2)))
        rows.extend(np.rint(previous + (target - previous) * k / steps).astype(int) for k in range(1, steps + 1))
        indices.extend([i] * steps); previous = target
    goals = np.asarray(rows); indices = np.asarray(indices)
    assert np.max(abs(np.diff(np.vstack([z['start'], goals]), axis=0))) <= 2
    return z, goals, indices


def compensation_screen(offset, bias):
    folder = OUT / f'index_comp{offset}_bias{bias}'; folder.mkdir(exist_ok=False)
    z, goals, source_index = compensated_index(offset)
    m, initial, _, _, provenance = assets('active_ccd50')
    saved = dict(np.load(OUT / 'baseline/state_1800.npz', allow_pickle=False))
    d = restore(m, saved); g = Guard(m); cube = m.geom('cube').id
    cert = SweepCertificate(m, g.hand, .3); holds = []; active = None; sparse = []
    begin = int(np.flatnonzero(source_index >= 1800)[0]); failure = []
    for i in range(begin, len(goals)):
        old_index = int(source_index[i])
        # Hypothetical persistent servo error, derived from recorded-vs-model
        # ID8 difference. It is not a calibrated motor or reconstructed state.
        ramp = float(np.clip((old_index - 1799) / 200., 0., 1.))
        m.actuator_biasprm[0, 0] = -m.actuator_gainprm[0, 0] * (2 * np.pi / 4096) * bias * ramp
        hold_id = 0 if 3351 <= old_index < 3405 else 1 if old_index >= 5035 else None
        if hold_id is not None and active != hold_id:
            active = hold_id; C0 = d.geom_xpos[cube].copy(); B0 = d.geom_xmat[cube].reshape(3, 3).copy()
            h = dict(samples=0, supported=0, clear=0, shift_mm=0., drift_deg=0., minimum_upper_mm=float('inf'))
            holds.append(h)
        d.ctrl[:] = ticks_to_sim(goals[i])
        for _ in range(75):
            mujoco.mj_step(m, d); stop = g.check(d)
            if np.linalg.norm(d.qpos[-7:-4] - initial['initial_cube_center']) > .08:
                stop = list(stop) + ['cube_translation_over80mm']
            if hold_id is not None:
                C, B = d.geom_xpos[cube], d.geom_xmat[cube].reshape(3, 3)
                lower, bad = cert.query(d, C, B); h['samples'] += 1
                h['supported'] += sample_model(m, d, g.hand, cube)['supported_sample']
                h['clear'] += not bad and lower > .3; h['minimum_upper_mm'] = min(h['minimum_upper_mm'], lower)
                h['shift_mm'] = max(h['shift_mm'], float(np.linalg.norm(C - C0) * 1000))
                h['drift_deg'] = max(h['drift_deg'], orientation_error_deg(B, B0))
            if stop: break
        if hold_id is not None:
            B = d.geom_xmat[cube].reshape(3, 3)
            h['target_error_deg'] = orientation_error_deg(B, upright_yaw_target(initial['initial_cube_rotation'], initial['up'], 90 if hold_id == 0 else 180))
            h['tilt_deg'] = face_up_error_deg(B, 5, initial['up'])
        if (old_index + 1) % 300 == 0 or stop:
            row = {'original_goal': old_index + 1, 'stop': stop, 'support': sample_model(m, d, g.hand, cube)['loaded_hand_geoms']}
            sparse.append(row); print(json.dumps(dict(offset=offset, bias=bias, **row)), flush=True)
        if old_index == 3404 and (h['clear'] != 4050 or h['supported'] / 4050 <= .8 or h['target_error_deg'] >= 5 or h['tilt_deg'] >= 5):
            failure = ['first_hold_task_missed']; break
        if stop: break
    passed = bool(not stop and not failure and i + 1 == len(goals) and len(holds) == 2 and all(
        h['samples'] == h['clear'] == 4050 and h['supported'] / 4050 > .8 and h['target_error_deg'] < 5
        and h['tilt_deg'] < 5 and h['shift_mm'] < 2 and h['drift_deg'] < 3 for h in holds))
    np.savez_compressed(folder / 'candidate.npz', start=z['start'], goals=goals, source_indices=source_index)
    np.savez_compressed(folder / 'end_state.npz', **snapshot(d))
    row = dict(offset=offset, hypothetical_ID8_bias_ticks=bias, done=i + 1, total=len(goals), passed=passed,
               stop=stop, task_failure=failure, holds=holds, worst=g.worst, terminal_guard=g.last,
               sparse=sparse, prefix_goals_unchanged=1800, model_provenance=provenance,
               bias_is_hypothesis_not_motor_calibration=True, native_guards_changed=False,
               devices_opened=0, physical_cube_reconstructed=False, source_sha256=sha(SOURCE),
               candidate_sha256=sha(folder / 'candidate.npz'))
    save_json(folder / 'review.json', row)
    print(json.dumps({k: row[k] for k in ('offset', 'hypothetical_ID8_bias_ticks', 'done', 'total', 'passed', 'stop', 'task_failure', 'holds')}), flush=True)


def thumb_contact_ik(weight):
    from scipy.optimize import least_squares
    folder = OUT / f'thumb_contact_ik{weight:g}'; folder.mkdir(exist_ok=False)
    z = dict(np.load(SOURCE, allow_pickle=False)); goals = z['goals'].copy()
    m, initial, _, _, provenance = assets('active_ccd50')
    d = restore(m, dict(np.load(OUT / 'baseline/state_2000.npz', allow_pickle=False)))
    k = mujoco.MjData(m); g = Guard(m); cube = m.geom('cube').id; tip = m.geom('th_tip').id
    cert = SweepCertificate(m, g.hand, .3); holds = []; active = None; trace = []; failure = []
    previous = goals[1999].copy(); th = np.asarray([15, 14, 13, 12]); TICK = 2 * np.pi / 4096
    for i in range(2000, len(goals)):
        desired = goals[i].copy()
        command = previous + np.clip(desired - previous, -2, 2)
        if i < 2450:
            k.qpos[:] = d.qpos; k.qvel[:] = 0
            lag = d.qpos[:16] - d.ctrl
            k.qpos[:16] = ticks_to_sim(command) + lag
            base = k.qpos.copy()
            lower = np.maximum(g.lo[:4], previous[:4] - 2) - desired[:4]
            upper = np.minimum(g.hi[:4], previous[:4] + 2) - desired[:4]
            def residual(delta):
                k.qpos[:] = base; k.qpos[th] = ticks_to_sim(desired)[th] + lag[th] + delta * TICK
                mujoco.mj_forward(m, k)
                gap = mujoco.mj_geomDistance(m, k, tip, cube, .2, None) * 1000
                return np.r_[max(0., gap + .03) * weight, delta * .005]
            fit = least_squares(residual, np.clip(command[:4] - desired[:4], lower, upper),
                                bounds=(lower - 1e-9, upper + 1e-9), diff_step=.05, max_nfev=18)
            command[:4] = np.clip(np.rint(desired[:4] + fit.x), previous[:4] - 2, previous[:4] + 2).astype(int)
        assert np.max(abs(command - previous)) <= 2 and np.all(command >= g.lo) and np.all(command <= g.hi)
        goals[i] = command; previous = command.copy()
        hold_id = 0 if 3351 <= i < 3405 else 1 if i >= len(goals) - 54 else None
        if hold_id is not None and active != hold_id:
            active = hold_id; C0 = d.geom_xpos[cube].copy(); B0 = d.geom_xmat[cube].reshape(3, 3).copy()
            h = dict(samples=0, supported=0, clear=0, shift_mm=0., drift_deg=0., minimum_upper_mm=float('inf')); holds.append(h)
        d.ctrl[:] = ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(m, d); stop = g.check(d)
            if np.linalg.norm(d.qpos[-7:-4] - initial['initial_cube_center']) > .08:
                stop = list(stop) + ['cube_translation_over80mm']
            if hold_id is not None:
                C, B = d.geom_xpos[cube], d.geom_xmat[cube].reshape(3, 3)
                gap, bad = cert.query(d, C, B); h['samples'] += 1; h['clear'] += not bad and gap > .3
                h['supported'] += sample_model(m, d, g.hand, cube)['supported_sample']; h['minimum_upper_mm'] = min(h['minimum_upper_mm'], gap)
                h['shift_mm'] = max(h['shift_mm'], float(np.linalg.norm(C - C0) * 1000)); h['drift_deg'] = max(h['drift_deg'], orientation_error_deg(B, B0))
            if stop: break
        if hold_id is not None:
            B = d.geom_xmat[cube].reshape(3, 3)
            h['target_error_deg'] = orientation_error_deg(B, upright_yaw_target(initial['initial_cube_rotation'], initial['up'], 90 if hold_id == 0 else 180))
            h['tilt_deg'] = face_up_error_deg(B, 5, initial['up'])
        if (i + 1) % 100 == 0 or stop:
            sup = sample_model(m, d, g.hand, cube)
            row = {'goal': i + 1, 'stop': stop, 'thumb_change': (command[:4] - z['goals'][i, :4]).tolist(),
                   'support_N': sup['upward_support_N'], 'contacts': sup['loaded_hand_geoms']}
            trace.append(row); print(json.dumps(dict(weight=weight, **row)), flush=True)
        if i == 3404 and (h['clear'] != 4050 or h['supported'] / 4050 <= .8 or h['target_error_deg'] >= 5 or h['tilt_deg'] >= 5):
            failure = ['first_hold_task_missed']; break
        if stop: break
    passed = bool(not stop and not failure and i + 1 == len(goals) and len(holds) == 2 and all(
        h['samples'] == h['clear'] == 4050 and h['supported'] / 4050 > .8 and h['target_error_deg'] < 5
        and h['tilt_deg'] < 5 and h['shift_mm'] < 2 and h['drift_deg'] < 3 for h in holds))
    np.savez_compressed(folder / 'candidate.npz', start=z['start'], goals=goals)
    np.savez_compressed(folder / 'end_state.npz', **snapshot(d))
    row = dict(thumb_contact_weight=weight, done=i + 1, total=len(goals), passed=passed, stop=stop, task_failure=failure,
               holds=holds, worst=g.worst, terminal_guard=g.last, trace=trace, model_provenance=provenance,
               prefix_goals_unchanged=2000, native_guards_changed=False, source_sha256=sha(SOURCE),
               candidate_sha256=sha(folder / 'candidate.npz'), physical_cube_reconstructed=False, devices_opened=0)
    save_json(folder / 'review.json', row)
    print(json.dumps({k: row[k] for k in ('thumb_contact_weight', 'done', 'total', 'passed', 'stop', 'task_failure', 'holds')}), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--inspect', action='store_true'); p.add_argument('--search', action='store_true')
    p.add_argument('--index-comp', type=int); p.add_argument('--bias', type=int, default=0); p.add_argument('--thumb-ik', type=float); a = p.parse_args()
    if a.inspect: inspect()
    if a.search: search()
    if a.index_comp is not None: compensation_screen(a.index_comp, a.bias)
    if a.thumb_ik is not None: thumb_contact_ik(a.thumb_ik)
