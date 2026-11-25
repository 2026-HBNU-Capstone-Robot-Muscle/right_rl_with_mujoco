"""Offline ring-finger third-contact proposals, followed by free-cube replay.

Pinned poses are used for geometric proposals only. No physical device access.
Native ranges and existing dynamic guards are preserved.
"""
from pathlib import Path
import argparse
import hashlib
import json
import time

import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.stats import qmc

from research_rotation_body_hold_20261104 import restore, inspect_for, native, motor
from hand_motor_mapping import SIM_TO_MOTOR_IDS, TICK
from upper_layer_cylinder_clearance_20261105 import precompute, gaps
from canonical_grip_dynamics_20261105 import run
from research_probe_index_handoff_20261109 import ramp

ROOT = Path(__file__).resolve().parent
MODEL = ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
STATES = {
    'before_transfer': ROOT / 'right_hand_rotation_20261109/middle_probe_reconstruction_linux/checkpoint_1042.npz',
    'after_transfer': ROOT / 'right_hand_rotation_20261109/index_thumb_transfer/cp1042_together/end.npz',
}
OUT = ROOT / 'right_hand_rotation_20261109/ring_third_support_v2'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def search(label, samples_power, max_dynamic):
    began = time.perf_counter()
    folder = OUT / label
    folder.mkdir(parents=True, exist_ok=True)
    m = mujoco.MjModel.from_binary_path('source.mjb', assets={'source.mjb': MODEL.read_bytes()})
    z = np.load(STATES[label], allow_pickle=False)
    d = restore(m, z)
    base = d.qpos.copy()
    pd = d.ctrl - base[:16]
    cube = m.geom('cube').id
    C = d.geom_xpos[cube].copy()
    B = d.geom_xmat[cube].reshape(3, 3).copy()
    hand, _, check = inspect_for(m)
    ring_cache = [row for row in precompute(m, hand)
                  if m.body(int(m.geom_bodyid[row[0]])).name.startswith('rf_')]
    contact_names = ('rf_tip', 'rf_ds_collision_1', 'rf_md_collision_1', 'rf_px_collision')
    contact_geoms = [m.geom(name).id for name in contact_names]
    lo, hi, _, _ = native(m)
    ids = np.asarray(SIM_TO_MOTOR_IDS) - 1
    active = np.arange(8, 12)
    low = (np.ceil(lo[ids]) + 4 - 2048) * TICK - pd
    high = (np.floor(hi[ids]) - 4 - 2048) * TICK - pd
    reference = base[active].copy()
    initial_ctrl = motor(d.ctrl)

    def pose(x):
        d.qpos[:] = base
        d.qpos[active] = x
        d.ctrl[:] = d.qpos[:16] + pd
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)

    def contact(g):
        points = np.zeros(6)
        raw = float(mujoco.mj_geomDistance(m, d, cube, g, 1., points)) * 1000
        witness = float(np.linalg.norm(points[3:] - points[:3])) * 1000
        contacts = [c for c in d.contact if {int(c.geom1), int(c.geom2)} == {cube, g}]
        if contacts:
            actual = min(contacts, key=lambda c: c.dist)
            distance = float(actual.dist) * 1000
            point = actual.pos.copy()
        else:
            # A zero query with separated witness points cannot establish contact.
            # Its witness length is only a search proxy, never proof of touching.
            distance = max(raw, witness)
            point = (points[:3] + points[3:]) / 2
        height = -float((point - C) @ B[:, 1]) * 1000
        return distance, height, point

    proposals = []
    seeds = qmc.scale(qmc.Sobol(4, scramble=True, seed=20261109).random_base2(samples_power), low[active], high[active])
    seeds = np.vstack([reference, seeds])
    for i, x in enumerate(seeds):
        pose(x)
        support, own = check(d)
        self_mm = max(own.values(), default=0.)
        support_mm = max(support.values(), default=0.)
        for g in contact_geoms:
            distance, height, _ = contact(g)
            score = abs(distance + .05) + max(0., height - 6.) * 3 + self_mm * 150 + support_mm * 150
            proposals.append((score, g, x.copy(), distance, height, self_mm, support_mm))
        if i and i % 1024 == 0:
            print(json.dumps({'state': label, 'sampled': i, 'best_score': min(v[0] for v in proposals)}), flush=True)
    proposals.sort(key=lambda row: row[0])
    selected = []
    for row in proposals:
        if all(row[1] != previous[1] or np.linalg.norm(row[2] - previous[2]) > .2 for previous in selected):
            selected.append(row)
        if len(selected) == 24:
            break
    fitted = []
    for i, row in enumerate(selected):
        _, g, seed, *_ = row

        def residual(x):
            pose(x)
            distance, height, _ = contact(g)
            support, own = check(d)
            upper_gap = min(value for _, value in gaps(m, d, C, B, ring_cache, True))
            return np.r_[
                (distance + .05) * 4,
                max(0., height - 6.) * 6,
                max(own.values(), default=0.) * 2000,
                max(support.values(), default=0.) * 2000,
                max(0., .5 - upper_gap) * 8,
                (x - reference) * .01,
            ]

        fit = least_squares(residual, seed, bounds=(low[active], high[active]),
                            max_nfev=150, diff_step=1e-5, ftol=1e-7, xtol=1e-7, gtol=1e-7)
        values = residual(fit.x)
        distance, height, point = contact(g)
        support, own = check(d)
        gap = min(value for _, value in gaps(m, d, C, B, ring_cache, True))
        goal = motor(d.ctrl)
        observed_contacts = [c for c in d.contact if {int(c.geom1), int(c.geom2)} == {cube, g}]
        accepted = bool(-.2 <= distance <= .5 and height <= 6.2 and
                        observed_contacts and not own and max(support.values(), default=0.) <= .15 and gap > .3)
        item = {'fit_index': i, 'contact_geom': m.geom(g).name,
                'ring_qpos': fit.x.tolist(), 'goal': goal.tolist(),
                'distance_mm': distance, 'contact_height_mm': height,
                'contact_world_m': point.tolist(), 'ring_upper_gap_mm': gap,
                'self_penetration_mm': max(own.values(), default=0.),
                'support_penetration_mm': max(support.values(), default=0.),
                'actual_pinned_contact_count': len(observed_contacts),
                'geometry_accepted': accepted, 'residual_norm': float(np.linalg.norm(values))}
        fitted.append(item)
        print(json.dumps({'state': label, 'fit': i, 'geom': item['contact_geom'], 'distance': distance,
                          'height': height, 'self_mm': item['self_penetration_mm'],
                          'gap': gap, 'geometry_accepted': accepted}), flush=True)
    dynamic_cases = []
    distinct = []
    for item in sorted((r for r in fitted if r['geometry_accepted']), key=lambda r: r['residual_norm']):
        goal = np.asarray(item['goal'], int)
        if any(np.max(np.abs(goal - old)) < 8 for old in distinct):
            continue
        distinct.append(goal)
        case_folder = folder / ('fit_' + str(item['fit_index']))
        case_folder.mkdir(exist_ok=True)
        rows = np.vstack([ramp(initial_ctrl, goal), np.tile(goal, (134, 1))])
        np.savez_compressed(case_folder / 'research_path.npz', start=initial_ctrl, goals=rows)
        metric = run(m, z, rows, 1, case_folder / 'end.npz', case_folder / 'trajectory.npz', False,
                     required_lower_contacts=('if_tip', 'th_tip'))
        dynamic_cases.append({'fit_index': item['fit_index'], 'metric': metric})
        (case_folder / 'review.json').write_text(json.dumps(dynamic_cases[-1], indent=2) + '\n', encoding='utf-8')
        print(json.dumps({'state': label, 'dynamic_fit': item['fit_index'], 'safety': metric['safety_passed'],
                          'tilt': metric['final_up_tilt_deg'], 'task': metric['whole_grip_task_passed']}), flush=True)
        if len(dynamic_cases) >= max_dynamic:
            break
    result = {
        'status': 'ring_third_contact_offline_research', 'source_model_sha256': sha(MODEL),
        'source_state_sha256': sha(STATES[label]), 'pinned_geometry_only_for_proposals': True,
        'zero_distance_with_separated_witnesses_not_accepted_as_contact': True,
        'geometry_admission_requires_actual_pinned_contact': True,
        'sampled_ring_configurations': len(seeds), 'fits': fitted, 'dynamic_cases': dynamic_cases,
        'geometry_accepted': sum(r['geometry_accepted'] for r in fitted),
        'coarse_task_passes': sum(r['metric']['whole_grip_task_passed'] for r in dynamic_cases),
        'ring_force_support_certified': False, 'full_1ms_sweep_certified': False,
        'devices_opened': 0, 'hardware_execution_allowed': False,
        'elapsed_seconds': time.perf_counter() - began,
    }
    (folder / 'review.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', choices=STATES, default='after_transfer')
    parser.add_argument('--samples-power', type=int, default=12)
    parser.add_argument('--max-dynamic', type=int, default=3)
    args = parser.parse_args()
    result = search(args.state, args.samples_power, args.max_dynamic)
    print(json.dumps({k: result[k] for k in ('sampled_ring_configurations', 'geometry_accepted', 'coarse_task_passes', 'elapsed_seconds')}), flush=True)


if __name__ == '__main__':
    main()
