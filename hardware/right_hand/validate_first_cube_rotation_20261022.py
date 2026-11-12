"""Independent offline replay; no ports/camera or hardware deployment.

Checks cube orientation by a projected face axis, including no-motion controls.
Synthetic cube placement and native servo approximations remain limitations.
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import time
import mujoco
import numpy as np
from mujoco_playground import registry
from scipy.spatial.transform import Rotation
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261022'
SOURCE = ROOT / 'cube_grasp_trial_20261022_141534_666300/result.json'


def projected_yaw(a, b, up):
    ra = Rotation.from_quat(np.asarray(a)[[1, 2, 3, 0]]).as_matrix()
    rb = Rotation.from_quat(np.asarray(b)[[1, 2, 3, 0]]).as_matrix()
    axis = int(np.argmin(np.abs(up @ ra)))
    va, vb = ra[:, axis].copy(), rb[:, axis].copy()
    va -= up * (up @ va)
    vb -= up * (up @ vb)
    va /= np.linalg.norm(va)
    vb /= np.linalg.norm(vb)
    return float(np.degrees(np.arctan2(up @ np.cross(va, vb), va @ vb)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('candidate')
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--quick', action='store_true')
    args = p.parse_args()
    source_path = OUT / args.candidate
    raw = source_path.read_bytes()
    candidate = json.loads(raw)
    assert candidate['source_sha256'] == hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    base = np.asarray(candidate['base_goal_ticks'], dtype=int)
    goals = np.asarray(candidate['candidate_goal_ticks'])
    assert goals.ndim == 2 and goals.shape[1] == 16
    assert np.array_equal(goals, np.rint(goals))
    goals = goals.astype(int)
    assert np.max(np.abs(np.diff(np.vstack([base, goals]), axis=0))) <= 2
    assert np.all(goals[:, 12:] == base[12:])
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    template, manifest = build_with_middle_thumb_hulls(env)
    target = float(candidate['requested_signed_yaw_deg'])
    gain0 = template.actuator_gainprm.copy()
    bias0 = template.actuator_biasprm.copy()
    hand = {template.body('palm').id}
    for body in range(template.nbody):
        if int(template.body_parentid[body]) in hand:
            hand.add(body)
    support = {template.geom(n).id for n in ('support_glass', 'support_wood')}
    cube = template.geom('cube').id
    tips = {template.geom(n).id for n in ('if_tip', 'mf_tip', 'rf_tip', 'th_tip')}
    up = -np.array(template.opt.gravity)
    up /= np.linalg.norm(up)
    qgoals = np.asarray([ticks_to_sim(q) for q in goals])
    joints = template.actuator_trnid[:, 0]
    assert np.all(qgoals >= template.jnt_range[joints, 0] - 1e-12)
    assert np.all(qgoals <= template.jnt_range[joints, 1] + 1e-12)
    assert np.all(qgoals >= template.actuator_ctrlrange[:, 0] - 1e-12)
    assert np.all(qgoals <= template.actuator_ctrlrange[:, 1] + 1e-12)
    ranges0 = template.jnt_range.copy()
    controls0 = template.actuator_ctrlrange.copy()
    period = float(candidate['goal_period_s'])
    hold = max(20, int(candidate['hold_samples']))

    def simulate(job):
        index, gain, friction, mass, control = job
        m = copy.copy(template)
        set_cube_physics(m, side_m=.055, mass_kg=mass)
        m.actuator_gainprm[:] = gain0
        m.actuator_biasprm[:] = bias0
        m.actuator_gainprm[:, 0] *= gain
        m.actuator_biasprm[:, 1] *= gain
        # Vary cube friction, preserving all collision definitions and ranges.
        m.geom_friction[cube] *= friction
        d = mujoco.MjData(m)
        d.qpos[:] = candidate['validation'][index]['initial_qpos']
        d.ctrl[:] = ticks_to_sim(base)
        mujoco.mj_forward(m, d)
        # Use identical zero-velocity starts for candidate and its control.
        origin = d.qpos[-7:].copy()
        path = np.repeat(ticks_to_sim(base)[None, :], len(goals), axis=0) if control else qgoals
        peaks = np.zeros(3)
        tracking = shift = drop = 0.
        cube_on_support = False
        fault = None
        longest = stable = 0
        trace = []
        contacts = []
        for k, goal in enumerate(list(path) + [path[-1]] * hold):
            d.ctrl[:] = goal
            active = False
            for _ in range(round(period / m.opt.timestep)):
                mujoco.mj_step(m, d)
                contacts = []
                active = False
                for c in d.contact:
                    a, b = int(c.geom1), int(c.geom2)
                    depth = max(0., -float(c.dist) * 1000)
                    if cube in (a, b):
                        other = b if a == cube else a
                        contacts.append(m.geom(other).name)
                        peaks[2] = max(peaks[2], depth)
                        cube_on_support |= other in support
                        active |= other in tips
                    elif a in support or b in support:
                        peaks[1] = max(peaks[1], depth)
                    elif int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
                        peaks[0] = max(peaks[0], depth)
                tracking = max(tracking, float(np.max(np.abs(d.qpos[:16] - goal))))
                shift = max(shift, float(np.linalg.norm(d.qpos[-7:-4] - origin[:3]) * 1000))
                drop = max(drop, float((origin[:3] - d.qpos[-7:-4]) @ up * 1000))
            finite = np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
            yaw = projected_yaw(origin[3:], d.qpos[-4:], up) if finite else 0.
            a = Rotation.from_quat(origin[3:][[1, 2, 3, 0]])
            b = Rotation.from_quat(d.qpos[-4:][[1, 2, 3, 0]])
            rv = (b * a.inv()).as_rotvec() * 180 / np.pi
            off = float(np.linalg.norm(rv - (rv @ up) * up))
            if not finite: fault = 'nonfinite'
            elif cube_on_support: fault = 'cube_on_external_support'
            elif peaks[0] > 1e-6: fault = 'self_contact'
            elif peaks[1] > .15: fault = 'hand_support_contact'
            elif peaks[2] > 1.5: fault = 'deep_cube_contact'
            elif tracking > .12: fault = 'tracking'
            elif shift > 20 or drop > 8: fault = 'cube_left_start_region'
            qualifies = not fault and abs(yaw - target) <= 4 and off <= 8 and shift <= 12 and drop <= 4 and active
            stable = stable + 1 if k >= len(goals) and qualifies else 0
            longest = max(longest, stable)
            if index == 0 and gain == friction == 1 and mass == .1 and not control and k % 2 == 0:
                trace.append({'time_s': (k + 1) * period, 'qpos': d.qpos.tolist()})
            if fault: break
        assert np.array_equal(ranges0, m.jnt_range)
        assert np.array_equal(controls0, m.actuator_ctrlrange)
        return dict(reset=index, gain=gain, cube_friction_scale=friction, mass_kg=mass,
                    control=control, fault=fault, projected_yaw_deg=yaw, off_axis_deg=off,
                    max_tracking_rad=tracking, max_shift_mm=shift, drop_mm=drop,
                    max_self_mm=peaks[0], max_hand_support_mm=peaks[1], max_cube_mm=peaks[2],
                    cube_on_external_support=cube_on_support, final_contacts=sorted(set(contacts)),
                    qualified_hold_seconds=longest * period, trace=trace,
                    success=bool(not fault and longest * period >= .5 and qualifies))

    variations = [(1., 1., .1)] if args.quick else [
        (gain, friction, .1) for gain in (.5, 1., 1.5) for friction in (.7, 1., 1.3)
    ] + [(1., 1., mass) for mass in (.08, .12)]
    jobs = [(i, *v, control) for v in variations for i in range(len(candidate['validation']))
            for control in (False, True)]
    began = time.monotonic()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(simulate, jobs))
    controls = {(r['reset'], r['gain'], r['cube_friction_scale'], r['mass_kg']): r
                for r in rows if r['control']}
    motion = [r for r in rows if not r['control']]
    for row in motion:
        key = (row['reset'], row['gain'], row['cube_friction_scale'], row['mass_kg'])
        baseline = controls[key]['projected_yaw_deg']
        delta = row['projected_yaw_deg'] - baseline
        row['no_motion_yaw_deg'] = baseline
        row['yaw_beyond_control_deg'] = delta
        row['success'] &= np.sign(delta) == np.sign(target) and abs(delta) >= 5
    passed = all(r['success'] for r in motion)
    clearance_passed = all(r['fault'] is None for r in motion)
    result = dict(status='independent_offline_rotation_replay', passed=passed,
                  clearance_tracking_screen_passed=clearance_passed,
                  requested_rotation_passed=passed,
                  successful_cases=sum(r['success'] for r in motion), total_cases=len(motion),
                  elapsed_seconds=time.monotonic() - began,
                  candidate_sha256=hashlib.sha256(raw).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  source_sha256=candidate['source_sha256'], model_manifest=manifest,
                  native_ranges_unchanged=True, real_cube_registration_verified=False,
                  hardware_execution_allowed=False, hardware_commands_sent=0,
                  algorithm='CEM_not_SAC', live_sac=False, trials=rows)
    target_path = source_path.with_name(source_path.stem + '_independent.json')
    target_path.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('trials', 'model_manifest')}, indent=2))
    print('Saved:', target_path)
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(main())
