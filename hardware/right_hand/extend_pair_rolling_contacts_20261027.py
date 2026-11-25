"""Offline rolling-contact continuation. Never imports or opens robot devices.

Contact points may move tangentially on the same two cube faces. The accepted
goals remain bounded by the existing motor ranges. A separate free dynamics
replay is required; prescribed object IK does not establish rotation.
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import argparse
import json
import time
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / 'right_hand_rotation_20261027/reached_pair_yaw_study_v2'
OLD = ROOT / 'right_hand_rotation_20261027/reached_pair_contact_yaw_extension'
OUT = ROOT / 'right_hand_rotation_20261027/reached_pair_rolling_contacts'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--with-lift', action='store_true')
    parser.add_argument('--allow-contact-reposition', action='store_true')
    args = parser.parse_args()
    output = OUT.with_name(OUT.name + '_with_lift') if args.with_lift else OUT
    if args.allow_contact_reposition:
        output = output.with_name(output.name + '_reposition')
    began = time.perf_counter()
    output.mkdir(exist_ok=True)
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    m, _ = build(registry.load('LeapCubeFaceUp', config=cfg), output / 'scene.xml')
    proof = json.loads((ROOT / 'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    correct_model(m, np.array(proof['start_ticks']))
    set_cube_physics(m, .055, .1)
    m.opt.timestep = .002
    up = np.array(json.loads((ROOT / 'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world'])
    m.opt.gravity[:] = -9.81 * up
    state = np.load(SOURCE / 'nominal_grasp_state.npz')
    d = mujoco.MjData(m)
    d.qpos[:] = state['qpos']
    d.ctrl[:] = state['ctrl']
    mujoco.mj_forward(m, d)
    C = d.qpos[-7:-4].copy()
    B = d.xmat[m.body('cube').id].reshape(3, 3).copy()
    cube = m.geom('cube').id
    tips = [m.geom(n + '_tip').id for n in ('th', 'mf')]
    old = np.load(OLD / 'continued.npz')
    base, bias = old['start'], old['bias_ticks']
    lo, hi = limits()
    hand = {m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:
            hand.add(b)
    support = {i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_', 'leap_mount_collision')) or m.geom(i).name == 'floor'}
    inspect = cached_contacts(m, hand, support)
    horizontal = np.cross([0., 0., 1.], up)
    horizontal /= np.linalg.norm(horizontal)
    basis = np.column_stack([horizontal, np.cross(up, horizontal)])
    reports = []
    for movable in ('pair', 'all'):
        if args.allow_contact_reposition and movable != 'all':
            continue
        ids = np.array([0, 1, 2, 3, 8, 9, 10, 11]) if movable == 'pair' else np.arange(16)
        lower = np.maximum(lo[ids], lo[ids] - bias[ids]) + .001
        upper = np.minimum(hi[ids], hi[ids] - bias[ids]) - .001
        for rolling_weight in (.5, .15):
            if args.allow_contact_reposition and rolling_weight != .15:
                continue
            previous = old['goals'][-1].astype(float)
            goals, angles = list(old['goals']), list(old['desired_angles'])
            shift = np.array([0., 0., 4.]) if args.with_lift else np.zeros(2)
            log = []
            rng = np.random.default_rng(427)

            def put(command, angle, shift_mm):
                rotation = Rotation.from_rotvec(up * np.radians(angle)).as_matrix() @ B
                lift = shift_mm[2] if args.with_lift else 4.
                center = C + up * lift * .001 + basis @ shift_mm[:2] * .001
                d.qpos[:16] = ticks_to_sim(command + bias)
                d.qpos[-7:-4] = center
                d.qpos[-4:] = Rotation.from_matrix(rotation).as_quat()[[3, 0, 1, 2]]
                d.qvel[:] = 0
                mujoco.mj_forward(m, d)
                return center, rotation

            def measures(center, rotation):
                points, gaps = [], []
                for tip in tips:
                    segment = np.zeros(6)
                    gaps.append(mujoco.mj_geomDistance(m, d, cube, tip, 1., segment) * 1000)
                    points.append(rotation.T @ (segment[:3] - center) * 1000)
                su, sp = inspect(d)
                other = max((max(0., -float(c.dist) * 1000) for c in d.contact
                             if cube in (int(c.geom1), int(c.geom2))
                             and (int(c.geom2) if int(c.geom1) == cube else int(c.geom1)) not in tips), default=0.)
                return np.array(points), np.array(gaps), max(sp.values(), default=0.), max(su.values(), default=0.), other

            center, rotation = put(previous, 66, shift)
            contact_points = measures(center, rotation)[0]
            for angle in range(69, 109, 3):
                def evaluate(x):
                    q = previous.astype(float).copy()
                    q[ids] = x[:len(ids)]
                    center, rotation = put(q, angle, x[len(ids):])
                    return q, measures(center, rotation)

                def objective(x):
                    q, (points, gaps, sp, su, other) = evaluate(x)
                    return np.r_[4 * (points[:, 0] - [-27.5, 27.5]),
                                 rolling_weight * (points[:, 1:] - contact_points[:, 1:]).ravel(),
                                 12 * np.maximum(0, abs(points[:, 1:]) - 25).ravel(),
                                 4 * (gaps + .65), 30 * sp, 30 * max(0, su - .1),
                                 30 * max(0, other - .7), .002 * (q[ids] - previous[ids]),
                                 .015 * x[len(ids):], .025 * (x[len(ids):] - shift)]

                best = None
                for attempt in range(6):
                    initial = np.r_[previous[ids], shift]
                    if attempt:
                        initial += np.r_[rng.normal(0, 30 if attempt < 3 else 100, len(ids)), rng.normal(0, 3, len(shift))]
                    lower_all = np.r_[lower, [-15, -15, 4] if args.with_lift else [-15, -15]]
                    upper_all = np.r_[upper, [15, 15, 25] if args.with_lift else [15, 15]]
                    fit = least_squares(objective, np.clip(initial, lower_all, upper_all), bounds=(lower_all, upper_all),
                                        diff_step=.0001, max_nfev=180)
                    x = fit.x.copy()
                    x[:len(ids)] = np.rint(x[:len(ids)])
                    q, (points, gaps, sp, su, other) = evaluate(x)
                    face_error = max(abs(points[:, 0] - [-27.5, 27.5]))
                    drift = np.max(abs(points[:, 1:] - contact_points[:, 1:]))
                    # Repositioning the point on the same cube face is an IK
                    # alternative, not a collision/force/range exemption. Free
                    # replay must decide if this contact transfer works.
                    ok = bool(face_error < .3 and np.max(abs(points[:, 1:])) <= 25
                              and (args.allow_contact_reposition or drift <= 5)
                              and np.min(gaps) >= -1.25 and np.max(gaps) <= .7
                              and sp <= 1e-6 and su <= .15 and other <= 1)
                    score = np.linalg.norm(objective(x))
                    row = dict(angle_deg=angle, goal=np.rint(q).astype(int).tolist(), shift_mm=x[len(ids):].tolist(),
                               contact_points_mm=points.tolist(), gaps_mm=gaps.tolist(), face_error_mm=float(face_error),
                               contact_slide_mm=float(drift), self_mm=sp, support_mm=su, other_cube_mm=other,
                               passed=ok, score=float(score))
                    if best is None or (ok and not best['passed']) or (ok == best['passed'] and score < best['score']):
                        best = row
                    if ok:
                        break
                log.append(best)
                print(json.dumps(dict(movable=movable, rolling_weight=rolling_weight, **best)), flush=True)
                if not best['passed']:
                    break
                q = np.array(best['goal'])
                count = max(1, int(np.ceil(max(abs(q - previous)) / 2)))
                goals.extend(np.rint(previous + (q - previous) * i / count).astype(int) for i in range(1, count + 1))
                angles.extend([angle] * count)
                previous, shift = q.astype(float), np.array(best['shift_mm'])
                contact_points = np.array(best['contact_points_mm'])
            name = f'{movable}_roll{rolling_weight:g}'
            np.savez_compressed(output / (name + '.npz'), start=base, goals=np.asarray(goals), desired_angles=np.asarray(angles), bias_ticks=bias)
            row = dict(name=name, reached_angle_deg=float(max(angles)), goals=len(goals), nominal_seconds=len(goals) * .075,
                       endpoint_log=log, contact_reposition_allowed=args.allow_contact_reposition,
                       free_cube_not_checked=True, full_edge_screen_not_done=True)
            reports.append(row)
            (output / 'ik_review.json').write_text(json.dumps(dict(status='rolling_contact_endpoint_diagnostic', cases=reports,
                 hardware_execution_allowed=False, devices_opened=False, elapsed_seconds=time.perf_counter() - began), indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
