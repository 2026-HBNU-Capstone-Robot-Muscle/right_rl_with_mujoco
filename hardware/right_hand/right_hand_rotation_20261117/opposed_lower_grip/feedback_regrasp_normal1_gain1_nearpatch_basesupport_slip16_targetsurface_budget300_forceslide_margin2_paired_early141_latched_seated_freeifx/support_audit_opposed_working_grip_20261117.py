"""Independently replay offline goals and audit the complete working-grip hold.

The reference is the saved hand-relative cube pose. Gravity tilt is reported,
not silently replaced in previous gravity-upright task verdicts. Clearance is
a separating-plane lower bound against the exact upper sweep cylinder; the
slab/radial screen is not a Euclidean distance bound near cylinder caps.
No hardware transport, model edits, cube forces or post-start resets.
"""
from pathlib import Path
import argparse, hashlib, json, sys, time
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore, inspect_for, native, motor
from hand_motor_mapping import ticks_to_sim
from upper_layer_separation_certificate_20261105 import SweepCertificate

MODEL = ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
ROLES = ('if_tip', 'mf_tip', 'th_tip')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replay(m, z, rows, hold_start, out, target_center=None, target_rotation=None, required_roles=ROLES):
    required_roles=tuple(required_roles)
    if (not 2 <= len(required_roles) <= 3 or len(set(required_roles)) != len(required_roles)
        or not set(required_roles) <= set(ROLES) or 'th_tip' not in required_roles):
        raise ValueError('Thumb and at least one distinct back finger required')
    d = restore(m, z)
    hand, supports, check = inspect_for(m)
    lo, hi, nlo, nhi = native(m)
    rows = np.asarray(rows, int)
    start = motor(d.ctrl)
    if rows.ndim != 2 or rows.shape[1] != 16 or not 0 <= hold_start < len(rows):
        raise ValueError('Invalid recorded goal/hold contract')
    if np.any(rows < lo) or np.any(rows > hi) or np.max(abs(np.diff(np.vstack([start, rows]), axis=0))) > 2:
        raise ValueError('Native range or two-tick goal bound')
    cube = m.geom('cube').id
    C0 = d.geom_xpos[cube].copy()
    target_center = C0.copy() if target_center is None else np.asarray(target_center)
    B0 = d.geom_xmat[cube].reshape(3, 3).copy()
    target_rotation = B0.copy() if target_rotation is None else np.asarray(target_rotation)
    certificate = SweepCertificate(m, hand, .3)
    gap = 0.
    steps = lower_three = required_good = opposed = supported = 0
    cert_bad = 0
    minimum_cert = float('inf')
    max_angle = max_shift = 0.
    stop = None
    snapshots = []
    timeline = []
    worst = dict(self_mm=0., support_mm=0., cube_mm=0., tracking_rad=0., range_rad=0.)
    for index, goal in enumerate(rows):
        d.ctrl[:] = ticks_to_sim(goal)
        for sub in range(75):
            mujoco.mj_step(m, d)
            su, sp = check(d)
            current = dict(self_mm=max(sp.values(), default=0.), support_mm=max(su.values(), default=0.),
                cube_mm=0., tracking_rad=float(max(abs(d.qpos[:16]-d.ctrl))),
                range_rad=float(max(0., max(nlo-d.qpos[:16]), max(d.qpos[:16]-nhi))))
            C = d.geom_xpos[cube]
            B = d.geom_xmat[cube].reshape(3, 3)
            inhand = external = False
            loads = dict.fromkeys(ROLES, 0.)
            front = back = underneath = 0.
            for ci, contact in enumerate(d.contact):
                aa, bb = int(contact.geom1), int(contact.geom2)
                if cube not in (aa, bb):
                    continue
                other = bb if aa == cube else aa
                if int(m.geom_bodyid[other]) not in hand:
                    external |= other in supports and contact.dist < .0004
                    continue
                current['cube_mm'] = max(current['cube_mm'], max(0., -float(contact.dist)*1000))
                inhand |= contact.dist < .0004
                if index < hold_start or contact.dist >= .0004:
                    continue
                force = np.zeros(6)
                mujoco.mj_contactForce(m, d, ci, force)
                normal = (contact.frame[:3]*(1 if aa == cube else -1)) @ B
                local = (contact.pos-C) @ B
                name = m.geom(other).name
                if -local[1] < 55/6000 and name in loads:
                    loads[name] += float(force[0])
                    if name == 'th_tip' and normal[2] > .5:
                        front += float(force[0])
                    elif name in ('if_tip', 'mf_tip') and normal[2] < -.5:
                        back += float(force[0])
                if local[1] > .020 and normal[1] > .5:
                    underneath += float(force[0])
            for key, value in current.items():
                worst[key] = max(worst[key], value)
            gap = 0. if inhand else gap+.001
            reasons = []
            for key, limit in [('self_mm', 1e-6), ('support_mm', .15), ('cube_mm', 1.),
                               ('tracking_rad', .18), ('range_rad', 1e-6)]:
                if current[key] > limit:
                    reasons.append(key)
            if not np.isfinite(np.r_[d.qpos, d.qvel]).all(): reasons.append('nonfinite_state')
            if external: reasons.append('external_cube_support')
            if gap > .25: reasons.append('hand_contact_loss')
            if np.linalg.norm(C-C0) > .08: reasons.append('cube_translation_bound')
            if index >= hold_start:
                steps += 1
                lower_three += all(loads[name] > .2 for name in ROLES)
                required_good += all(loads[name] > .2 for name in required_roles)
                opposed += front > .2 and back > .2
                supported += underneath > .05
                max_angle = max(max_angle, float(np.degrees(np.linalg.norm(Rotation.from_matrix(B@target_rotation.T).as_rotvec()))))
                max_shift = max(max_shift, float(np.linalg.norm(C-target_center)*1000))
                value, bad = certificate.query(d, C, B)
                minimum_cert = min(minimum_cert, value)
                cert_bad += bool(bad)
                if bad: reasons.append('upper_sweep_clearance')
            if reasons:
                stop = dict(goal=index+1, substep=sub+1, reasons=reasons, current=current)
                break
        if index % 20 == 0 or index == len(rows)-1 or stop:
            timeline.append(dict(goal=index+1, hold_steps=steps, max_angle_deg=max_angle,
                max_shift_mm=max_shift, certified_gap_mm=minimum_cert if steps else None))
        if index % 2 == 0 or index == len(rows)-1 or stop:
            snapshots.append(d.qpos.copy())
        if stop: break
    expected = (len(rows)-hold_start)*75
    verdict = bool(stop is None and steps == expected and cert_bad == 0 and max_angle < 5
                   and max_shift < 2 and required_good/max(1, steps) > .8 and opposed/max(1, steps) > .8)
    if out is not None:
        np.savez_compressed(out/'audit_end_state.npz', qpos=d.qpos, qvel=d.qvel, ctrl=d.ctrl,
            act=d.act, qacc_warmstart=d.qacc_warmstart, time=d.time,
            up=z['up'], initial_cube_rotation=z['initial_cube_rotation'], initial_cube_center=z['initial_cube_center'])
        np.savez_compressed(out/'audit_trajectory.npz', qpos=snapshots)
    return dict(status='independent_complete_working_hold_replay', passed=verdict, stop=stop,
        completed_goals=index+1, total_goals=len(rows), hold_start_zero_based=hold_start,
        evaluated_hold_seconds=steps*.001, expected_hold_steps=expected, hold_steps=steps,
        reference='saved hand-relative working pose', maximum_angle_error_deg=max_angle,
        maximum_center_error_mm=max_shift, lower_three_contact_fraction=lower_three/max(1, steps),
        required_lower_contacts=list(required_roles),required_lower_contact_fraction=required_good/max(1, steps),
        opposed_front_back_fraction=opposed/max(1, steps), palm_underside_support_fraction=supported/max(1, steps),
        minimum_exact_support_plane_gap_mm=minimum_cert if steps else None,
        upper_space_bad_steps=cert_bad, certificate=certificate.metadata(), worst=worst,
        source_gravity_tilt_deg=float(np.degrees(np.arccos(np.clip((-B0[:, 1])@z['up'], -1, 1)))),
        final_gravity_tilt_deg=float(np.degrees(np.arccos(np.clip((-B[:, 1])@z['up'], -1, 1)))),
        timeline=timeline, previous_gravity_upright_verdict_unchanged=True,
        original_limits_unchanged=True, devices_opened=False, cube_reset_after_start=False,
        physical_grasp_verified=False, reaching_path_verified=False, continuous_cycle_verified=False,
        hardware_execution_allowed=False,validator_sha256=sha(Path(__file__)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--parent', required=True)
    a = ap.parse_args()
    base = ROOT/'right_hand_rotation_20261117/opposed_lower_grip'/a.parent
    parent = json.loads((base/'review.json').read_bytes())
    if parent['stop'] is not None: raise ValueError('Parent stopped before completing requested hold')
    source = ROOT/parent['source_path']
    if sha(source) != parent['source_sha256'] or sha(MODEL) != parent['model_sha256']:
        raise ValueError('Parent source/model hash changed')
    candidate = np.load(base/'research_goals.npz', allow_pickle=False)
    phases = candidate['phases']
    hold = np.flatnonzero(phases != 'level')
    if not len(hold) or not np.all(phases[hold[0]:] != 'level'):
        raise ValueError('Expected one contiguous hold segment')
    out = base/'complete_hold_audit'
    out.mkdir(exist_ok=False)
    m = mujoco.MjModel.from_binary_path('model.mjb', assets={'model.mjb': MODEL.read_bytes()})
    began = time.perf_counter()
    result = replay(m, dict(np.load(source, allow_pickle=False)), candidate['goals'], int(hold[0]), out)
    result.update(source_sha256=sha(source), model_sha256=sha(MODEL),
        parent_review_sha256=sha(base/'review.json'), goals_sha256=sha(base/'research_goals.npz'),
        source_script_sha256=sha(Path(__file__)), elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('timeline', 'certificate')}, ensure_ascii=True), flush=True)


if __name__ == '__main__':
    main()
