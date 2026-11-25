"""Offline reaction-load probe bound to the current selected grip's own model.

This is a rigid free cube, NOT an articulated layer or a left-hand model.
Torque magnitudes are hypothetical. Existing dynamic bounds are unchanged.
"""
import os
os.environ['JAX_PLATFORMS'] = 'cpu'
import argparse
import hashlib
import json
import time
from pathlib import Path
import numpy as np
import mujoco
from research_rotation_body_hold_20261104 import restore, inspect_for, native, angle
from upper_layer_separation_certificate_20261105 import SweepCertificate
from cube_contact_material_policy_20261029 import apply_material
from cube_task_frame_20261105 import normal,local_working_basis
from canonical_grip_dynamics_20261105 import lower_contact_roles

R = Path(__file__).resolve().parent
ROOT = R / 'right_hand_rotation_20261105/canonical_body_grip'
CANDIDATE = ROOT / 'recentered_free/i-77_m-16_t-56/from_reference/candidate.npz'
EXPECTED_CANDIDATE = 'c10e455847bf0e91111c0b149b14831528ce622ced6c46209d34c584ba251566'


def load_fraction(seconds):
    """One second baseline, ramp, five seconds full, ramp off, two seconds settle."""
    return min(1., max(0., seconds - 1.), max(0., 8. - seconds))


def evaluate(m, z, torque_nm, save):
    if abs(torque_nm) > .05 or not np.isfinite(torque_nm):
        raise ValueError('hypothetical torque outside bounded research scope')
    if abs(m.opt.timestep - .001) > 1e-12:
        raise ValueError('requires the original 1ms model')
    d = restore(m, z)
    cube = m.geom('cube').id
    body = m.body('cube').id
    hand, support, check = inspect_for(m)
    _, _, nlo, nhi = native(m)
    C0 = d.qpos[-7:-4].copy()
    B0 = d.xmat[body].reshape(3, 3).copy()
    face=int(z['top_face_id']) if 'top_face_id' in z else 5
    face_normal=normal(face);work_basis=local_working_basis(face)
    required_contacts=lower_contact_roles(tuple(str(n) for n in z['required_lower_contacts']) if 'required_lower_contacts' in z else ('mf_tip','th_tip'))
    axis = -B0[:, 1] if face==5 else B0@face_normal
    control = d.ctrl.copy()
    cert = SweepCertificate(m, hand, .3)
    worst = dict(self_mm=0., support_mm=0., cube_mm=0., tracking_rad=0., range_rad=0.)
    maxyaw = maxshift = maxtilt = gap = longest = 0.
    minimum_gap = float('inf')
    cert_bad = cert_skipped = count = lower_good = 0
    trace = []
    stop = None
    for sample in range(134):
        for sub in range(75):
            seconds = (sample * 75 + sub) * .001
            fraction = load_fraction(seconds)
            d.xfrc_applied[body, 3:] = axis * torque_nm * fraction
            mujoco.mj_step(m, d)
            count += 1
            su, sp = check(d)
            worst['self_mm'] = max(worst['self_mm'], max(sp.values(), default=0.))
            worst['support_mm'] = max(worst['support_mm'], max(su.values(), default=0.))
            worst['tracking_rad'] = max(worst['tracking_rad'], float(max(abs(d.qpos[:16] - d.ctrl))))
            worst['range_rad'] = max(worst['range_rad'], float(max(0., max(nlo - d.qpos[:16]), max(d.qpos[:16] - nhi))))
            B = d.xmat[body].reshape(3, 3)
            C = d.qpos[-7:-4]
            inhand = external = False
            contacts = {}
            for ci, c in enumerate(d.contact):
                a, b = int(c.geom1), int(c.geom2)
                if cube not in (a, b):
                    continue
                other = b if a == cube else a
                if int(m.geom_bodyid[other]) in hand:
                    inhand |= c.dist < .0004
                    worst['cube_mm'] = max(worst['cube_mm'], max(0., -float(c.dist) * 1000))
                    force = np.zeros(6)
                    mujoco.mj_contactForce(m, d, ci, force)
                    height = -float((c.pos - C) @ B[:, 1]) * 1000 if face==5 else float((c.pos-C)@(B@face_normal))*1000
                    if height < 55 / 6:
                        name = m.geom(other).name
                        contacts[name] = contacts.get(name, 0.) + float(force[0])
                elif other in support and c.dist < .0004:
                    external = True
            lower_good += all(contacts.get(name, 0.) > .2 for name in required_contacts)
            gap = 0. if inhand else gap + .001
            longest = max(longest, gap)
            yaw = abs(angle(B, B0, axis))
            shift = float(np.linalg.norm(C - C0) * 1000)
            similarity=B[:,1]@B0[:,1] if face==5 else (B@face_normal)@(B0@face_normal)
            tilt = float(np.degrees(np.arccos(np.clip(similarity, -1, 1))))
            maxyaw, maxshift, maxtilt = max(maxyaw, yaw), max(maxshift, shift), max(maxtilt, tilt)
            if not cert_bad:
                box_rotation=d.geom_xmat[cube].reshape(3,3)
                clearance, bad = cert.query(d, d.geom_xpos[cube],box_rotation if face==5 else box_rotation@work_basis)
                minimum_gap = min(minimum_gap, clearance)
                cert_bad += bool(bad)
            else:
                cert_skipped += 1
            if not np.array_equal(d.ctrl, control):
                raise RuntimeError('reaction probe changed targets')
            if (worst['self_mm'] > 1e-6 or worst['support_mm'] > .15
                    or worst['cube_mm'] > 1 or worst['tracking_rad'] > .18
                    or worst['range_rad'] > 1e-6 or external or longest > .25
                    or shift > 80):
                stop = 'unchanged contact/range/tracking/retention bound'
                break
        if sample % 13 == 0 or sample == 133 or stop:
            trace.append(dict(time_s=count * .001, load_fraction=fraction,
                              yaw_change_deg=angle(B, B0, axis), shift_mm=shift,
                              lower_contact_N=contacts, certified_gap_mm=minimum_gap))
        if stop:
            break
    np.savez_compressed(save, qpos=d.qpos, qvel=d.qvel, ctrl=d.ctrl, act=d.act,
                        qacc_warmstart=d.qacc_warmstart, time=d.time, up=z['up'],
                        initial_cube_rotation=z['initial_cube_rotation'],
                        initial_cube_center=z['initial_cube_center'],top_face_id=np.array(face),
                        required_lower_contacts=np.asarray(required_contacts))
    why = []
    if stop or count != 10050:
        why.append('original_dynamic_safety_or_incomplete')
    if maxyaw >= 3:
        why.append('yaw_change')
    if maxshift >= 2:
        why.append('translation')
    if maxtilt >= 3:
        why.append('tilt_change')
    if cert_bad or cert_skipped or cert.queries != count or minimum_gap <= .3:
        why.append('upper_layer_separation')
    if lower_good / max(1, count) <= .8:
        why.append('opposed_lower_body_contacts')
    return dict(top_face_id=face,required_lower_contacts=list(required_contacts),hypothetical_reaction_torque_nm=torque_nm, axis_world=axis.tolist(),
                safety_passed=stop is None and count == 10050, stop=stop,
                steps=count, max_yaw_change_deg=maxyaw, max_center_shift_mm=maxshift,
                max_tilt_change_deg=maxtilt, minimum_separating_gap_1ms_mm=minimum_gap,
                certificate_bad_steps=cert_bad, certificate_unqueried_steps=cert_skipped,
                certificate=cert.metadata(), opposed_lower_body_contact_fraction=lower_good / max(1, count),
                constant_targets=True, worst=worst, load_filter_passed=not why,
                reasons=why, trace=trace)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--friction', type=float, choices=(.2, .3, .5), required=True)
    ap.add_argument('--gain', type=float, choices=(.5, 1., 1.5), default=1.)
    ap.add_argument('--candidate', type=Path, default=CANDIDATE)
    ap.add_argument('--matrix-tag', default='full_placement_v2')
    ap.add_argument('--output-tag', default='selected_v2')
    ap.add_argument('--magnitude', type=float, nargs='+', default=[.003, .01, .02])
    a = ap.parse_args()
    candidate_sha = hashlib.sha256(a.candidate.read_bytes()).hexdigest()
    if a.candidate == CANDIDATE and candidate_sha != EXPECTED_CANDIDATE:
        raise ValueError('selected candidate changed')
    if not a.magnitude or any(not 0 < value <= .05 for value in a.magnitude) or len(set(a.magnitude)) != len(a.magnitude):
        raise ValueError('bounded distinct hypothetical load magnitudes required')
    matrix = ROOT / a.matrix_tag / 'front-10'
    raw_report = json.loads((matrix / 'review.json').read_bytes())
    if raw_report['candidate_sha256'] != candidate_sha:
        raise ValueError('source matrix belongs to a different candidate')
    source_case = next(c for c in raw_report['cases'] if (c['gain_scale'], c['friction']) == (a.gain, a.friction))
    if not source_case['continuous'] or not source_case['continuous']['safety_passed']:
        raise ValueError('source case is incomplete or failed its dynamic guard')
    folder = matrix / f'g{a.gain:g}_f{a.friction:g}'
    # Replay saved only the nominal binary. Other g1 cases differ solely in the
    # explicit material hypothesis, reconstructed by the same replay policy.
    model = matrix / 'g1_f0.3/model.mjb'
    source = folder / 'end_state.npz'
    m = mujoco.MjModel.from_binary_path(str(model))
    m.actuator_gainprm[:, 0] *= a.gain
    m.actuator_biasprm[:, 1] *= a.gain
    hand, _, _ = inspect_for(m)
    material = apply_material(m, a.friction, hand, m.geom('cube').id)
    z = np.load(source)
    out = ROOT / 'layer_reaction' / (f'{a.output_tag}_f{a.friction:g}' if a.gain == 1 else f'{a.output_tag}_g{a.gain:g}_f{a.friction:g}')
    out.mkdir(parents=True, exist_ok=True)
    cases = []
    for torque in [0.] + [sign * magnitude for magnitude in a.magnitude for sign in (-1, 1)]:
        began = time.perf_counter()
        row = evaluate(m, z, torque, out / f'torque{torque:+g}.npz')
        row['elapsed_seconds'] = time.perf_counter() - began
        cases.append(row)
        report = dict(status='hypothetical_reaction_on_source_bound_rigid_cube',
                      candidate=str(a.candidate.relative_to(R)), candidate_sha256=candidate_sha,
                      source=str(source.relative_to(R)), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                      model=str(model.relative_to(R)), model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                      material_reconstructed_from_same_full_replay_policy=material,
                      source_matrix_sha256=hashlib.sha256((matrix/'review.json').read_bytes()).hexdigest(),
                      friction=a.friction, gain_scale=a.gain, book_front_mm=-10., cases=cases,
                      load_profile='baseline0-1s, ramp1-2s, full2-7s, rampoff7-8s, settle8-10.05s',
                      actual_left_hand_load_measured=False, articulated_layer_not_simulated=True,
                      physics_and_current_and_PID_and_native_guards_unchanged=True,
                      hardware_execution_allowed=False, devices_opened=False)
        (out / 'review.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps({k: v for k, v in row.items() if k not in ('trace', 'certificate')}), flush=True)


if __name__ == '__main__':
    main()
