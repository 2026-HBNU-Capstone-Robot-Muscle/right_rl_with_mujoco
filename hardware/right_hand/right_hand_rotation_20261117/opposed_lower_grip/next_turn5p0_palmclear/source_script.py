"""Bounded next-turn research from the actual saved end of a verified hold.

Perfect simulated pose/contact feedback; not SAC or a hardware executable.
No cube/hand resets between the hold endpoint and this continuation.
"""
from pathlib import Path
import argparse, json, sys, time
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore, inspect_for, native, motor
from research_quasistatic_wrench_path_20261105 import allocate_native, hybrid_increment
from offline_fractional_goal_accumulator_20261105 import FractionalGoals
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS, TICK
from audit_opposed_working_grip_20261117 import MODEL, ROLES, replay, sha

PARENT = ROOT/'right_hand_rotation_20261117/opposed_lower_grip/level_triple_normal1p2_settle1_settled_frame_active_hold400_heightlock0p0/complete_hold_audit'


class PalmClearPlanner(FractionalGoals):
    """Project the proposed tick increment away from a nearby palm surface.

    This edits the bounded proposal, not collision geometry or stop thresholds.
    The free dynamics and independent replay must still pass all guards.
    """
    def __init__(self, start, lo, hi, model, data):
        super().__init__(start, lo, hi)
        self.model, self.data = model, data
        self.interventions = 0
        self.ids = np.asarray(SIM_TO_MOTOR_IDS)-1

    def advance(self, increment):
        m, d = self.model, self.data
        palm, tip = m.geom('palm_collision_6').id, m.geom('th_tip').id
        points = np.zeros(6)
        distance = float(mujoco.mj_geomDistance(m, d, palm, tip, .003, points))
        proposed = np.asarray(increment, float).copy()
        if 1e-9 < distance < .002:
            normal = (points[3:]-points[:3])/distance
            jac = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jac, None, points[3:], int(m.geom_bodyid[tip]))
            row = normal@jac[:, :16]
            gradient = np.zeros(16)
            gradient[self.ids] = row*TICK
            velocity = float(normal@jac@d.qvel)
            required = .0002+max(0., -velocity*.075)-distance
            lower, upper = self.increment_bounds()
            if float(gradient@proposed) < required:
                maximum = np.where(gradient >= 0, upper, lower)
                if float(gradient@maximum) < required:
                    raise ValueError('thumb_palm_clearance_projection_infeasible')
                left, right = 0., 1.
                while float(gradient@np.clip(proposed+right*gradient, lower, upper)) < required:
                    right *= 2
                for _ in range(50):
                    mid = (left+right)/2
                    if float(gradient@np.clip(proposed+mid*gradient, lower, upper)) < required: left = mid
                    else: right = mid
                proposed = np.clip(proposed+right*gradient, lower, upper)
                self.interventions += 1
        return super().advance(proposed)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--turn-deg', type=float, choices=(-5., 5., 15., 90.), default=5.)
    ap.add_argument('--avoid-thumb-palm', action='store_true')
    a = ap.parse_args()
    previous = json.loads((PARENT/'review.json').read_bytes())
    if not previous['passed'] or sha(MODEL) != previous['model_sha256']:
        raise ValueError('Verified source hold/model required')
    source = PARENT/'audit_end_state.npz'
    out = ROOT/'right_hand_rotation_20261117/opposed_lower_grip'/('next_turn'+str(a.turn_deg).replace('.', 'p')+('_palmclear' if a.avoid_thumb_palm else ''))
    out.mkdir(exist_ok=False)
    (out/'source_script.py').write_bytes(Path(__file__).read_bytes())
    m = mujoco.MjModel.from_binary_path('model.mjb', assets={'model.mjb': MODEL.read_bytes()})
    z = dict(np.load(source, allow_pickle=False))
    d = restore(m, z)
    hand, supports, check = inspect_for(m)
    lo, hi, nlo, nhi = native(m)
    cube, body = m.geom('cube').id, m.body('cube').id
    C0 = d.geom_xpos[cube].copy()
    B0 = d.geom_xmat[cube].reshape(3, 3).copy()
    axis = -B0[:, 1]
    targetB = Rotation.from_rotvec(axis*np.radians(a.turn_deg)).as_matrix()@B0
    goal = start = motor(d.ctrl)
    planner = PalmClearPlanner(start, lo, hi, m, d) if a.avoid_thumb_palm else FractionalGoals(start, lo, hi)
    references = {'if_tip': .8, 'mf_tip': .8, 'th_tip': 1.6}
    heights, normals, memory = {}, {}, {}
    for name in ROLES:
        g = m.geom(name).id
        c = min((c for c in d.contact if {int(c.geom1), int(c.geom2)} == {cube, g}), key=lambda c: c.dist)
        normal = c.frame[:3]*(1 if int(c.geom1) == cube else -1)
        normals[name] = (d.geom_xmat[g].reshape(3, 3).T@normal, B0.T@normal)
        heights[name] = float((c.pos-C0)@axis)
    rows, phases, trace = [], [], []
    hold_start = None
    stable = 0
    stop = None
    began = time.perf_counter()
    loss = 0.
    for index in range(534):
        phase = 'turn' if hold_start is None else 'hold'
        fraction = min(1., index/134.)
        blend = fraction*fraction*(3-2*fraction)
        Bt = targetB if hold_start is not None else Rotation.from_rotvec(axis*np.radians(a.turn_deg*blend)).as_matrix()@B0
        C = d.geom_xpos[cube].copy()
        B = d.geom_xmat[cube].reshape(3, 3).copy()
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
        F = -m.body_mass[body]*m.opt.gravity+30*(C0-C)-1.5*velocity[3:]
        T = .04*Rotation.from_matrix(Bt@B.T).as_rotvec()-.002*velocity[:3]
        proposal, allocation = allocate_native(m, d, cube, C, F, T, references.copy(), hand, True, (nlo, nhi), ROLES, normal_reference_weight=.2)
        if proposal is None:
            stop = {'reason': 'force_allocation', 'details': allocation['reason']}
            break
        try:
            goal = hybrid_increment(m, d, allocation, planner, C, C0, B, Bt, tip_memory=memory,
                contact_height_targets=heights, world_up=-B[:, 1], maximum_tracking_rad=.12,
                contact_normal_frames=normals, tip_memory_names=ROLES, recovery_normal_targets=references)
        except ValueError as exc:
            if str(exc) != 'thumb_palm_clearance_projection_infeasible': raise
            stop = {'reason': str(exc), 'goal': index+1}
            break
        rows.append(goal.copy())
        phases.append(phase)
        d.ctrl[:] = ticks_to_sim(goal)
        for sub in range(75):
            mujoco.mj_step(m, d)
            su, sp = check(d)
            values = dict(self_mm=max(sp.values(), default=0.), support_mm=max(su.values(), default=0.),
                cube_mm=0., tracking_rad=float(max(abs(d.qpos[:16]-d.ctrl))),
                range_rad=float(max(0., max(nlo-d.qpos[:16]), max(d.qpos[:16]-nhi))))
            inhand = external = False
            for c in d.contact:
                aa, bb = int(c.geom1), int(c.geom2)
                if cube not in (aa, bb): continue
                other = bb if aa == cube else aa
                if int(m.geom_bodyid[other]) in hand:
                    inhand |= c.dist < .0004
                    values['cube_mm'] = max(values['cube_mm'], max(0., -float(c.dist)*1000))
                else: external |= other in supports and c.dist < .0004
            loss = 0. if inhand else loss+.001
            failed = [key for key, limit in [('self_mm', 1e-6), ('support_mm', .15),
                ('cube_mm', 1.), ('tracking_rad', .18), ('range_rad', 1e-6)] if values[key] > limit]
            if external: failed.append('external_cube_support')
            if loss > .25: failed.append('hand_contact_loss')
            if not np.isfinite(np.r_[d.qpos, d.qvel]).all(): failed.append('nonfinite_state')
            if np.linalg.norm(d.qpos[-7:-4]-C0) > .08: failed.append('translation_bound')
            if failed:
                stop = dict(reason='original_dynamic_bound', goal=index+1, substep=sub+1, failed=failed, values=values)
                break
        B = d.geom_xmat[cube].reshape(3, 3).copy()
        error = float(np.degrees(np.linalg.norm(Rotation.from_matrix(targetB@B.T).as_rotvec())))
        rotation = float(np.degrees(Rotation.from_matrix(B@B0.T).as_rotvec()@axis))
        shift = float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000)
        ready = index >= 134 and error < 1. and shift < 1.5 and abs(rotation) >= abs(a.turn_deg)*.8
        if hold_start is None:
            stable = stable+1 if ready else 0
            if stable >= 10: hold_start = len(rows)
        if index % 20 == 0 or stop:
            trace.append(dict(goal=index+1, phase=phase, actual_turn_deg=rotation, target_error_deg=error, center_error_mm=shift))
            print(json.dumps(trace[-1]), flush=True)
        if stop or (hold_start is not None and len(rows)-hold_start >= 134): break
        if hold_start is None and index >= 399:
            stop = {'reason': 'bounded_turn_not_reached', 'actual_turn_deg': rotation, 'target_error_deg': error}
            break
    np.savez_compressed(out/'research_goals.npz', start=start, goals=rows, phases=phases)
    np.savez_compressed(out/'end_state.npz', qpos=d.qpos, qvel=d.qvel, ctrl=d.ctrl, act=d.act,
        qacc_warmstart=d.qacc_warmstart, time=d.time, up=z['up'],
        initial_cube_rotation=z['initial_cube_rotation'], initial_cube_center=z['initial_cube_center'])
    audit = None
    if stop is None and hold_start is not None and len(rows)-hold_start == 134:
        audit = replay(m, z, rows, hold_start, out, C0, targetB)
    report = dict(status='bounded_next_turn_from_verified_hold', source_path=source.relative_to(ROOT).as_posix(),
        source_sha256=sha(source), parent_review_sha256=sha(PARENT/'review.json'), model_sha256=sha(MODEL),
        script_sha256=sha(Path(__file__)), target_turn_deg=a.turn_deg, actual_turn_deg=rotation,
        target_error_deg=error, center_error_mm=shift, stop=stop, goals=len(rows), hold_start=hold_start,
        audit=audit, small_turn_and_hold_passed=bool(audit and audit['passed']), trace=trace,
        thumb_palm_clearance_projection=a.avoid_thumb_palm,
        clearance_projection_interventions=getattr(planner, 'interventions', 0),
        original_dynamic_guards_unchanged=True, cube_reset_after_start=False, perfect_simulated_feedback=True,
        new_SAC_training=False, hardware_execution_allowed=False, devices_opened=False,
        full_quarter_turn_cycle_verified=False, elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('trace', 'audit')}, ensure_ascii=True), flush=True)


if __name__ == '__main__':
    main()
