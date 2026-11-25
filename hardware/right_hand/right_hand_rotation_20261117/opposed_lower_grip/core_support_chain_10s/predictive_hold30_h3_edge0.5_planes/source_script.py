"""Offline finite-difference shooting controller for the achieved opposed grip.

Every candidate is rolled forward with unchanged free MuJoCo dynamics. Only
bounded integer joint goals are committed; the cube is never repositioned in
the accepted trajectory. This requires perfect model state, not real sensors.
"""
from pathlib import Path
import argparse, json, sys, time
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import numpy as np
import mujoco
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from audit_opposed_working_grip_20261117 import MODEL, replay, sha
from research_rotation_body_hold_20261104 import restore, inspect_for, native, motor
from hand_motor_mapping import TICK, SIM_TO_MOTOR_IDS, ticks_to_sim
from upper_layer_separation_certificate_20261105 import SweepCertificate, cylinder_minimum
from gait_middle_contact_20261117 import observe

BASE = ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
PARENT = BASE/'core_support_chain_10s'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--horizon', type=int, choices=(1, 3), default=3)
    ap.add_argument('--edge-weight', type=float, choices=(0., .5), default=.5)
    args = ap.parse_args()
    out = PARENT/f'predictive_hold30_h{args.horizon}_edge{args.edge_weight:g}_planes'
    out.mkdir(exist_ok=False)
    (out/'source_script.py').write_bytes(Path(__file__).read_bytes())
    prior = json.loads((PARENT/'review.json').read_bytes())
    source = PARENT/'audit_end_state.npz'
    original = ROOT/prior['source_path']
    if not prior['working_grip_passed'] or sha(MODEL) != prior['model_sha256'] or sha(original) != prior['source_sha256']:
        raise RuntimeError('Verified source/model required')
    m = mujoco.MjModel.from_binary_path('model.mjb', assets={'model.mjb': MODEL.read_bytes()})
    z = dict(np.load(source, allow_pickle=False))
    d = restore(m, z)
    trial = mujoco.MjData(m)
    hand, supports, check = inspect_for(m)
    lo, hi, nlo, nhi = native(m)
    cube = m.geom('cube').id
    C0 = d.geom_xpos[cube].copy()
    B0 = d.geom_xmat[cube].reshape(3, 3).copy()
    active = (np.asarray(SIM_TO_MOTOR_IDS)-1)[np.r_[np.arange(8), np.arange(12, 16)]]
    state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
    saved = np.zeros(mujoco.mj_stateSize(m, state_spec))
    start = motor(d.ctrl)
    goal = start.copy()
    references, _ = observe(m, d, cube)
    reference_loads = np.array([references[n] for n in ('mf_tip', 'th_tip')])
    certificate = SweepCertificate(m, hand, .3)
    certificate.query(d, C0, B0)
    rows, trace, snapshots = [], [], []
    stop = None
    loss = 0.
    candidates = 0
    began = time.perf_counter()
    properties = {k: getattr(m, k).copy() for k in (
        'actuator_gainprm', 'actuator_biasprm', 'actuator_forcerange', 'jnt_range',
        'geom_size', 'geom_contype', 'geom_conaffinity', 'geom_friction')}

    def features(x):
        C = x.geom_xpos[cube]
        B = x.geom_xmat[cube].reshape(3, 3)
        loads, contacts = observe(m, x, cube)
        normal = np.array([loads[n] for n in ('mf_tip', 'th_tip')])
        edge = 0.
        if 'mf_tip' in contacts:
            local = (contacts['mf_tip'][1].pos-C)@B*1000
            edge = max(0., abs(local[0])-25.5)*args.edge_weight
        else:
            edge = 2.*args.edge_weight
        G = x.geom_xmat[certificate.ids].reshape(-1, 3, 3)
        axes = np.einsum('nij,nj->ni', G, certificate.normal)@B
        p = (x.geom_xpos[certificate.ids]-C)@B*1000
        values = cylinder_minimum(axes)-certificate.offset-np.einsum('ni,ni->n', axes, p)
        # The current valid plane is held fixed in each prediction. These
        # values remain exact lower bounds, even if a better plane exists.
        clear = np.maximum(.8-values, 0)/.2
        su, sp = check(x)
        collision = np.array([max(sp.values(), default=0.), max(su.values(), default=0.)])/.02
        rot = Rotation.from_matrix(B@B0.T).as_rotvec()
        return np.r_[(C-C0)*2000, np.degrees(rot),
                     x.qvel[-6:-3]*100, x.qvel[-3:]*3,
                     (normal-reference_loads)/.3, edge, collision, clear]

    def predict(candidate):
        nonlocal candidates
        mujoco.mj_setState(m, trial, saved, state_spec)
        trial.ctrl[:] = ticks_to_sim(candidate)
        # mj_step recomputes forward dynamics; no state projection or cube
        # pose assignment is performed inside any forward rollout.
        bad = False
        for step in range(75*args.horizon):
            mujoco.mj_step(m, trial)
            if step % 5 == 0:
                su, sp = check(trial)
                bad |= max(sp.values(), default=0.) > 1e-6 or max(su.values(), default=0.) > .15
        candidates += 1
        return features(trial), bad

    for index in range(266):
        mujoco.mj_getState(m, d, saved, state_spec)
        baseline, baseline_bad = predict(goal)
        lower = np.maximum(np.ceil(lo)-goal, -2.)
        upper = np.minimum(np.floor(hi)-goal, 2.)
        # Keep hypothetical goals within the unchanged 0.12rad soft tracking
        # margin. The actual trajectory is checked against 0.18rad every ms.
        actual = np.empty(16)
        actual[np.asarray(SIM_TO_MOTOR_IDS)-1] = d.qpos[:16]/TICK+2048
        lower = np.maximum(lower, actual-.12/TICK-goal)
        upper = np.minimum(upper, actual+.12/TICK-goal)
        # These vectors use physical motor-ID order: ring IDs13..16.
        lower[12:16] = upper[12:16] = 0.
        J = np.zeros((len(baseline), len(active)))
        for col, joint in enumerate(active):
            step = 1. if upper[joint] >= 1. else (-1. if lower[joint] <= -1. else 0.)
            if step:
                candidate = goal.astype(float)
                candidate[joint] += step
                J[:, col] = (predict(candidate)[0]-baseline)/step
        amin, amax = lower[active], upper[active]
        free = amax-amin > 1e-9
        delta = np.zeros(len(active))
        delta[~free] = (amin[~free]+amax[~free])/2
        if np.any(free):
            fit = lsq_linear(np.vstack([J[:, free], np.eye(sum(free))*.1]),
                np.r_[-baseline-J[:, ~free]@delta[~free], np.zeros(sum(free))],
                bounds=(amin[free], amax[free]), tol=1e-8, max_iter=100)
            delta[free] = fit.x
        choices = [(float(baseline@baseline)+(1e8 if baseline_bad else 0), goal.copy())]
        for fraction in (1., .5):
            candidate = goal.copy()
            candidate[active] = np.rint(goal[active]+delta*fraction).astype(int)
            candidate = np.clip(candidate, np.ceil(goal+lower), np.floor(goal+upper)).astype(int)
            value, bad = predict(candidate)
            choices.append((float(value@value)+(1e8 if bad else 0), candidate))
        score, goal = min(choices, key=lambda item: item[0])
        if max(abs(goal-(rows[-1] if rows else start))) > 2 or np.any(goal<lo) or np.any(goal>hi):
            raise RuntimeError('Native integer/two-tick goal contract')
        rows.append(goal.copy())
        d.ctrl[:] = ticks_to_sim(goal)
        gap = None
        for sub in range(75):
            mujoco.mj_step(m, d)
            su, sp = check(d)
            values = dict(self_mm=max(sp.values(), default=0.), support_mm=max(su.values(), default=0.),
                cube_mm=0., tracking_rad=float(max(abs(d.qpos[:16]-d.ctrl))),
                range_rad=float(max(0., max(nlo-d.qpos[:16]), max(d.qpos[:16]-nhi))))
            inhand = external = False
            for c in d.contact:
                a, b = int(c.geom1), int(c.geom2)
                if cube not in (a, b): continue
                other = b if a == cube else a
                if int(m.geom_bodyid[other]) in hand:
                    inhand |= c.dist < .0004
                    values['cube_mm'] = max(values['cube_mm'], max(0., -float(c.dist)*1000))
                else:
                    external |= other in supports and c.dist < .0004
            loss = 0. if inhand else loss+.001
            failed = [key for key, bound in [('self_mm', 1e-6), ('support_mm', .15), ('cube_mm', 1.),
                ('tracking_rad', .18), ('range_rad', 1e-6)] if values[key] > bound]
            if external: failed.append('external_cube_support')
            if loss > .25: failed.append('hand_contact_loss')
            if not np.isfinite(np.r_[d.qpos, d.qvel]).all(): failed.append('nonfinite_state')
            if np.linalg.norm(d.geom_xpos[cube]-C0) > .08: failed.append('translation_bound')
            gap, bad = certificate.query(d, d.geom_xpos[cube], d.geom_xmat[cube].reshape(3, 3))
            if bad: failed.append('upper_sweep_clearance')
            if failed:
                stop = dict(reason='unchanged_guard', goal=index+1, substep=sub+1, failed=failed, values=values)
                break
        loads, live = observe(m, d, cube)
        if index % 20 == 0 or stop:
            C = d.geom_xpos[cube]
            B = d.geom_xmat[cube].reshape(3, 3)
            trace.append(dict(goal=index+1, cost=score, lower_N=loads, gap_mm=gap,
                center_error_mm=float(np.linalg.norm(C-C0)*1000),
                angle_error_deg=float(np.degrees(np.linalg.norm(Rotation.from_matrix(B@B0.T).as_rotvec()))),
                middle_edge_margin_mm=float(27.5-abs(((live['mf_tip'][1].pos-C)@B)[0]*1000)) if 'mf_tip' in live else None,
                elapsed_seconds=time.perf_counter()-began))
            print(json.dumps(trace[-1]), flush=True)
        if index % 2 == 0 or stop: snapshots.append(d.qpos.copy())
        if stop: break
    for k, v in properties.items():
        if not np.array_equal(v, getattr(m, k)): raise RuntimeError('Model mutated')
    np.savez_compressed(out/'suffix.npz', start=start, goals=rows)
    np.savez_compressed(out/'trajectory.npz', qpos=snapshots)
    np.savez_compressed(out/'end_state.npz', qpos=d.qpos, qvel=d.qvel, ctrl=d.ctrl, act=d.act,
        qacc_warmstart=d.qacc_warmstart, time=d.time, up=z['up'],
        initial_cube_rotation=z['initial_cube_rotation'], initial_cube_center=z['initial_cube_center'])
    audit = None
    if stop is None and len(rows) == 266:
        old = np.load(PARENT/'research_goals.npz', allow_pickle=False)
        full = np.vstack([old['goals'], rows])
        target = json.loads((BASE/'feedback_regrasp_normal1_gain1_nearpatch_basesupport_slip16_targetsurface_budget300_forceslide_margin2_paired_early141_latched_seated_freeifx/review.json').read_bytes())
        audit = replay(m, dict(np.load(original, allow_pickle=False)), full, prior['hold_start'], out,
            target['hold_target_center_world_m'], target['hold_target_rotation_world'], required_roles=('mf_tip', 'th_tip'))
        np.savez_compressed(out/'research_goals.npz', start=old['start'], goals=full, hold_start=prior['hold_start'])
    result = dict(status='finite_difference_predictive_achieved_grip_hold', passed=bool(audit and audit['passed']),
        stop=stop, audit=audit, suffix_goals=len(rows), prediction_horizon_goals=args.horizon,
        middle_edge_weight=args.edge_weight, prediction_rollouts=candidates, trace=trace,
        source_sha256=sha(source), source_path=source.relative_to(ROOT).as_posix(), model_sha256=sha(MODEL),
        original_source_sha256=sha(original), parent_review_sha256=sha(PARENT/'review.json'),
        script_sha256=sha(Path(__file__)), model_properties_unchanged=True, perfect_simulated_feedback=True,
        speculative_prediction_state_copies=True, accepted_cube_pose_resets=0,
        full_quarter_turn_cycle_verified=False, new_SAC_training=False, physical_grasp_verified=False,
        devices_opened=False, hardware_execution_allowed=False, elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('trace', 'audit')}, ensure_ascii=True), flush=True)


if __name__ == '__main__':
    main()
