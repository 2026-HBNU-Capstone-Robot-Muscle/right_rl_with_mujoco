"""Offline staged body grasp/lift research. Never opens physical devices.

IK temporarily pins the cube for geometry proposals. Every dynamic replay uses
the actual free cube state from the preceding stage, with unchanged guards.
"""
import os
os.environ['JAX_PLATFORMS'] = 'cpu'
import argparse, json, time
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from research_rotation_body_hold_20261104 import restore, evaluate, motor, inspect_for, native
from hand_motor_mapping import TICK, SIM_TO_MOTOR_IDS
from upper_layer_cylinder_clearance_20261105 import precompute, gaps

R = Path(__file__).resolve().parent
CP = R/'right_hand_rotation_20261105/preintegration/reference_end'
LOWER = R/'right_hand_rotation_20261105/preintegration/body_grip/lower_m20_t16_n10'
OUT = R/'right_hand_rotation_20261105/canonical_body_grip/staged_regrasp'
A = np.r_[np.arange(4, 8), np.arange(12, 16)]

def proposal(m, z, lift_mm, pinch_mm, level, thumb_lower_mm):
    d = restore(m, z); p = restore(m, z)
    base = d.qpos.copy(); pd = d.ctrl-base[:16]
    cube = m.geom('cube').id
    C = d.geom_xpos[cube].copy(); B = d.geom_xmat[cube].reshape(3, 3).copy()
    up = z['up']; y = -up
    zz = B[:, 2]-y*(B[:, 2]@y); zz /= np.linalg.norm(zz)
    endB = np.column_stack([np.cross(y, zz), y, zz]) if level else B
    rot = Slerp([0, 1], Rotation.from_matrix([B, endB]))
    lo, hi, nlo, nhi = native(m); ids = np.array(SIM_TO_MOTOR_IDS)-1
    low = np.maximum((lo[ids]+4-2048)*TICK-pd, nlo)
    high = np.minimum((hi[ids]-4-2048)*TICK-pd, nhi)
    hand, _, check = inspect_for(m); geometry = precompute(m, hand)
    tips = []
    for name in ('mf_tip', 'th_tip'):
        g = m.geom(name).id
        cs = [c for c in d.contact if {int(c.geom1), int(c.geom2)} == {cube, g}]
        if not cs: return None, [], 'missing initial lower-body tip contact'
        c = min(cs, key=lambda c: c.dist)
        G = d.geom_xmat[g].reshape(3, 3)
        n = c.frame[:3]*(1 if int(c.geom1) == cube else -1)
        tips.append((g, G.T@(c.pos-d.geom_xpos[g]), G.T@n, B.T@(c.pos-C), B.T@n))
    current = base[:16].copy(); previous = motor(d.ctrl); rows = []; trace = []
    # Establish the lower body pinch before any upward transport.
    stages = [('pinch', 8), ('transport', max(16, int(lift_mm*2)))]
    for label, steps in stages:
      for step in range(1, steps+1):
        f = step/steps
        pf = f if label == 'pinch' else 1.
        tf = 0. if label == 'pinch' else f
        Br = rot(tf).as_matrix(); Cr = C+up*lift_mm*tf/1000
        quat = Rotation.from_matrix(Br).as_quat(); pose = np.r_[Cr, quat[3], quat[:3]]
        seed = current[A].copy()
        def measure(x):
            p.qpos[:] = base; p.qpos[:16] = current; p.qpos[A] = x
            p.qpos[-7:] = pose; p.ctrl[:] = p.qpos[:16]+pd; p.qvel[:] = 0
            mujoco.mj_forward(m, p); err = []; normal = []
            for role, (g, local, localn, cl, cn) in enumerate(tips):
                target = cl-cn*pinch_mm*pf/1000
                if role == 1: target = target+np.array([0, thumb_lower_mm*tf/1000, 0])
                G = p.geom_xmat[g].reshape(3, 3)
                err.extend((p.geom_xpos[g]+G@local-(Cr+Br@target))*1000)
                normal.extend((G@localn-Br@cn)*5)
            su, sp = check(p)
            pen = max((max(0., -float(c.dist)*1000) for c in p.contact if cube in (int(c.geom1), int(c.geom2))), default=0.)
            return np.array(err), np.array(normal), max(sp.values(), default=0.), max(su.values(), default=0.), pen
        def residual(x):
            e, n, selfmm, supportmm, pen = measure(x)
            return np.r_[e, n, (x-seed)*.15, selfmm*500, supportmm*500, max(0, pen-.9)*100]
        lowx = np.maximum(low[A], seed-.18); highx = np.minimum(high[A], seed+.18)
        if np.any(lowx >= highx): return None, trace, 'native IK interval rejected'
        fit = least_squares(residual, np.clip(seed, lowx+1e-8, highx-1e-8), bounds=(lowx, highx), max_nfev=80, ftol=1e-7, xtol=1e-7, gtol=1e-7)
        e, n, selfmm, supportmm, pen = measure(fit.x)
        err = float(max(np.linalg.norm(e.reshape(2, 3), axis=1)))
        q = motor(p.ctrl)
        trace.append(dict(stage=label, step=step, tip_error_mm=err, self_mm=selfmm, support_mm=supportmm, cube_mm=pen))
        if err > .8 or selfmm > 1e-6 or supportmm > .15 or pen > 1 or np.any(q<lo) or np.any(q>hi):
            return None, trace, 'unchanged geometry/native proposal bound'
        current[A] = fit.x
        count = max(1, int(np.ceil(max(abs(q-previous))/2)))
        rows.extend(np.rint(previous+(q-previous)*j/count).astype(int) for j in range(1, count+1))
        previous = q
      if label == 'pinch': rows.extend(previous.copy() for _ in range(27))
    rows.extend(previous.copy() for _ in range(134))
    cl = gaps(m, p, Cr, Br, geometry, True)
    return np.array(rows), trace, dict(pinned_endpoint_minimum_sweep_gap_mm=min(v for name, v in cl), desired_center=Cr.tolist())

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--pinch', type=float, default=.8); ap.add_argument('--lift', type=float, default=15); ap.add_argument('--thumb-lower', type=float, default=8); ap.add_argument('--level', action='store_true'); a = ap.parse_args()
    if not (0 <= a.pinch <= .9 and 0 <= a.lift <= 25 and 0 <= a.thumb_lower <= 15): raise ValueError('research grid bounds')
    tag = f'pinch{a.pinch:g}_lift{a.lift:g}_thumb{a.thumb_lower:g}_level{int(a.level)}'; out = OUT/tag; out.mkdir(parents=True, exist_ok=True)
    m = mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb')); z = np.load(LOWER/'end_state.npz'); began = time.perf_counter()
    rows, ik, endpoint = proposal(m, z, a.lift, a.pinch, a.level, a.thumb_lower)
    report = dict(status='staged_free_cube_grip_research', parameters=vars(a), endpoint=endpoint, IK=ik, devices_opened=False, hardware_execution_allowed=False, cube_never_reset_during_dynamic_suffix=True, checkpoint_is_nominal_only=True, full_placement_prefix_not_yet_replayed=True, geometry_force_and_native_guards_unchanged=True)
    if rows is not None:
        np.savez_compressed(out/'suffix.npz', start=motor(z['ctrl']), goals=rows)
        lower = np.load(LOWER/'suffix.npz'); initial = np.load(CP/'end_state.npz')
        assert np.array_equal(lower['goals'][-1], motor(z['ctrl']))
        allrows = np.vstack([lower['goals'], rows])
        dynamic = evaluate(m, initial, allrows, out/'end_state.npz')
        final = np.load(out/'end_state.npz'); d = restore(m, final); cube = m.geom('cube').id
        C = d.geom_xpos[cube]; B = d.geom_xmat[cube].reshape(3, 3); hand, _, _ = inspect_for(m)
        cl = gaps(m, d, C, B, precompute(m, hand), True)
        margin = min(v for name, v in cl); tilt = float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'], -1, 1))))
        report.update(dynamic=dynamic, minimum_final_sweep_gap_mm=margin, closest_geoms=sorted(cl, key=lambda r:r[1])[:6], final_tilt_deg=tilt, terminal_center_error_mm=float(np.linalg.norm(C-endpoint['desired_center'])*1000), suffix_task_passed=bool(dynamic['safety_passed'] and margin>.3 and tilt<5 and dynamic['hold_shift_mm']<2 and dynamic['lower_body_opposed_contact_fraction']>.8))
    report['elapsed_seconds'] = time.perf_counter()-began
    (out/'review.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='IK' and k!='dynamic'} | {'dynamic':{k:v for k,v in report.get('dynamic', {}).items() if k!='trace'}}), flush=True)
if __name__ == '__main__': main()
