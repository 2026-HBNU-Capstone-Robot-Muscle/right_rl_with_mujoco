"""Find opposing model contacts at the reached stage-2 cube, then free-replay.

Geometry optimization fixes the cube only in hypothetical IK copies. Every
selected approach runs with the free cube and all unchanged native guards.
This script has no motor/camera interfaces and never authorizes hardware.
"""
import json
import sys
import time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
WORK = ROOT / 'right_hand_rotation_20261109/prehardware_policy_cycle'
sys.path.insert(0, str(WORK))
import mujoco
from scipy.optimize import differential_evolution
from scipy.spatial import ConvexHull
from native_cycle import assets, restore, Guard, BOUND, motor, ticks_to_sim, TICK, SIM_TO_MOTOR_IDS, sha, save_json, angle
from whole_hand_support import sample_model
from upper_layer_cylinder_clearance_20261105 import precompute
from cube_task_frame_20261105 import orientation_error_deg
from screen_direct_work_grip_20261112 import transition

OUT = ROOT / 'right_hand_rotation_20261112/stage3_preparation/opposed_contact_v1'


def main():
    if OUT.exists():
        raise RuntimeError('Preserve completed geometry search')
    OUT.mkdir(parents=True)
    (OUT / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    source = ROOT / 'right_hand_rotation_20261112/stage2_work_grip/loaded_models/gain1/mu0.2_end_state.npz'
    m, initial, _, _, provenance = assets('active_ccd50'); m.geom_friction[:, 0] = .2
    z = dict(np.load(source, allow_pickle=False)); d = restore(m, z); g = Guard(m); cube = m.geom('cube').id
    C, B = d.geom_xpos[cube].copy(), d.geom_xmat[cube].reshape(3, 3).copy()
    sid = np.asarray(SIM_TO_MOTOR_IDS)-1; lag = d.ctrl-d.qpos[:16]
    low = np.maximum(g.qlo, (np.ceil(g.lo[sid])+2-2048)*TICK-lag)
    high = np.minimum(g.qhi, (np.floor(g.hi[sid])-2-2048)*TICK-lag)
    geometry = {i: v for i, v, _ in precompute(m, g.hand)}
    rows = []; fits = {}; began = time.perf_counter()
    for axis, middle_sign in ((0, 1), (0, -1), (2, -1), (2, 1)):
        profile = f'axis{axis}_middle{middle_sign:+d}'
        for finger, active, sign in (('mf', np.arange(4, 8), middle_sign), ('th', np.arange(12, 16), -middle_sign)):
            tip = m.geom(finger+'_tip').id
            hull = ConvexHull(geometry[tip]); normals, offsets = hull.equations[:, :3], -hull.equations[:, 3]
            local = np.zeros(3); local[axis] = sign*27.45; local[1] = 10.; local[2 if axis == 0 else 0] = 12.
            target = C + B@local/1000; normal = sign*B[:, axis]; proposal = restore(m, z)

            def measure(x):
                proposal.qpos[:] = z['qpos']; proposal.qpos[active] = x
                proposal.ctrl[:] = proposal.qpos[:16]+lag; mujoco.mj_forward(m, proposal)
                R = proposal.geom_xmat[tip].reshape(3, 3)
                planes = normals@(R.T@(target-proposal.geom_xpos[tip]))-offsets
                j = int(np.argmax(planes)); support, own = g.contacts(proposal)
                penetration = max((-float(c.dist)*1000 for c in proposal.contact if cube in (int(c.geom1), int(c.geom2))), default=0.)
                return float(planes[j]*1000), float(np.linalg.norm(R@normals[j]+normal)), support, own, penetration

            def objective(x):
                gap, normal_error, support, own, penetration = measure(x)
                return abs(gap+.025) + normal_error*2 + 1500*max(own.values(), default=0.) + 1000*max(support.values(), default=0.) + 500*max(0., penetration-.8)

            rng = np.random.default_rng(1190+axis+int(active[0])+middle_sign)
            population = rng.uniform(low[active], high[active], (32, 4))
            population[0] = np.clip(d.qpos[active], low[active], high[active])
            fit = differential_evolution(objective, list(zip(low[active], high[active])), init=population,
                                         seed=axis+int(active[0])+20+middle_sign, maxiter=100, tol=1e-6, polish=True, workers=1)
            gap, normal_error, support, own, penetration = measure(fit.x)
            contacts = [c for c in proposal.contact if {int(c.geom1), int(c.geom2)} == {cube, tip} and c.dist < .0002]
            aligned = [c for c in contacts if (c.frame[:3]*(1 if int(c.geom1) == cube else -1))@normal > .7]
            goal = motor(proposal.ctrl)
            valid = bool(aligned and max(own.values(), default=0.) <= 1e-6 and max(support.values(), default=0.) <= .15
                         and penetration <= 1 and np.all(goal >= g.lo) and np.all(goal <= g.hi))
            r = {'profile': profile, 'finger': finger, 'static_contact_candidate': valid, 'nfev': int(fit.nfev),
                 'target_cube_frame_mm': local.tolist(), 'plane_gap_mm': gap, 'normal_error': normal_error,
                 'self_pairs_mm': own, 'cube_penetration_mm': penetration, 'goal_ticks': goal.tolist()}
            rows.append(r)
            if valid:
                fits[(profile, finger)] = (proposal.qpos.copy(), proposal.ctrl.copy())
                np.savez_compressed(OUT / (profile+'_'+finger+'.npz'), qpos=proposal.qpos, ctrl=proposal.ctrl)
            save_json(OUT / 'geometry_progress.json', {'rows': rows})
            print(json.dumps({k: r[k] for k in ('profile', 'finger', 'static_contact_candidate', 'plane_gap_mm', 'normal_error')}), flush=True)
    trials = []
    for profile in dict.fromkeys(r['profile'] for r in rows):
        if any((profile, f) not in fits for f in ('mf', 'th')): continue
        proposal = restore(m, z)
        for finger, active in (('mf', np.arange(4, 8)), ('th', np.arange(12, 16))):
            proposal.qpos[active] = fits[(profile, finger)][0][active]
        proposal.ctrl[:] = proposal.qpos[:16]+lag; mujoco.mj_forward(m, proposal)
        _, own = g.contacts(proposal)
        if max(own.values(), default=0.) > 1e-6: continue
        ctrl = proposal.ctrl.copy()
        for finger, active in (('mf', np.arange(4, 8)), ('th', np.arange(12, 16))):
            tip = m.geom(finger+'_tip').id
            contacts = [c for c in proposal.contact if {int(c.geom1), int(c.geom2)} == {cube, tip}]
            if not contacts: break
            c = min(contacts, key=lambda c: c.dist)
            normal = c.frame[:3]*(1 if int(c.geom1) == cube else -1)
            jac = np.zeros((3, m.nv)); mujoco.mj_jac(m, proposal, jac, None, c.pos, int(m.geom_bodyid[tip]))
            ctrl[active] += (-jac[:, :16].T@normal/m.actuator_gainprm[:, 0])[active]*.4
        target = motor(ctrl)
        if np.any(target < g.lo) or np.any(target > g.hi): continue
        for order in ('all', 'MF_TH_RF_IF', 'TH_MF_RF_IF'):
            data = restore(m, z); guard = Guard(m); moves = transition(motor(data.ctrl), target, order)
            goals = np.vstack([moves, np.tile(target, (54, 1))]); stop = []; count = supported = mf = th = 0
            C0 = B0 = None; shift = drift = 0.
            for i, command in enumerate(goals):
                data.ctrl[:] = ticks_to_sim(command)
                for _ in range(75):
                    mujoco.mj_step(m, data); stop = guard.check(data)
                    if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center']) > .08: stop = list(stop)+['cube_translation_over80mm']
                    if i >= len(moves):
                        if C0 is None: C0, B0 = data.geom_xpos[cube].copy(), data.geom_xmat[cube].reshape(3, 3).copy()
                        load = sample_model(m, data, guard.hand, cube); count += 1; supported += load['supported_sample']
                        forces = {f: sum(v['normal_N'] for n, v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(n).id])).name or '').startswith(f+'_')) for f in ('mf', 'th')}
                        mf += forces['mf'] > .1; th += forces['th'] > .1
                        shift = max(shift, float(np.linalg.norm(data.geom_xpos[cube]-C0)*1000))
                        drift = max(drift, orientation_error_deg(data.geom_xmat[cube].reshape(3, 3), B0))
                    if stop: break
                if stop: break
            sub = OUT / (profile+'_'+order); sub.mkdir()
            np.savez_compressed(sub / 'research_goals.npz', start=motor(z['ctrl']), goals=goals[:i+1])
            np.savez_compressed(sub / 'end_state.npz', **{k: getattr(data, k) for k in BOUND}, time=data.time)
            r = {'profile': profile, 'order': order, 'goals': i+1, 'stop': stop, 'hold_samples': count,
                 'support_fraction': supported/max(1, count), 'middle_loaded_fraction': mf/max(1, count),
                 'thumb_loaded_fraction': th/max(1, count), 'hold_shift_mm': shift, 'hold_drift_deg': drift,
                 'yaw_change_deg': angle(data.geom_xmat[cube].reshape(3, 3), B, initial['up']),
                 'paired_contact_passed': bool(not stop and count == 4050 and supported/count > .8 and mf/count > .8 and th/count > .8 and shift < 2 and drift < 3), 'worst': guard.worst}
            save_json(sub / 'review.json', r); trials.append(r)
            print(json.dumps({k: v for k, v in r.items() if k != 'worst'}), flush=True)
    save_json(OUT / 'review.json', {'status': 'stage2_cube_opposed_contact_reacquisition', 'source': source.relative_to(ROOT).as_posix(),
              'source_sha256': sha(source), 'model_provenance': provenance, 'geometry': rows, 'trials': trials,
              'elapsed_seconds': time.perf_counter()-began, 'friction_hypothesis': .2,
              'cube_fixed_only_in_IK_copies': True, 'real_cube_dynamic_state_reconstructed': False,
              'native_guards_and_limits_unchanged': True, 'forces_not_calibrated_to_450mA': True,
              'devices_opened': 0, 'hardware_execution_allowed': False})


if __name__ == '__main__':
    main()
