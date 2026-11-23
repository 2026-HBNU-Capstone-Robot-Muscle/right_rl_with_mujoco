"""Bounded, offline cube-placement hypothesis with the actual measured hand fixed.

No pose measurement, motion authorization, old-grip substitution or model edits.
"""
from pathlib import Path
import hashlib, json, time
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim

ROOT = Path(__file__).resolve().parent
BASE = ROOT/'right_hand_rotation_20261117/user_measured_grip'
MODEL = ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
SOURCE = ROOT/'measured_grip_view_20261117_175648_673729/result.json'


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    observation = json.loads(SOURCE.read_bytes())
    m = mujoco.MjModel.from_binary_path('model.mjb', assets={'model.mjb': MODEL.read_bytes()})
    d = mujoco.MjData(m)
    d.qpos[:16] = ticks_to_sim(observation['after']['actual_ticks'])
    d.ctrl[:] = ticks_to_sim(observation['after']['goal_ticks'])
    mujoco.mj_forward(m, d)
    palm, cube = m.body('palm').id, m.geom('cube').id
    P, R = d.xpos[palm].copy(), d.xmat[palm].reshape(3,3).copy()
    hand = {palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:
            hand.add(b)
    explicit = {int(b if a == cube else a) for a,b in zip(m.pair_geom1,m.pair_geom2) if cube in (int(a),int(b))}
    geoms = [g for g in range(m.ngeom) if int(m.geom_bodyid[g]) in hand and
             (g in explicit or int(m.geom_contype[g]) & int(m.geom_conaffinity[cube]) or
              int(m.geom_contype[cube]) & int(m.geom_conaffinity[g]))]
    groups = {role:np.array([i for i,g in enumerate(geoms) if
              (m.body(int(m.geom_bodyid[g])).name or '').startswith(role)],dtype=int)
              for role in ('if_', 'mf_', 'rf_', 'th_', 'palm')}
    def geometry(x):
        B = R @ Rotation.from_euler('xyz',[x[4],x[5],x[3]],degrees=True).as_matrix()
        q = Rotation.from_matrix(B).as_quat()
        d.qpos[-7:-4] = P+R@(x[:3]/1000)
        d.qpos[-4:] = np.r_[q[3],q[:3]]
        mujoco.mj_fwdPosition(m,d)
        distances = np.array([mujoco.mj_geomDistance(m,d,g,cube,.15,None)*1000 for g in geoms])
        nearest = {role:float(np.min(distances[idx])) for role,idx in groups.items()}
        return distances, nearest
    def residual(x):
        distances, near = geometry(x)
        return np.r_[np.maximum(-distances-.3,0)*30,
            [(near['th_']+.02)*3,(near['mf_']+.02)*3,(near['palm']+.02)*3],
            (x[:3]-[-42,-25,-62])*.025, x[4:]*.01]
    began=time.perf_counter();cases=[]
    for yaw in (0,15,30,45,60,75):
        if time.perf_counter()-began>50:break
        fit=least_squares(residual,[-42,-25,-62,yaw,1.,1.],
                          bounds=([-60,-65,-85,-5,-15,-15],[-25,10,-50,95,15,15]),
                          max_nfev=180,diff_step=1e-4,ftol=1e-7,xtol=1e-7,gtol=1e-7)
        distances,near=geometry(fit.x)
        ranked=sorted(zip(geoms,distances),key=lambda row:row[1])
        cases.append(dict(center_in_palm_mm=fit.x[:3].tolist(),yaw_in_palm_degrees=float(fit.x[3]),
            roll_pitch_in_palm_degrees=fit.x[4:].tolist(),
            cost=float(np.dot(fit.fun,fit.fun)),nearest_group_gap_mm=near,
            max_cube_penetration_mm=float(max(0,-distances.min())),
            nearest_geometry=[dict(name=m.geom(g).name,gap_mm=float(dist)) for g,dist in ranked[:10]]))
    cases.sort(key=lambda r:r['cost'])
    best=cases[0]
    good=best['max_cube_penetration_mm']<=.5 and all(abs(best['nearest_group_gap_mm'][k])<=.5 for k in ('th_','mf_','palm'))
    report=dict(kind='static_cube_placement_hypothesis_with_actual_hand_fixed',model_sha256=sha(MODEL),
        observation_sha256=sha(SOURCE),script_sha256=sha(Path(__file__)),cases=cases,
        static_hypothesis_found=good,elapsed_seconds=time.perf_counter()-began,
        hand_angles_changed=False,cube_55mm=True,cube_pose_measured=False,
        palm_seating_and_near_upright_cube_are_search_assumptions=True,
        dynamic_or_hardware_execution_approved=False,model_modified=False,
        original_measured_grip_remains_reference=True)
    out=BASE/'cube_placement_hypothesis';out.mkdir(exist_ok=True)
    (out/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(static_hypothesis_found=good,best=best,elapsed_seconds=report['elapsed_seconds'])))


if __name__=='__main__':
    main()
