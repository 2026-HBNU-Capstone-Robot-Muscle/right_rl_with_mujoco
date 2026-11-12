"""IK contact-location retargeting of a demonstrated path; offline candidate only.

This does not prescribe a cube trajectory or edit the demonstrated reference.
Its separate physics replay retains all real contact and actuator forces.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse, hashlib, json, time
from pathlib import Path
from collections import Counter
import numpy as np
import mujoco
from scipy.optimize import least_squares
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim, sim_to_hw, SIM_TO_MOTOR_IDS, TICK
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model
from calibrated_quarter_plan_20261028 import LOADED_PATH
from simulate_handoff_yaw_continuation_20261028 import make_seeds,OUT

ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--delta-source',choices=['model_transfer','image_relative'],default='model_transfer');args=parser.parse_args()
    out=OUT/('retarget_'+args.delta_source);out.mkdir(parents=True,exist_ok=True);began=time.perf_counter()
    z=np.load(LOADED_PATH);start=z['start'];original=z['goals'];phases=z['phases']
    seeds,seed_meta=make_seeds();source='retained_model_transfer' if args.delta_source=='model_transfer' else 'retained_image_relative'
    delta=seeds[source][:3]-seeds['original_yaw_seed'][:3]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    m,manifest=build(registry.load('LeapCubeFaceUp',config=cfg),out/'ik_scene.xml',book_projection_mm=-10.)
    correct_model(m,start)
    cube=m.geom_bodyid==m.body('cube').id;m.geom_contype[cube]=0;m.geom_conaffinity[cube]=0
    d=mujoco.MjData(m);d.qpos[-7:]=[0,0,2,1,0,0,0]
    tips=[m.geom(n+'_tip').id for n in ('if','mf','rf','th')]
    lo,hi=limits();ids=np.asarray(SIM_TO_MOTOR_IDS)-1
    lower=(np.ceil(lo[ids])-2048)*TICK+1e-7;upper=(np.floor(hi[ids])-2048)*TICK-1e-7
    cache={};reports=[]
    def retarget(q):
        key=tuple(int(v) for v in q)
        if key in cache:return cache[key].copy()
        reference=ticks_to_sim(q);d.qpos[:16]=reference;mujoco.mj_forward(m,d)
        positions=d.geom_xpos[tips].copy()+delta;rotations=d.geom_xmat[tips].reshape(4,3,3).copy()
        def residual(x):
            d.qpos[:16]=x;mujoco.mj_forward(m,d)
            return np.r_[(d.geom_xpos[tips]-positions).ravel(),
                         .004*(d.geom_xmat[tips].reshape(4,3,3)-rotations).ravel(),.001*(x-reference)]
        def jacobian(x):
            d.qpos[:16]=x;mujoco.mj_forward(m,d)
            pos=[];rot=[]
            for g in tips:
                jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mujoco.mj_jacGeom(m,d,jp,jr,g)
                pos.append(jp[:,:16]);R=d.geom_xmat[g].reshape(3,3)
                dr=np.zeros((3,3,16))
                for axis in range(3):dr[:,axis,:]=np.cross(jr[:,:16].T,R[:,axis]).T
                rot.append(.004*dr.reshape(9,16))
            return np.vstack([*pos,*rot,.001*np.eye(16)])
        fit=least_squares(residual,np.clip(reference,lower,upper),jac=jacobian,bounds=(lower,upper),
            max_nfev=60,ftol=1e-7,xtol=1e-7,gtol=1e-7)
        d.qpos[:16]=fit.x;mujoco.mj_forward(m,d)
        errors=np.linalg.norm(d.geom_xpos[tips]-positions,axis=1)*1000
        goal=np.rint(sim_to_hw(fit.x)/TICK).astype(int)
        assert np.all(goal>=lo) and np.all(goal<=hi)
        cache[key]=goal
        reports.append(dict(source_goal=list(key),retarget_goal=goal.tolist(),tip_errors_mm=errors.tolist(),evaluations=fit.nfev,
            maximum_motor_delta_ticks=int(max(abs(goal-q)))))
        return goal.copy()
    keys={0,len(original)-1}
    keys.update(range(0,len(original),16))
    keys.update(i for i in range(1,len(phases)) if phases[i]!=phases[i-1])
    keys.update(i-1 for i in range(1,len(phases)) if phases[i]!=phases[i-1])
    rows=[];labels=[];previous=start.copy()
    def add(target,count,phase):
        nonlocal previous
        count=max(count,1,int(np.ceil(max(abs(target-previous))/2)))
        rows.extend(np.rint([previous+(target-previous)*i/count for i in range(1,count+1)]).astype(int))
        labels.extend([str(phase)]*count);previous=target.copy()
    add(retarget(start),1,'retarget_open')
    last=-1
    for k in sorted(keys):
        add(retarget(original[k]),k-last,phases[k]);last=k
    rows=np.asarray(rows,dtype=int)
    assert np.max(abs(np.diff(np.vstack([start,rows]),axis=0)))<=2
    assert np.all(rows>=lo) and np.all(rows<=hi)
    np.savez_compressed(out/'candidate.npz',start=start,goals=rows,phases=np.asarray(labels),sample_period_s=.075)
    report=dict(status='offline_contact_location_IK_retarget_candidate',delta_source=args.delta_source,delta_world_mm=(delta*1000).tolist(),
        source_path_sha256=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest(),original_targets_unchanged=True,
        candidate_goals=len(rows),phase_counts=dict(Counter(labels)),nominal_seconds=len(rows)*.075,
        key_solutions=len(reports),maximum_tip_error_mm=max(max(v['tip_errors_mm']) for v in reports),
        median_tip_error_mm=float(np.median([v['tip_errors_mm'] for v in reports])),solutions=reports,
        native_ranges_preserved=True,max_goal_step_ticks=2,cube_disabled_only_for_IK=True,
        dynamics_validation_pending=True,real_cube_metric_registration_verified=False,
        hardware_execution_allowed=False,devices_opened=False,seed_metadata=seed_meta,model=manifest,
        elapsed_seconds=time.perf_counter()-began)
    (out/'ik_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('solutions','model','seed_metadata')}),flush=True)

if __name__=='__main__':main()
