"""Offline finite-actuator continuation: optimize controls, never the cube state.

The prefix is the reached two-finger grasp/yaw diagnostic. Every rollout starts
from a saved physics state and lets the cube move under gravity and contacts.
No robot modules, ports, targets, or device operations are used.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
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
from hand_motor_mapping import ticks_to_sim, sim_to_hw, TICK
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2'
PREFIX=ROOT/'right_hand_rotation_20261027/reached_pair_contact_yaw/lift4_press0.75.npz'
OUT=ROOT/'right_hand_rotation_20261027/pair_free_yaw_optimization'


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--resume');parser.add_argument('--all-fingers',action='store_true');parser.add_argument('--output-folder',default='pair_free_yaw_optimization_v2');parser.add_argument('--native-margin-ticks',type=int,default=0);parser.add_argument('--self-weight',type=float,default=40.);parser.add_argument('--finish-hold-samples',type=int,default=0);parser.add_argument('--resume-hold-samples',type=int,default=40);parser.add_argument('--self-clearance-mm',type=float,default=0.);parser.add_argument('--cube-penalty-threshold-mm',type=float,default=.8);args=parser.parse_args()
    if not 0<=args.native_margin_ticks<=8 or not 40<=args.self_weight<=2000:raise ValueError('Invalid conservative diagnostic margins')
    if Path(args.output_folder).name!=args.output_folder:raise ValueError('Expected output basename')
    output=OUT.with_name(args.output_folder);prefix=ROOT/args.resume if args.resume else PREFIX;prefix.resolve().relative_to(ROOT)
    began=time.perf_counter();output.mkdir(exist_ok=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    m,_=build(registry.load('LeapCubeFaceUp',config=cfg),output/'scene.xml')
    proof=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    correct_model(m,np.array(proof['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002
    up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world'])
    m.opt.gravity[:]=-9.81*up;cube=m.geom('cube').id;m.geom_friction[cube,0]=.3
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    source=np.load(SOURCE/'nominal_grasp_state.npz');data=np.load(prefix);low,high=limits()
    d=mujoco.MjData(m);d.qpos[:]=source['qpos'];d.qvel[:]=source['qvel'];d.ctrl[:]=source['ctrl']
    for _ in range(1000):mujoco.mj_step(m,d)
    B=d.xmat[m.body('cube').id].reshape(3,3).copy();C=d.qpos[-7:-4].copy()
    # Integer goals must be inside fractional native endpoints. Preserve the
    # source candidate; regenerate its quantized values and replay from the
    # original physics state rather than reusing its reported final angles.
    original_goals=data['goals'];regenerated=np.clip(original_goals,np.ceil(low)+args.native_margin_ticks,np.floor(high)-args.native_margin_ticks).astype(int)
    rounding_corrections=int(np.count_nonzero(regenerated!=original_goals))
    goals=list(regenerated)+[regenerated[-1].copy() for _ in range(args.resume_hold_samples)]
    if np.any(abs(np.diff(np.vstack([data['start'],goals]),axis=0))>2):raise ValueError('Regenerated prefix exceeds 2 ticks')
    steps=0
    for i,q in enumerate(goals,1):
        d.ctrl[:]=ticks_to_sim(q);target=round(i*.075/.002)
        for _ in range(target-steps):mujoco.mj_step(m,d)
        steps=target
    kind=mujoco.mjtState.mjSTATE_INTEGRATION
    size=mujoco.mj_stateSize(m,kind)
    def save_state():
        value=np.empty(size);mujoco.mj_getState(m,d,value,kind);return value
    def restore(value):
        mujoco.mj_setState(m,d,value,kind);mujoco.mj_forward(m,d)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support)
    clearance_pair=(m.geom('mf_tip').id,m.geom('palm_collision_6').id)
    ids=np.arange(16) if args.all_fingers else np.array([0,1,2,3,8,9,10,11]);previous=regenerated[-1].astype(float);accepted_state=save_state()
    angles=list(data['desired_angles'])+[float(data['desired_angles'][-1]) for _ in range(args.resume_hold_samples)];log=[];rng=np.random.default_rng(20261027)
    def orientation():
        R=d.xmat[m.body('cube').id].reshape(3,3)
        delta=Rotation.from_matrix(R@B.T).as_quat()
        yaw=float(np.degrees(2*np.arctan2(delta[:3]@up,delta[3])))
        tilt=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1,0]))@(B@np.array([0.,-1,0])),-1,1))))
        return yaw,tilt,R
    start_yaw=orientation()[0]
    print(json.dumps(dict(stage='free_continuation_start',yaw_deg=start_yaw,source_quantized_goals_regenerated=rounding_corrections,moving_motor_ids=(ids+1).tolist())),flush=True)
    evaluations=0
    for iteration in range(15):
        restore(accepted_state);yaw_before,_,_=orientation();desired=min(93.,yaw_before+7.)
        targetR=Rotation.from_rotvec(up*np.radians(desired)).as_matrix()@B
        startC=d.qpos[-7:-4].copy();n_goals=60
        lower=np.maximum(np.ceil(low[ids])+args.native_margin_ticks,previous[ids]-100)+.001
        upper=np.minimum(np.floor(high[ids])-args.native_margin_ticks,previous[ids]+100)-.001
        def rollout(x,rounded=False,full_check=False):
            nonlocal evaluations
            evaluations+=1;restore(accepted_state);goal=previous.astype(float).copy();goal[ids]=x
            if rounded:goal=np.rint(goal)
            path=np.array([previous+(goal-previous)*k/n_goals for k in range(1,n_goals+1)])
            if rounded:path=np.rint(path)
            if args.finish_hold_samples:path=np.vstack([path,np.repeat(path[-1][None,:],args.finish_hold_samples,axis=0)])
            spmax=sumax=cpmax=trackmax=range_excess=nohand=longest=0.;minimum_gap=1e9;external=False;substeps=0
            for sample,q in enumerate(path,1):
                d.ctrl[:]=ticks_to_sim(q);target=round(sample*.075/.002)
                for step in range(target-substeps):
                    mujoco.mj_step(m,d)
                    if not full_check and step!=target-substeps-1:continue
                    su,sp=inspect(d);spmax=max(spmax,max(sp.values(),default=0.));sumax=max(sumax,max(su.values(),default=0.))
                    if args.self_clearance_mm:minimum_gap=min(minimum_gap,float(mujoco.mj_geomDistance(m,d,*clearance_pair,1.,None)*1000))
                    trackmax=max(trackmax,float(max(abs(d.qpos[:16]-d.ctrl))))
                    raw=sim_to_hw(d.qpos[:16])/TICK;range_excess=max(range_excess,float(np.max(np.maximum(low-raw,raw-high))))
                    touching=False
                    for c in d.contact:
                        a,b=int(c.geom1),int(c.geom2)
                        if cube not in (a,b):continue
                        other=b if a==cube else a
                        if int(m.geom_bodyid[other]) in hand:
                            cpmax=max(cpmax,max(0.,-float(c.dist)*1000));touching|=c.dist<.0004
                        elif other in support and c.dist<.0004:external=True
                    increment=.002 if full_check else .075
                    nohand=0. if touching else nohand+increment;longest=max(longest,nohand)
                substeps=target
            mujoco.mj_forward(m,d);yaw,tilt,R=orientation();shift=float(np.linalg.norm(d.qpos[-7:-4]-C)*1000)
            meta=dict(yaw_deg=yaw,tilt_change_deg=tilt,self_mm=spmax,support_mm=sumax,cube_mm=cpmax,max_tracking_rad=trackmax,
                      actual_native_excess_ticks=range_excess,center_shift_mm=shift,no_hand_contact_seconds=longest,external_cube_contact=external,minimum_middle_tip_palm_gap_mm=minimum_gap if args.self_clearance_mm else None)
            return path,meta,R,d.qpos[-7:-4].copy()
        def objective(x):
            _,r,R,center=rollout(x)
            rot_error=Rotation.from_matrix(R@targetR.T).as_rotvec()*30
            yaw_error=(r['yaw_deg']-desired)*np.pi/180*60
            return np.r_[rot_error*.5,yaw_error,(center-startC)*1000*.08,
                         args.self_weight*r['self_mm'],40*max(0,r['support_mm']-.1),100*max(0,r['cube_mm']-args.cube_penalty_threshold_mm),
                         10*max(0,args.self_clearance_mm-r['minimum_middle_tip_palm_gap_mm']) if args.self_clearance_mm else 0.,
                         100*max(0,r['max_tracking_rad']-.15),5*max(0,r['actual_native_excess_ticks']-.5),
                         100*max(0,r['no_hand_contact_seconds']-.15),200*float(r['external_cube_contact']),
                         .003*(x-previous[ids])]
        options=[]
        for attempt in range(2):
            initial=previous[ids].copy()
            if attempt:initial+=rng.normal(0,30,len(ids))
            fit=least_squares(objective,np.clip(initial,lower,upper),bounds=(lower,upper),diff_step=.0002,max_nfev=30,ftol=.002,xtol=.0001)
            path,row,_,_=rollout(fit.x,rounded=True,full_check=True)
            row.update(attempt=attempt,goal=path[-1].astype(int).tolist(),target_yaw_deg=desired)
            row['passed']=bool(row['self_mm']<=1e-6 and row['support_mm']<=.15 and row['cube_mm']<=1
                               and row['max_tracking_rad']<=.18 and row['actual_native_excess_ticks']<=1
                               and row['center_shift_mm']<=80 and row['no_hand_contact_seconds']<=.25
                               and not row['external_cube_contact'] and row['tilt_change_deg']<=15 and row['yaw_deg']>yaw_before+.5
                               and np.all(path>=low) and np.all(path<=high) and np.max(abs(np.diff(np.vstack([previous,path]),axis=0)))<=2)
            options.append((row,path.copy(),save_state()))
            print(json.dumps(dict(iteration=iteration,**row)),flush=True)
            if row['passed'] and row['yaw_deg']>=desired-1.:break
        valid=[v for v in options if v[0]['passed']]
        if not valid:
            log.append(dict(iteration=iteration,rejected=[v[0] for v in options]));break
        row,path,accepted_state=max(valid,key=lambda item:item[0]['yaw_deg']);goals.extend(path.astype(int));angles.extend([desired]*len(path));previous=path[-1].astype(float);log.append(row)
        np.savez_compressed(output/'candidate.npz',start=data['start'],goals=np.asarray(goals),desired_angles=np.asarray(angles),bias_ticks=data['bias_ticks'])
        report=dict(status='free_cube_control_optimization_diagnostic',start_yaw_deg=start_yaw,latest_nominal_yaw_deg=row['yaw_deg'],steps=log,evaluations=evaluations,
                    goals=len(goals),native_force_limits_retained=True,cube_pose_never_forced_during_rollout=True,robustness_not_checked=True,
                    hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
        (output/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        if row['yaw_deg']>=89.:break
    restore(accepted_state);yaw,tilt,_=orientation()
    report=dict(status='free_cube_control_optimization_diagnostic',source_candidate=prefix.relative_to(ROOT).as_posix(),source_quantized_goals_regenerated=rounding_corrections,native_goal_margin_ticks=args.native_margin_ticks,self_penalty_weight=args.self_weight,finish_hold_samples=args.finish_hold_samples,middle_tip_palm_clearance_target_mm=args.self_clearance_mm,cube_overlap_soft_penalty_starts_mm=args.cube_penalty_threshold_mm,start_yaw_deg=start_yaw,latest_nominal_yaw_deg=yaw,tilt_change_deg=tilt,steps=log,evaluations=evaluations,
                goals=len(goals),native_force_limits_retained=True,cube_pose_never_forced_during_rollout=True,robustness_not_checked=True,
                hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
    (output/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='steps'}),flush=True)


if __name__=='__main__':main()
