"""Offline measured-hand geometry and hypothetical free-cube replay.

The path consists of two measured EMPTY runs. Prescribing those measured joint
positions under a hypothetical load is a diagnostic assumption, not identified
loaded motor dynamics or a certificate for hardware execution.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,hashlib
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
from hand_motor_mapping import ticks_to_sim
from observed_book_model_policy import classify_self_contacts
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/completed_empty_path_review_20261025'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--geometry-only',action='store_true');parser.add_argument('--kinematic',action='store_true');args=parser.parse_args()
    trace=np.load(OUT/'goal_indexed_actual_path.npz');actual=trace['actual_ticks'];q=ticks_to_sim(actual)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'measured_trace_scene.xml');set_cube_physics(m,.055,.1)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    cube=m.geom('cube').id;d=mujoco.MjData(m)
    original_armature=m.dof_armature[:16].copy()
    if args.kinematic:m.dof_armature[:16]=1e6
    suffix='_kinematic_v2' if args.kinematic else ''
    def contacts(data):
        pairs={};support={};cubehand=0.;touch=False;external=False
        for c in data.contact:
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if c.dist<0 and (ha and hb or a in supports and hb or b in supports and ha):
                pair='|'.join(sorted((m.geom(a).name,m.geom(b).name)));target=pairs if ha and hb else support
                target[pair]=max(target.get(pair,0.),-float(c.dist)*1000)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cubehand=max(cubehand,-float(c.dist)*1000);touch=touch or c.dist<.0003
                if c.dist<.0003 and (other in supports or m.geom(other).name=='floor'):external=True
        return pairs,support,cubehand,touch,external
    geometry=[];maxima={};supportmax={}
    raw_actual=trace['actual_raw_ticks'];allq=ticks_to_sim(raw_actual)
    for i,pose in enumerate(allq):
        d.qpos[:16]=pose;d.qvel[:]=0;d.qpos[-7:]=[10,10,10,1,0,0,0];mujoco.mj_forward(m,d)
        pairs,support,_,_,_=contacts(d)
        for values,dest in ((pairs,maxima),(support,supportmax)):
            for pair,depth in values.items():
                if depth>dest.get(pair,{}).get('depth_mm',0):dest[pair]=dict(depth_mm=depth,measured_record=i)
        classified=classify_self_contacts(pairs)
        if classified or support:geometry.append(dict(measured_record=i,raw_self_contacts_mm=pairs,classified_self_contacts_mm=classified,support_contacts_mm=support))
    result=dict(model=manifest,raw_actual_records=len(allq),raw_actual_self_maxima=maxima,raw_actual_support_maxima=supportmax,
        geometry_contact_records=geometry,source_npz_sha256=hashlib.sha256((OUT/'goal_indexed_actual_path.npz').read_bytes()).hexdigest(),
        actual_samples_separate_runs=True,continuous_actual_execution_verified=False,loaded_motor_response_verified=False,
        hardware_execution_allowed=False,hardware_commands_sent=0,diagnostic_only=True,cases=[],
        trajectory_boundary_method=('Prescribed qpos/qvel, infinite effective hand inertia limit, not loaded actuator identification' if args.kinematic else 'Finite hand inertia with direct pose resets; not a faithful prescribed-motion boundary'),
        original_hand_armature=original_armature.tolist(),diagnostic_hand_armature=m.dof_armature[:16].tolist())
    (OUT/('actual_geometry_review'+suffix+'.json')).write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('MEASURED_GEOMETRY',json.dumps({k:result[k] for k in ('raw_actual_records','raw_actual_self_maxima','raw_actual_support_maxima')}),flush=True)
    if args.geometry_only:return
    old=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_clearance/reverse_original_native_verification.json').read_bytes())
    ref=next(r for r in old['runs'] if r.get('mode')=='moving_path' and r['gain_scale']==1 and r['friction']==.8 and r['offset_from_nominal_mm']==[0,0])
    step=float(m.opt.timestep);period=.05;grid=np.arange(len(q))*period
    cube0=np.array(ref['initial_cube_qpos']);dur=grid[-1];hold=4.;target=target_quaternion(cube0[-4:])
    path_times=np.arange(0,dur+hold+step/2,step)
    qs=np.column_stack([np.interp(path_times,grid,q[:,j]) for j in range(16)])
    velocities=np.gradient(qs,step,axis=0)
    for friction in (.4,.8,1.2):
      for mode in ('measured_segment_splice','stationary_control'):
        d=mujoco.MjData(m);m.geom_friction[cube,0]=friction;d.qpos[:16]=q[0];d.qpos[-7:]=cube0;d.ctrl[:]=q[0]
        mujoco.mj_forward(m,d);raise_mm=0.
        while contacts(d)[2]>.05 and raise_mm<15:
            d.qpos[-5]+=.001;raise_mm+=1;mujoco.mj_forward(m,d)
        start_penetration=contacts(d)[2]
        if start_penetration>.05:raise RuntimeError('No valid free-cube starting clearance')
        for _ in range(round(1/step)):
            d.qpos[:16]=q[0];d.qvel[:16]=0;d.ctrl[:]=q[0];mujoco.mj_step(m,d)
        settled=d.qpos[-7:].copy();start_drift=float(np.degrees(Rotation.from_matrix(Rotation.from_quat(settled[-4:][[1,2,3,0]]).as_matrix()@Rotation.from_quat(cube0[-4:][[1,2,3,0]]).as_matrix().T).magnitude()))
        poses=[];errors=[];maxself=0.;maxsupport=0.;gap=maxgap=0.;external=False;contact_end=False
        for i,t in enumerate(path_times):
            pose=q[0] if mode=='stationary_control' else qs[i];vel=np.zeros(16) if mode=='stationary_control' else velocities[i]
            d.qpos[:16]=pose;d.qvel[:16]=vel;d.ctrl[:]=pose;mujoco.mj_forward(m,d)
            pairs,support,_,touch,other=contacts(d)
            maxself=max(maxself,max(pairs.values(),default=0.));maxsupport=max(maxsupport,max(support.values(),default=0.))
            gap=0. if touch else gap+step;maxgap=max(maxgap,gap);external=external or other;contact_end=touch
            poses.append(d.qpos.copy());errors.append(float(np.degrees(2*np.arccos(np.clip(abs(d.qpos[-4:]@target)/np.linalg.norm(d.qpos[-4:]),0,1)))))
            mujoco.mj_step(m,d)
        poses=np.asarray(poses);errors=np.asarray(errors);last=poses[-1,-4:]
        whole=float(np.degrees(Rotation.from_matrix(Rotation.from_quat(last[[1,2,3,0]]).as_matrix()@Rotation.from_quat(cube0[-4:][[1,2,3,0]]).as_matrix().T).magnitude()))
        row=dict(mode=mode,friction=friction,start_lift_mm=raise_mm,starting_penetration_mm=start_penetration,
            settle_orientation_drift_deg=start_drift,final_target_error_deg=float(errors[-1]),hold_maximum_target_error_deg=float(errors[path_times>=dur].max()),
            whole_cube_orientation_change_deg=whole,maximum_self_overlap_mm=maxself,maximum_support_overlap_mm=maxsupport,
            maximum_no_hand_contact_seconds=maxgap,external_cube_contact=external,hand_contact_at_end=contact_end,
            source_initial_cube_qpos=cube0.tolist(),settled_cube_qpos=settled.tolist(),final_cube_qpos=poses[-1,-7:].tolist(),
            actual_empty_hand_prescribed_under_hypothetical_load=True,loaded_response_measured=False)
        result['cases'].append(row);print('MEASURED_CUBE',json.dumps(row),flush=True)
        if mode!='stationary_control' and friction==.8:
            np.savez_compressed(OUT/('measured_nominal_cube_replay'+suffix+'.npz'),qpos=poses,times=path_times,goal_errors_deg=errors,source_goal_ticks=trace['goal_ticks'],actual_empty_ticks=actual)
    (OUT/('actual_geometry_and_cube_replay'+suffix+'.json')).write_text(json.dumps(result,indent=2),encoding='utf-8')

if __name__=='__main__':main()
