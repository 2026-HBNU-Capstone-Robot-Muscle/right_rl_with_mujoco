"""Offline contact chronology for the conditional native quarter turn."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco,mujoco.rollout
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
from hand_motor_mapping import ticks_to_sim
from observed_book_model_policy import classify_self_contacts
ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261024/original_sequence_clearance'
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_loaded_model'

def main():
    OUT.mkdir(exist_ok=True)
    raw=np.array(json.loads((BASE/'CAD_adapted_forward_reference_path_v2.json').read_bytes())['dense_goal_ticks'])[::-1]
    q=ticks_to_sim(raw)
    verification=json.loads((BASE/'reverse_original_native_verification.json').read_bytes())
    ref=next(r for r in verification['runs'] if r.get('mode')=='moving_path' and r['gain_scale']==1 and r['friction']==.8 and r['offset_from_nominal_mm']==[0,0])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'scene.xml');set_cube_physics(m,.055,.1);cube=m.geom('cube').id;m.geom_friction[cube,0]=.8
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    kind=mujoco.mjtState.mjSTATE_FULLPHYSICS;initial=np.array(ref['initial_state']);substeps=round(.05/m.opt.timestep)
    commands=np.r_[q[1:],np.repeat(q[-1:],80,axis=0)];ctrl=np.repeat(commands,substeps,axis=0)
    with mujoco.rollout.Rollout(nthread=1) as pool:
        ss,_=pool.rollout(m,[mujoco.MjData(m)],initial[None],ctrl[None])
    states=ss[0];poses=states[:,1:1+m.nq];target=target_quaternion(np.array(ref['initial_cube_qpos'])[-4:])
    angles=2*np.arccos(np.clip(abs(poses[:,-4:]@target)/np.maximum(np.linalg.norm(poses[:,-4:],axis=1),1e-12),0,1))
    d=mujoco.MjData(m);first=None;maxima={};clear_stops=[];classified_trace=[]
    for i,state in enumerate(states):
        mujoco.mj_setState(m,d,state,kind);d.ctrl[:]=ctrl[i];mujoco.mj_forward(m,d)
        pairs={}
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if c.dist<0 and int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
                pair='|'.join(sorted((m.geom(a).name,m.geom(b).name)));depth=-float(c.dist)*1000
                pairs[pair]=max(pairs.get(pair,0.),depth)
                if depth>maxima.get(pair,{}).get('depth_mm',0):maxima[pair]=dict(depth_mm=depth,time_seconds=(i+1)*m.opt.timestep,goal_index=i//substeps,actual_angles=d.qpos[:16].tolist())
        classified=classify_self_contacts(pairs);worst=max(classified.values(),default=0.);classified_trace.append(worst)
        if first is None and worst>1e-6:first=dict(time_seconds=(i+1)*m.opt.timestep,goal_index=i//substeps,pairs=classified,target_error_deg=float(np.degrees(angles[i])))
        if i%round(1/m.opt.timestep)==0 and angles[i]<np.deg2rad(15) and first is None:clear_stops.append(i)
    observed=json.loads((ROOT/'reverse_start_placement_20261025_075405_164917/result.json').read_bytes())['placement_position_ticks']
    result=dict(model=manifest,source_native_initial_condition=ref['initial_cube_qpos'],physical_cube_registration_verified=False,
        native_initial_hand_difference_from_observation_ticks=np.round((ticks_to_sim(np.array(observed))-initial[1:17])*(4096/(2*np.pi)),3).tolist(),
        first_classified_self_contact=first,maximum_raw_self_contacts=maxima,
        first_inside_goal_seconds=(float((np.flatnonzero(angles<np.deg2rad(15))[0]+1)*m.opt.timestep) if np.any(angles<np.deg2rad(15)) else None),
        clear_candidate_stop_state_indices=clear_stops,final_target_error_deg=float(np.degrees(angles[-1])),
        hardware_commands_sent=0,hardware_execution_allowed=False)
    np.savez_compressed(OUT/'nominal_trace.npz',states=states,controls=ctrl,goal_ticks=raw,errors_rad=angles,classified_self_mm=np.array(classified_trace),initial_state=initial)
    (OUT/'contact_timing.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('model','maximum_raw_self_contacts')},ensure_ascii=False),flush=True)
    print('MAXIMA',json.dumps({k:{a:b for a,b in v.items() if a!='actual_angles'} for k,v in maxima.items()}),flush=True)

if __name__=='__main__':main()
