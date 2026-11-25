"""Offline placement sensitivity of the independently collision-cleared path."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from sac_forward_geometry import measure
from hand_motor_mapping import ticks_to_sim,sim_to_ticks
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/corrected_policy_reference'

def main():
    p=OUT/'projected_policy_reference_v2.json';blob=p.read_bytes();plan=json.loads(blob)
    assert plan['full_static_path_clear']
    path=np.array(plan['projected_goals_sim_rad']);parameters=np.array(plan['sparse_source_parameters'])
    dense=ticks_to_sim(np.array(plan['dense_raw_ticks']))
    schedules=[]
    for factor in (1.,2.):
        times=np.arange(round(parameters[-1]*factor)+1)/factor
        schedules.append(('reference_'+str(factor),np.array([np.interp(times,parameters,path[:,j]) for j in range(16)]).T))
    goals=dense[::4]
    if not np.array_equal(goals[-1],dense[-1]):goals=np.r_[goals,dense[-1:]]
    schedules.append(('8_ticks_max',goals))
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'placement_search_CAD.xml');set_cube_physics(m,.055,.1)
    d=mujoco.MjData(m);cube=m.geom('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    x=np.array([1.,0,0]);x-=up*(x@up);x/=np.linalg.norm(x);y=np.cross(up,x)
    reset=np.array(json.loads((ROOT/'sac_cube55_resets.json').read_bytes())['candidates'][0]['qpos'])
    kind=mujoco.mjtState.mjSTATE_INTEGRATION;size=mujoco.mj_stateSize(m,kind);rows=[]
    def contacts():
        finger=[];external=False
        for c in d.contact:
            if cube in (int(c.geom1),int(c.geom2)) and c.dist<.0003:
                other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
                if int(m.geom_bodyid[other]) in hand:finger.append(m.body(int(m.geom_bodyid[other])).name)
                if other in support or m.geom(other).name=='floor':external=True
        return sorted(set(finger)),external
    for dx in (-12.,-6.,0.,6.,12.):
        for dy in (-12.,-6.,0.,6.,12.):
            mujoco.mj_resetData(m,d);d.qpos[:]=reset;d.qpos[-7:-4]+=(dx*x+dy*y)/1000+up*.004
            d.ctrl[:]=path[0];m.geom_friction[cube,0]=.8;mujoco.mj_forward(m,d)
            overlap=max([-float(c.dist)*1000 for c in d.contact if cube in (int(c.geom1),int(c.geom2))]+[0.])
            if overlap>.5:rows.append(dict(offset_mm=[dx,dy],initial_overlap_mm=overlap,valid_start=False));continue
            for _ in range(round(1.5/m.opt.timestep)):mujoco.mj_step(m,d)
            initial=d.qpos[-7:].copy();R0=Rotation.from_quat(initial[-4:][[1,2,3,0]]).as_matrix()
            finger,external=contacts();green=float(np.degrees(np.arccos(np.clip((R0@np.array([0,-1.,0]))@up,-1,1))))
            if not finger or external or green>20:
                rows.append(dict(offset_mm=[dx,dy],initial_overlap_mm=overlap,valid_start=False,initial_green_up_error_deg=green));continue
            state=np.empty(size);mujoco.mj_getState(m,d,state,kind)
            for name,goals in schedules:
                mujoco.mj_setState(m,d,state,kind);mujoco.mj_forward(m,d);maxerror=0.;maxself=0.;holds=[]
                for i,g in enumerate(np.r_[goals,np.repeat(goals[-1:],60,axis=0)]):
                    d.ctrl[:]=g
                    for _ in range(round(.05/m.opt.timestep)):
                        mujoco.mj_step(m,d);maxerror=max(maxerror,float(np.max(abs(d.qpos[:16]-g))))
                        for c in d.contact:
                            if c.dist<0 and int(m.geom_bodyid[c.geom1]) in hand and int(m.geom_bodyid[c.geom2]) in hand:maxself=max(maxself,-float(c.dist)*1000)
                    if i>=len(goals):
                        tip,axis,error=map(float,measure(initial[-4:],d.qpos[-4:],xp=np));finger,external=contacts()
                        shift=float(np.linalg.norm(d.qpos[-7:-4]-initial[:3])*1000)
                        holds.append(dict(error_deg=float(np.degrees(error)),tip_deg=float(np.degrees(tip)),
                            retained=bool(finger and not external and shift<100),shift_mm=shift,contacts=finger))
                row=dict(offset_mm=[dx,dy],valid_start=True,initial_overlap_mm=overlap,initial_green_up_error_deg=green,
                    settled_cube_qpos=initial.tolist(),timing=name,nominal_seconds=len(goals)*.05,
                    max_control_step_ticks=int(np.max(abs(np.diff(sim_to_ticks(goals),axis=0)))),
                    maximum_tracking_rad=maxerror,maximum_hand_self_overlap_mm=maxself,final=holds[-1],
                    final_three_second_hold_reached=all(h['retained'] and h['error_deg']<15 for h in holds))
                rows.append(row)
                if row['final']['error_deg']<45:print('PROMISING_PLACEMENT',row,flush=True)
        good=[r for r in rows if r.get('valid_start')]
        print('PLACEMENT_GRID_X',dx,'valid_runs',len(good),'best_final_error',min((r['final']['error_deg'] for r in good),default=None),flush=True)
    report=dict(source_projection_sha256=hashlib.sha256(blob).hexdigest(),runs=rows,model=manifest,
        source_joint_goals_unchanged=True,placement_search_not_a_calibration=True,
        friction_hypothesis=.8,simulated_motor_parameters_identified=False,
        hardware_execution_allowed=False,hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False)
    (OUT/'projected_policy_placement_search.json').write_text(json.dumps(report,indent=2),encoding='utf-8')

if __name__=='__main__':main()
