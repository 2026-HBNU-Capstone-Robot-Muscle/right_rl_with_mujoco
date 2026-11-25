"""Retract TH along its actual closest cube normal before free-space routing."""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from scipy.optimize import lsq_linear
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,TICK,save_json,sha
from directed_contact_state import forwarded_state,tip_yaw_contact
from cube_task_frame_20261105 import orientation_error_deg
from whole_hand_support import sample_model

def main():
    p=argparse.ArgumentParser();p.add_argument('--tag',required=True);p.add_argument('--source-state',required=True);a=p.parse_args();base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/a.tag;source=(OUT/a.source_state).resolve()
    if not a.tag.replace('_','').isalnum() or folder.exists() or not source.is_relative_to(OUT.resolve()):raise ValueError('New local model comparison required')
    m,initial,*_=assets('active_ccd50');m.geom_friction[:,0]=.25;z=dict(np.load(source,allow_pickle=False));d=restore(m,z);g=Guard(m);cube=m.geom('cube').id;tip=m.geom('th_tip').id;active=np.arange(12,16);sid=np.asarray(SIM_TO_MOTOR_IDS)-1;previous=motor(d.ctrl);fraction=np.zeros(4);commands=[];stop=[];settle=None;samples=supported=0;C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();shift=drift=0.
    folder.mkdir();(folder/'source_retract_thumb_for_free_approach.py').write_bytes(Path(__file__).read_bytes())
    for step in range(200):
        sensed=forwarded_state(m,d);points=np.zeros(6);gap=float(mujoco.mj_geomDistance(m,sensed,cube,tip,.2,points))
        if gap>.0015 and settle is None:settle=30
        command=previous.copy()
        if settle is None:
            normal=points[3:]-points[:3];normal/=max(np.linalg.norm(normal),1e-12);J=np.zeros((3,m.nv));mujoco.mj_jac(m,sensed,J,None,points[3:],int(m.geom_bodyid[tip]));fit=lsq_linear(np.vstack([J[:,active]*TICK,np.eye(4)*1e-7]),np.r_[normal*.00005,np.zeros(4)],bounds=(-2,2));fraction+=fit.x;command[sid[active]]=previous[sid[active]]+np.clip(np.rint(fraction),-2,2);command[sid[active]]=np.clip(command[sid[active]],np.ceil(g.lo[sid[active]]),np.floor(g.hi[sid[active]]));fraction-=command[sid[active]]-previous[sid[active]]
        d.ctrl[active]=ticks_to_sim(command)[active];assert np.max(abs(command-previous))<=2;commands.append(command.copy());previous=command.copy()
        for _ in range(75):
            mujoco.mj_step(m,d);stop=g.check(d);shift=max(shift,float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000));drift=max(drift,orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0))
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if shift>2 or drift>3:stop=list(stop)+['retraction_disturbed_cube']
            if settle is not None:samples+=1;supported+=sample_model(m,d,g.hand,cube)['supported_sample']
            if stop:break
        if stop:break
        if settle is not None:
            settle-=1
            if settle==0:break
    actual=tip_yaw_contact(m,d,cube,tip,initial['up']);np.savez_compressed(folder/'research_goals.npz',start=motor(z['ctrl']),goals=commands);np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    r=dict(status='TH_actual_normal_retraction_before_free_route',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),steps=len(commands),stop=stop,actual_tip_contact=actual,supported_samples=supported,hold_samples=samples,maximum_cube_shift_mm=shift,maximum_cube_drift_deg=drift,free_approach_start_reached=bool(not stop and actual['gap_mm']>1 and samples==supported==2250),other12_joint_commands_unchanged=True,original_native_guards_and_joint_limits_preserved=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,worst=g.worst);save_json(folder/'review.json',r);print(json.dumps(r),flush=True)

if __name__=='__main__':main()
