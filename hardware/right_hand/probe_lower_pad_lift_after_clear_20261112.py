"""Hand over thumb translation to paired lower-pad motion before its limit.

Replay only150 verified commands before the earlier188th-command failure.
The thumb follows the free cube in an offline model; no hardware sensor claim.
This is not the final three-finger grasp or an execution packet.
"""
import json
import argparse
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from scipy.optimize import lsq_linear
from native_cycle import assets,restore,Guard,BOUND,motor,ticks_to_sim,sha,save_json
from cube_task_frame_20261105 import orientation_error_deg
from whole_hand_support import sample_model

PARENT=ROOT/'right_hand_rotation_20261112/support_first_transfer'
SOURCE=PARENT/'middle_shape_like_index/end_state.npz'
PUSH=PARENT/'clear_middle_before_push/middle_shape_like_index/research_goals.npz'
OUT=PARENT/'lower_pad_lift_after_clear'

def main():
    p=argparse.ArgumentParser();p.add_argument('--release-and-rigid-follow',action='store_true');args=p.parse_args()
    destination=OUT.parent/(OUT.name+'_v2') if args.release_and_rigid_follow else OUT
    destination.mkdir(exist_ok=False);m,initial,_,_,provenance=assets('active_ccd50');d=restore(m,dict(np.load(SOURCE,allow_pickle=False)));g=Guard(m)
    prefix=np.load(PUSH,allow_pickle=False)['goals'][:150]
    for command in prefix:
        d.ctrl[:]=ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(m,d)
            if g.check(d) or np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:
                raise RuntimeError('Safe transfer prefix no longer reproduces')
    np.savez_compressed(destination/'start_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
    start=motor(d.ctrl);previous=start.copy();cube=m.geom('cube').id;tip=m.geom('th_tip').id
    Cbegin=d.geom_xpos[cube].copy();Bbegin=d.geom_xmat[cube].reshape(3,3).copy();Pbegin=d.geom_xpos[tip].copy()
    up=np.asarray(initial['up']);up=up/np.linalg.norm(up);th=np.array([15,14,13,12]);TICK=2*np.pi/4096;jac=np.zeros((3,m.nv))
    rows=[];trace=[];count=supported=0;shift=drift=0.;stop=[]
    if args.release_and_rigid_follow:
        # Reduce existing thumb preload while both lower pads carry the cube.
        # This is a model-goal comparison, not a gain or force-limit change.
        actual=motor(d.qpos[:16]);relax=start.copy();relax[:4]=np.rint(actual[:4]+.3*(start[:4]-actual[:4])).astype(int)
        for _ in range(40):
            command=previous+np.clip(relax-previous,-2,2);assert np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[:]=ticks_to_sim(command);rows.append(command);previous=command.copy()
            for ms in range(75):
                mujoco.mj_step(m,d)
                if g.check(d) or np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:
                    raise RuntimeError('Thumb preload handover failed; no paired lift executed')
        Cbegin=d.geom_xpos[cube].copy();Bbegin=d.geom_xmat[cube].reshape(3,3).copy();Pbegin=d.geom_xpos[tip].copy()
    local_tip=Bbegin.T@(Pbegin-Cbegin)
    for i in range(254):
        command=previous.copy()
        if i<200:
            # Move both lower roots together; old upper links remain retracted.
            command[[7,11]]=start[[7,11]]+min(200,(i+1)*2)
            target_tip=(d.geom_xpos[cube]+d.geom_xmat[cube].reshape(3,3)@local_tip if args.release_and_rigid_follow
                        else Pbegin+(d.geom_xpos[cube]-Cbegin))
            desired=np.clip((target_tip-d.geom_xpos[tip])*1000,-.3,.3)
            mujoco.mj_jac(m,d,jac,None,d.geom_xpos[tip],int(m.geom_bodyid[tip]))
            matrix=jac[:,th]*TICK*1000;lo=np.maximum(-2,np.ceil(g.lo[:4])-previous[:4]);hi=np.minimum(2,np.floor(g.hi[:4])-previous[:4])
            delta=lsq_linear(np.vstack([matrix,.02*np.eye(4)]),np.r_[desired,np.zeros(4)],bounds=(lo-1e-9,hi+1e-9)).x
            command[:4]=np.clip(previous[:4]+np.rint(delta).astype(int),np.ceil(g.lo[:4]),np.floor(g.hi[:4])).astype(int)
        if i==200:C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
        assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi)
        d.ctrl[:]=ticks_to_sim(command);rows.append(command);previous=command.copy()
        for _ in range(75):
            mujoco.mj_step(m,d);stop=g.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=200:
                C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);count+=1;supported+=sample_model(m,d,g.hand,cube)['supported_sample']
                shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
            if stop:break
        if (i+1)%40==0 or stop:
            load=sample_model(m,d,g.hand,cube)
            row=dict(goal=i+1,stop=stop,cube_lift_mm=float((d.geom_xpos[cube]-Cbegin)@up*1000),
                total_shift_mm=float(np.linalg.norm(d.geom_xpos[cube]-Cbegin)*1000),loaded=load['loaded_hand_geoms'])
            trace.append(row);print(json.dumps(row),flush=True)
        if stop:break
    stable=bool(not stop and count==4050 and supported/4050>.8 and shift<2 and drift<3)
    r=dict(status='paired_lower_pad_motion_after_upper_middle_withdrawal',done=len(rows),total=294 if args.release_and_rigid_follow else 254,stop=stop,
        stable_supported_intermediate_hold=stable,hold_samples=count,supported_samples=supported,hold_shift_mm=shift,hold_drift_deg=drift,
        cube_lift_mm=float((d.geom_xpos[cube]-Cbegin)@up*1000),total_shift_mm=float(np.linalg.norm(d.geom_xpos[cube]-Cbegin)*1000),
        safe_prefix_goals=150,source_sha256=sha(SOURCE),prefix_source_sha256=sha(PUSH),
        model_provenance=provenance,trace=trace,worst=g.worst,terminal_guard=g.last,
        thumb_preload_goal_relief_and_rigid_cube_follow=args.release_and_rigid_follow,
        model_cube_feedback_not_a_hardware_controller=True,three_finger_final_grasp_included=False,
        native_guards_changed=False,physical_success=False,devices_opened=0,hardware_execution_allowed=False)
    np.savez_compressed(destination/'research_goals.npz',start=start,goals=rows)
    np.savez_compressed(destination/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
    save_json(destination/'review.json',r)
    print(json.dumps({k:v for k,v in r.items() if k not in ('trace','worst','terminal_guard','model_provenance')}),flush=True)

if __name__=='__main__':main()
