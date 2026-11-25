"""Offline upper-link placement after preparing both lower pads and thumb push.

Use existing working-grip upper-joint targets, keeping prepared lower joints
and thumb fixed. Final yaw is reported; left-hand alignment is separate.
"""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'))
import mujoco
from native_cycle import assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,face_up_error_deg,upright_yaw_target
from upper_layer_separation_certificate_20261105 import SweepCertificate

OUT=ROOT/'right_hand_rotation_20261112/support_first_transfer'
SOURCE=OUT/'thumb_push_5mm/end_state.npz'

def main():
    m,initial,_,_,provenance=assets('active_ccd50');saved=dict(np.load(SOURCE,allow_pickle=False))
    seed=restore(m,saved);start=motor(seed.ctrl);cube=m.geom('cube').id
    known=np.load(ROOT/'right_hand_rotation_20261112/stage2_work_grip/candidate.npz',allow_pickle=False)['goals'][-1]
    cases={'index_upper':[[4,6]],'index_then_middle_upper':[[4,6],[8,10]]}
    reports=[]
    for name,groups in cases.items():
        folder=OUT/name
        if folder.exists() and any(folder.iterdir()):raise RuntimeError('Preserve completed upper-link probe')
        folder.mkdir(exist_ok=True);d=restore(m,saved);g=Guard(m);previous=start.copy();moving=[]
        for group in groups:
            target=previous.copy();target[group]=known[group]
            n=max(1,int(np.ceil(np.max(abs(target-previous))/2)))
            moving.extend(np.rint(previous+(target-previous)*k/n).astype(int) for k in range(1,n+1));previous=target
        goals=np.vstack([moving,np.tile(previous,(54,1))]);cert=SweepCertificate(m,g.hand,.3)
        assert np.max(abs(np.diff(np.vstack([start,goals]),axis=0)))<=2
        count=supported=clear=0;shift=drift=0.;minimum=float('inf');C0=B0=None;task_failure=[];upper_bad_geoms=[]
        for i,command in enumerate(goals):
            d.ctrl[:]=ticks_to_sim(command)
            for _ in range(75):
                mujoco.mj_step(m,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if i>=len(moving):
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if C0 is None:C0=C.copy();B0=B.copy()
                    count+=1;supported+=sample_model(m,d,g.hand,cube)['supported_sample']
                    lower,bad=cert.query(d,C,B);minimum=min(minimum,lower);clear+=not bad and lower>.3
                    shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                    if bad or lower<=.3:
                        task_failure=['final_upper_certificate_first_failure'];upper_bad_geoms=[m.geom(int(cert.ids[j])).name for j in bad]
                if stop or task_failure:break
            if stop or task_failure:break
        B=d.geom_xmat[cube].reshape(3,3);tilt=face_up_error_deg(B,5,initial['up'])
        passed=bool(not stop and not task_failure and count==clear==4050 and supported/4050>.8 and shift<2 and drift<3 and tilt<5)
        row=dict(case=name,done=i+1,total=len(goals),moving_goals=len(moving),stop=stop,task_failure=task_failure,upper_bad_geoms=upper_bad_geoms,
            intermediate_clear_supported_grip_passed=passed,hold_samples=count,supported_samples=supported,upper_certified_samples=clear,
            minimum_upper_mm=minimum if count else None,hold_shift_mm=shift,hold_drift_deg=drift,tilt_deg=tilt,
            intermediate90_error_diagnostic_deg=orientation_error_deg(B,upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)),
            lower_joint_and_thumb_commands_frozen=True,final_left_acquisition_alignment_verified=False,
            source_sha256=sha(SOURCE),model_provenance=provenance,worst=g.worst,terminal_guard=g.last,
            physical_success=False,devices_opened=0,native_guards_unchanged=True)
        np.savez_compressed(folder/'research_goals.npz',start=start,goals=goals[:i+1])
        np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
        save_json(folder/'review.json',row);reports.append(row);save_json(OUT/'upper_link_progress.json',dict(rows=reports))
        print(json.dumps({k:v for k,v in row.items() if k not in ('worst','terminal_guard','model_provenance')}),flush=True)
    save_json(OUT/'upper_link_review.json',dict(rows=reports,all_conditions_safety_certified=False,
         final_left_acquisition_alignment_verified=False,physical_success=False,devices_opened=0))

if __name__=='__main__':main()
