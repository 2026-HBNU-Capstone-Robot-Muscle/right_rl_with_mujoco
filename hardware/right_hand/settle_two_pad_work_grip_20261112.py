"""Keep two lower pads aligned while freeing upper space after translation."""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'))
import mujoco
from native_cycle import assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg,face_up_error_deg,upright_yaw_target
from whole_hand_support import sample_model
from upper_layer_separation_certificate_20261105 import SweepCertificate

OUT=ROOT/'right_hand_rotation_20261112/support_first_transfer'
SOURCE=OUT/'thumb_push_5mm/end_state.npz'

def main():
    m,initial,_,_,provenance=assets('active_ccd50');z=dict(np.load(SOURCE,allow_pickle=False));d0=restore(m,z);start=motor(d0.ctrl)
    known=np.load(ROOT/'right_hand_rotation_20261112/stage2_work_grip/candidate.npz',allow_pickle=False)['goals'][-1]
    target=known.copy();target[11]=target[7];target[9]=target[5]
    # Ring stays as physically observed; matching lower joints remain a shared
    # platform. Upper-link moves and thumb departure have explicit stages.
    target[12:]=start[12:]
    groups={'MFupper':[8,10],'platformIF':[4,5,6,7,9,11],'TH':[0,1,2,3]}
    rows=[]
    for name,order in [('middle_then_index_then_thumb',('MFupper','platformIF','TH')),
                       ('thumb_then_index_then_middle',('TH','platformIF','MFupper'))]:
        folder=OUT/name;folder.mkdir(exist_ok=False);d=restore(m,z);g=Guard(m);previous=start.copy();moving=[];segments=[]
        for stage in order:
            goal=previous.copy();active=groups[stage];goal[active]=target[active]
            n=max(1,int(np.ceil(np.max(abs(goal-previous))/2)))
            moving.extend(np.rint(previous+(goal-previous)*k/n).astype(int) for k in range(1,n+1));previous=goal
            segments.append(dict(stage=stage,after_goal=len(moving)))
        goals=np.vstack([moving,np.tile(previous,(54,1))]);assert np.max(abs(np.diff(np.vstack([start,goals]),axis=0)))<=2
        cube=m.geom('cube').id;cert=SweepCertificate(m,g.hand,.3);count=supported=clear=0;shift=drift=0.;minimum=float('inf')
        stop=[];task_failure=[];upper_bad=[];C0=B0=None
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
                    if bad or lower<=.3:task_failure=['upper_certificate_first_failure'];upper_bad=[m.geom(int(cert.ids[j])).name for j in bad]
                if stop or task_failure:break
            if (i+1)%200==0 or stop or task_failure:print(json.dumps(dict(case=name,goal=i+1,stop=stop,task_failure=task_failure)),flush=True)
            if stop or task_failure:break
        B=d.geom_xmat[cube].reshape(3,3);tilt=face_up_error_deg(B,5,initial['up'])
        passed=bool(not stop and not task_failure and count==clear==4050 and supported/4050>.8 and shift<2 and drift<3 and tilt<5)
        row=dict(case=name,done=i+1,total=len(goals),stop=stop,task_failure=task_failure,upper_bad_geoms=upper_bad,
            supported_clear_grip_passed=passed,hold_samples=count,supported_samples=supported,upper_certified_samples=clear,
            minimum_upper_mm=minimum if count else None,hold_shift_mm=shift,hold_drift_deg=drift,tilt_deg=tilt,
            intermediate90_error_diagnostic_deg=orientation_error_deg(B,upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)),
            matched_lower_joint_targets=True,segments=segments,source_sha256=sha(SOURCE),model_provenance=provenance,
            worst=g.worst,terminal_guard=g.last,final_left_alignment_verified=False,native_guards_unchanged=True,
            physical_success=False,devices_opened=0)
        np.savez_compressed(folder/'research_goals.npz',start=start,goals=goals[:i+1])
        np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
        save_json(folder/'review.json',row);rows.append(row);save_json(OUT/'settle_progress.json',dict(rows=rows))
        print(json.dumps({k:v for k,v in row.items() if k not in ('terminal_guard','worst','model_provenance','segments')}),flush=True)
    save_json(OUT/'settle_review.json',dict(rows=rows,physical_success=False,devices_opened=0))

if __name__=='__main__':main()
