"""One targeted model comparison: reduce the stalled RF-root preload target."""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from native_cycle import assets,restore,Guard,BOUND,ticks_to_sim,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate
SOURCE=WORK/'next_cycle/handoff_refinement_20261110/initial_second_turn_hold_replay/research_goals.npz'
OUT=ROOT/'right_hand_rotation_20261112/nominal_cycle_actual_review/rf16_cap2177_model'
if OUT.exists():raise RuntimeError('Preserve comparison')
OUT.mkdir();z=dict(np.load(SOURCE,allow_pickle=False));q=z['goals'].copy();q[:,15]=np.minimum(q[:,15],2177)
assert np.max(abs(np.diff(np.vstack([z['start'],q]),axis=0)))<=2
m,initial,_,_,provenance=assets('active_ccd50');d=restore(m,initial);g=Guard(m);cube=m.geom('cube').id;cert=SweepCertificate(m,g.hand,.3)
holds=[];active=None;stop=[]
for i,command in enumerate(q):
    hold_id=0 if 3351<=i<3405 else 1 if i>=len(q)-54 else None
    if hold_id is not None and active!=hold_id:
        active=hold_id;C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
        h={'samples':0,'supported':0,'clear':0,'minimum_upper_mm':float('inf'),'shift_mm':0.,'drift_deg':0.};holds.append(h)
    d.ctrl[:]=ticks_to_sim(command)
    for _ in range(75):
        mujoco.mj_step(m,d);stop=g.check(d)
        if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
        if hold_id is not None:
            C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
            lower,bad=cert.query(d,C,B);h['samples']+=1;h['supported']+=sample_model(m,d,g.hand,cube)['supported_sample'];h['clear']+=not bad and lower>.3
            h['minimum_upper_mm']=min(h['minimum_upper_mm'],lower);h['shift_mm']=max(h['shift_mm'],float(np.linalg.norm(C-C0)*1000));h['drift_deg']=max(h['drift_deg'],orientation_error_deg(B,B0))
        if stop:break
    if hold_id is not None:
        h['target_error_deg']=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90 if hold_id==0 else 180))
        h['tilt_deg']=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up'])
    if stop:break
np.savez_compressed(OUT/'candidate.npz',start=z['start'],goals=q)
np.savez_compressed(OUT/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
passed=bool(not stop and i+1==len(q) and len(holds)==2 and all(h['samples']==h['clear']==4050 and h['supported']/4050>.8 and h['target_error_deg']<5 and h['tilt_deg']<5 and h['shift_mm']<2 and h['drift_deg']<3 for h in holds))
r={'status':'RF16_lower_goal_comparison_not_hardware_approval','source_sha256':sha(SOURCE),'candidate_sha256':sha(OUT/'candidate.npz'),
   'RF16_goal_cap_ticks':2177,'changed_commands':int(np.count_nonzero(q[:,15]!=z['goals'][:,15])),
   'goals_done':i+1,'total_goals':len(q),'stop':stop,'holds':holds,'passed':passed,'worst':g.worst,
   'terminal_guard':g.last,'model_provenance':provenance,'native_limits_and_stop_thresholds_changed':False,'devices_opened':0,'hardware_allowed':False}
save_json(OUT/'review.json',r);print(json.dumps({k:r[k] for k in ('goals_done','stop','holds','passed','terminal_guard')}),flush=True)
