"""Independently replay a small ID8 support correction from initial placement.

No cached state continuation, device APIs, changed native limits, or hidden
failure reclassification. This establishes only the nominal model result.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'))
import mujoco
from native_cycle import assets,restore,Guard,BOUND,ticks_to_sim,save_json,sha
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate
from whole_hand_support import sample_model
from repair_cycle_support_transfer_20261112 import compensated_index, SOURCE, OUT


def main():
    p=argparse.ArgumentParser();p.add_argument('--offset',type=int,required=True);a=p.parse_args()
    folder=OUT/f'independent_index{a.offset}';folder.mkdir(exist_ok=False)
    z,q,indices=compensated_index(a.offset)
    assert len(q)==5089 and np.array_equal(indices,np.arange(5089))
    m,initial,_,_,provenance=assets('active_ccd50');d=restore(m,initial);g=Guard(m);cube=m.geom('cube').id
    cert=SweepCertificate(m,g.hand,.3);holds=[];active=None;count=0;stop=[]
    for i,command in enumerate(q):
        hold=0 if 3351<=i<3405 else 1 if i>=len(q)-54 else None
        if hold is not None and active!=hold:
            active=hold;C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
            h=dict(samples=0,supported_samples=0,upper_certified_samples=0,minimum_upper_lower_mm=float('inf'),hold_shift_mm=0.,hold_drift_deg=0.);holds.append(h)
        d.ctrl[:]=ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(m,d);count+=1;stop=g.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if hold is not None:
                C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);lower,bad=cert.query(d,C,B)
                h['samples']+=1;h['supported_samples']+=sample_model(m,d,g.hand,cube)['supported_sample'];h['upper_certified_samples']+=not bad and lower>.3
                h['minimum_upper_lower_mm']=min(h['minimum_upper_lower_mm'],lower);h['hold_shift_mm']=max(h['hold_shift_mm'],float(np.linalg.norm(C-C0)*1000));h['hold_drift_deg']=max(h['hold_drift_deg'],orientation_error_deg(B,B0))
            if stop:break
        if hold is not None:
            h['target_error_deg']=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90 if hold==0 else 180))
            h['tilt_deg']=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up'])
        if (i+1)%500==0 or stop:print(json.dumps(dict(offset=a.offset,goal=i+1,stop=stop)),flush=True)
        if stop:break
    passed=bool(not stop and i+1==len(q) and len(holds)==2 and all(h['samples']==h['upper_certified_samples']==4050 and h['supported_samples']/4050>.8 and h['target_error_deg']<5 and h['tilt_deg']<5 and h['hold_shift_mm']<2 and h['hold_drift_deg']<3 for h in holds))
    np.savez_compressed(folder/'candidate.npz',start=z['start'],goals=q)
    np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
    r=dict(status='independent_index_support_correction_from_initial_placement',steps=i+1,total_goals=len(q),all1ms_samples=count,
           candidate_sha256=sha(folder/'candidate.npz'),source_files_sha256={SOURCE.relative_to(ROOT).as_posix():sha(SOURCE),
           'repair_cycle_support_transfer_20261112.py':sha(ROOT/'repair_cycle_support_transfer_20261112.py'),Path(__file__).name:sha(Path(__file__))},
           two_rotations_and_clear_holds_at_reached_locations_passed=passed,actual_state_resets_after_initial_placement=0,
           holds=holds,seams=[],seams_not_applicable_to_single_continuous_replay=True,stop=stop,passed=passed,
           final180_error_deg=holds[-1]['target_error_deg'] if holds else None,final_tilt_deg=holds[-1]['tilt_deg'] if holds else None,
           worst=g.worst,terminal_guard=g.last,model_provenance=provenance,native_guards_and_joint_bounds_unchanged=True,
           index_root_compensation_ticks=a.offset,RF16_goal_cap_ticks=2177,physical_success=False,hardware_execution_allowed=False,
           correction_is_unverified_physical_hypothesis=True,devices_opened=0)
    save_json(folder/'model_review.json',r)
    print(json.dumps({k:r[k] for k in ('passed','steps','stop','holds')}),flush=True)

if __name__=='__main__':main()
