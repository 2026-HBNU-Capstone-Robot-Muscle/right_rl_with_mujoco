"""Verify the new staged transfer continuously from original cube placement.

Old90deg failures remain untouched. Intermediate yaw is a diagnostic under the
user's new task; final left-hand acquisition alignment remains unverified.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'))
import mujoco
from native_cycle import assets,restore,Guard,BOUND,ticks_to_sim,motor,sha,save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,face_up_error_deg,upright_yaw_target
from upper_layer_separation_certificate_20261105 import SweepCertificate

OUT=ROOT/'right_hand_rotation_20261112/support_first_transfer'

def main():
    p=argparse.ArgumentParser();p.add_argument('--mu',type=float,choices=(.2,.3),default=.3);a=p.parse_args()
    folder=OUT/f'continuous_mu{a.mu:g}';folder.mkdir(exist_ok=False)
    paths=[ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz',OUT/'middle_roots_like_index/research_goals.npz',
           OUT/'thumb_push_5mm/research_goals.npz',OUT/'middle_then_index_then_thumb/research_goals.npz']
    values=[dict(np.load(p,allow_pickle=False)) for p in paths];previous=values[0]['start'];segments=[]
    for p,z in zip(paths,values):
        assert np.array_equal(previous,z['start']);previous=z['goals'][-1]
        segments.append(dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p),goals=len(z['goals'])))
    q=np.vstack([z['goals'] for z in values]);start=values[0]['start']
    assert np.max(abs(np.diff(np.vstack([start,q]),axis=0)))<=2
    np.savez_compressed(folder/'candidate.npz',start=start,goals=q)
    m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=a.mu;d=restore(m,initial);g=Guard(m)
    assert np.all(q>=g.lo) and np.all(q<=g.hi)
    cube=m.geom('cube').id;cert=SweepCertificate(m,g.hand,.3);count=supported=clear=0;shift=drift=0.;minimum=float('inf')
    stop=[];task_failure=[];bad_names=[];C0=B0=None;checks=0
    for i,command in enumerate(q):
        d.ctrl[:]=ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(m,d);checks+=1;stop=g.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=len(q)-54:
                C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                if C0 is None:C0=C.copy();B0=B.copy()
                count+=1;supported+=sample_model(m,d,g.hand,cube)['supported_sample'];lower,bad=cert.query(d,C,B)
                clear+=not bad and lower>.3;minimum=min(minimum,lower)
                shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                if bad or lower<=.3:task_failure=['upper_certificate_first_failure'];bad_names=[m.geom(int(cert.ids[j])).name for j in bad]
            if stop or task_failure:break
        if (i+1)%500==0 or stop or task_failure:print(json.dumps(dict(mu=a.mu,goal=i+1,stop=stop,task_failure=task_failure)),flush=True)
        if stop or task_failure:break
    B=d.geom_xmat[cube].reshape(3,3);tilt=face_up_error_deg(B,5,initial['up'])
    passed=bool(not stop and not task_failure and i+1==len(q) and count==clear==4050 and supported/4050>.8 and shift<2 and drift<3 and tilt<5)
    row=dict(status='continuous_initial_to_two_pad_transfer_and_supported_clear_hold',passed=passed,mu=a.mu,
        done=i+1,total=len(q),all1ms_samples=checks,stop=stop,task_failure=task_failure,upper_bad_geoms=bad_names,
        hold_samples=count,supported_samples=supported,upper_certified_samples=clear,minimum_upper_mm=minimum if count else None,
        hold_shift_mm=shift,hold_drift_deg=drift,tilt_deg=tilt,
        intermediate90_error_diagnostic_deg=orientation_error_deg(B,upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)),
        intermediate90_not_acceptance_gate_for_new_task=True,source_segments=segments,model_provenance=provenance,
        first1417_goals_equal_physically_observed_route=True,actual_state_resets_after_initial_placement=0,
        final_left_acquisition_alignment_verified=False,next_rotation_included=False,physical_success=False,
        candidate_sha256=sha(folder/'candidate.npz'),worst=g.worst,terminal_guard=g.last,native_guards_unchanged=True,
        sliding_friction_is_hypothesis_not_measurement=True,devices_opened=0,hardware_execution_allowed=False)
    np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
    save_json(folder/'review.json',row)
    print(json.dumps({k:v for k,v in row.items() if k not in ('source_segments','worst','terminal_guard','model_provenance')}),flush=True)

if __name__=='__main__':main()
