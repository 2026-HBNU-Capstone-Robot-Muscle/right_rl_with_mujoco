"""Offline rearward thumb contact -> distal-first transfer comparisons."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from scipy.optimize import lsq_linear
from native_cycle import assets,restore,Guard,BOUND,motor,ticks_to_sim,sha,save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,aligned_top_face
from tip_first_transfer_20261112 import propose,rearward_side_offset

BASE=ROOT/'right_hand_rotation_20261112/support_first_transfer'
SOURCE=BASE/'lower_pad_lift_after_clear_v2/start_state.npz'
OUT=ROOT/'right_hand_rotation_20261112/tip_first_transfer_v1'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--compliant',action='store_true');parser.add_argument('--lookahead',type=int,default=64);parser.add_argument('--fraction',type=float);a=parser.parse_args()
    out=OUT.with_name('tip_first_transfer_v3' if a.lookahead==40 else 'tip_first_transfer_v2') if a.compliant else OUT
    out.mkdir(exist_ok=False);results=[]
    for fraction in ((a.fraction,) if a.fraction is not None else (.25,.5,.75,1.)):
        folder=out/f'fraction{fraction:g}';folder.mkdir();m,initial,_,_,provenance=assets('active_ccd50');d=restore(m,dict(np.load(SOURCE,allow_pickle=False)));g=Guard(m)
        cube=m.geom('cube').id;th=m.geom('th_tip').id;B=d.geom_xmat[cube].reshape(3,3).copy();C=d.geom_xpos[cube].copy()
        # White/front +Z and orange side -X are a contact-frame hypothesis.
        offset=rearward_side_offset(B[:,2],-B[:,0],3.);P=d.geom_xpos[th].copy();active=np.arange(12,16);ids=np.array([3,2,1,0]);T=2*np.pi/4096
        start=motor(d.ctrl);previous=start.copy();acc=start.astype(float);rows=[];phases=[];stop=[]
        def execute(command,phase):
            nonlocal previous,stop
            assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi)
            previous=command.copy();d.ctrl[:]=ticks_to_sim(command);rows.append(command.copy());phases.append(phase)
            for _ in range(75):
                mujoco.mj_step(m,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if stop:return False
            return True
        for i in range(80):
            target=P+offset*min(1.,(i+1)/60.);delta=np.clip((target-d.geom_xpos[th])*1000*.3,-.2,.2)
            J=np.zeros((3,m.nv));mujoco.mj_jac(m,d,J,None,d.geom_xpos[th],int(m.geom_bodyid[th]));A=J[:,active]*T*1000
            lo=np.maximum(-2,np.ceil(g.lo[ids])-acc[ids]);hi=np.minimum(2,np.floor(g.hi[ids])-acc[ids])
            dt=lsq_linear(np.vstack([A,.015*np.eye(4)]),np.r_[delta,np.zeros(4)],bounds=(lo-1e-9,hi+1e-9)).x
            acc[ids]=np.clip(acc[ids]+dt,np.ceil(g.lo[ids]),np.floor(g.hi[ids]));command=previous.copy();command[ids]=np.rint(acc[ids]).astype(int)
            if not execute(command,'rearward_thumb_contact'):break
        count=supported=0;shift=drift=0.;C0=B0=None;proposal=None
        if not stop:
            proposal=propose(previous,g.lo,g.hi,fraction=fraction)
            for phase,nominal in zip(proposal.phases,proposal.goals):
                command=nominal.copy()
                if a.compliant:
                    command=previous.copy()
                    if phase!='transfer_hold':
                        primary=np.array([0,8] if phase=='tips_first' else [1,10]);actual=motor(d.qpos[:16])
                        wanted=previous[primary]+np.clip(nominal[primary]-previous[primary],-2,2)
                        wanted=np.clip(wanted,actual[primary]-a.lookahead,actual[primary]+a.lookahead)
                        command[primary]=previous[primary]+np.clip(wanted-previous[primary],-2,2)
                        # Yield loaded non-primary joints instead of forcing
                        # a curled tip against an unchanged proximal preload.
                        other=np.setdiff1d(np.r_[np.arange(4),np.arange(8,12)],primary)
                        for j in other:
                            lag=previous[j]-actual[j]
                            if abs(lag)>40:
                                relief=actual[j]+np.sign(lag)*30
                                command[j]=previous[j]+int(np.clip(relief-previous[j],-2,2))
                    command=np.clip(command,np.ceil(g.lo),np.floor(g.hi)).astype(int)
                if phase=='transfer_hold' and C0 is None:C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
                if not execute(command,phase):break
                if phase=='transfer_hold':
                    load=sample_model(m,d,g.hand,cube);count+=1;supported+=load['supported_sample']
                    shift=max(shift,float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000));drift=max(drift,orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0))
        r=dict(fraction=fraction,stop=stop,done=len(rows),completed=bool(not stop and count==54),
               supported_hold_goal_observations=supported,hold_goal_observations=count,hold_shift_mm=shift,hold_drift_deg=drift,
               cube_shift_mm=float(np.linalg.norm(d.geom_xpos[cube]-C)*1000),
               top=aligned_top_face(d.geom_xmat[cube].reshape(3,3),initial['up']),targets=proposal.targets if proposal else None,
               final_load=sample_model(m,d,g.hand,cube),worst=g.worst,terminal_guard=g.last,
               source_sha256=sha(SOURCE),rearward_distance_hypothesis_mm=3.,model_provenance=provenance,
               cached_mu03_start_local_transition_only=True,full_initial_to_final_replay_required=True,
               native_guards_and_limits_unchanged=True,maximum_curl_not_automatically_authorized=True,
               compliant_nonprimary_goal_relief=a.compliant,
               primary_model_lookahead_ticks=a.lookahead,
               commanded_targets_reached=bool(proposal is not None and np.array_equal(previous,proposal.goals[-1])),
               contact_limited_targets_are_not_called_maximum_flexion=True,
               physical_success=False,hardware_execution_allowed=False,devices_opened=0)
        np.savez_compressed(folder/'research_goals.npz',start=start,goals=rows,phases=phases);np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
        save_json(folder/'review.json',r);results.append(r);save_json(out/'review.json',dict(rows=results,devices_opened=0,hardware_execution_allowed=False))
        print(json.dumps({k:r[k] for k in ('fraction','done','stop','completed','supported_hold_goal_observations','hold_shift_mm','cube_shift_mm','top')}),flush=True)

if __name__=='__main__':main()
