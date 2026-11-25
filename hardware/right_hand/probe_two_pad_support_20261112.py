"""Offline receiving-pad probe for the user's staged cube transfer proposal.

Intermediate90deg is recorded, not a rejection gate in this NEW transfer task.
Left-hand acquisition alignment and full-cycle success are not established.
Native safety limits/guards are unchanged. No physical device interfaces.
"""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from native_cycle import assets,restore,Guard,BOUND,ticks_to_sim,motor,sha,save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target
from screen_direct_work_grip_20261112 import transition

OUT=ROOT/'right_hand_rotation_20261112/support_first_transfer'
SOURCE=ROOT/'right_hand_rotation_20261112/stage2_preparation/direct_work_grip_v1/prefix_end_state.npz'
FIRST=ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz'

def main():
    OUT.mkdir(exist_ok=False)
    m,initial,_,_,provenance=assets('active_ccd50');saved=dict(np.load(SOURCE,allow_pickle=False))
    first=dict(np.load(FIRST,allow_pickle=False));d0=restore(m,saved);start=motor(d0.ctrl)
    assert np.array_equal(start,first['goals'][-1])
    up=np.asarray(initial['up']);up=up/np.linalg.norm(up)
    cube=m.geom('cube').id;IF=m.geom('if_bs_collision_1').id;MF=m.geom('mf_bs_collision_1').id
    cases={}
    target=start.copy();target[11]=start[7];target[9]=start[5];cases['middle_roots_like_index']=target
    target=start.copy();target[[11,9,10,8]]=start[[7,5,6,4]];cases['middle_shape_like_index']=target
    rows=[]
    for name,target in cases.items():
        out=OUT/name;out.mkdir();d=restore(m,saved);g=Guard(m)
        assert np.all(target>=g.lo) and np.all(target<=g.hi)
        moves=transition(start,target,'all');q=np.vstack([moves,np.tile(target,(54,1))])
        stop=[];samples=supported=0;shift=drift=0.;C0=B0=None
        for i,command in enumerate(q):
            d.ctrl[:]=ticks_to_sim(command)
            for _ in range(75):
                mujoco.mj_step(m,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if i>=len(moves):
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if C0 is None:C0=C.copy();B0=B.copy()
                    samples+=1;supported+=sample_model(m,d,g.hand,cube)['supported_sample']
                    shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                if stop:break
            if stop:break
        supported_hold=bool(not stop and i+1==len(q) and samples==4050 and supported/4050>.8 and shift<2 and drift<3)
        row=dict(case=name,goals=i+1,planned_goals=len(q),moving_goals=len(moves),stop=stop,
            intermediate_supported_hold_passed=supported_hold,hold_samples=samples,supported_samples=supported,
            hold_shift_mm=shift,hold_drift_deg=drift,intermediate90_error_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),upright_yaw_target(initial['initial_cube_rotation'],up,90)),
            intermediate90_is_not_acceptance_gate_for_new_transfer_task=True,
            base_geom_center_height_difference_mm=float((d.geom_xpos[MF]-d.geom_xpos[IF])@up*1000),
            base_geom_frame_difference_deg=orientation_error_deg(d.geom_xmat[MF].reshape(3,3),d.geom_xmat[IF].reshape(3,3)),
            terminal_support=sample_model(m,d,g.hand,cube),worst=g.worst,terminal_guard=g.last,
            lower_pad_geometry_is_diagnostic_not_receiving_contact_certificate=True,
            final_left_acquisition_alignment_verified=False,thumb_push_and_upper_link_translation_included=False,
            physical_success=False,devices_opened=0)
        np.savez_compressed(out/'research_goals.npz',start=start,goals=q[:i+1])
        np.savez_compressed(out/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
        save_json(out/'review.json',row);rows.append(row)
        save_json(OUT/'progress.json',dict(rows=rows))
        print(json.dumps({k:v for k,v in row.items() if k not in ('terminal_support','worst','terminal_guard')}),flush=True)
    save_json(OUT/'review.json',dict(status='receiving_middle_pad_first_probe',rows=rows,
        source_state_sha256=sha(SOURCE),physically_observed_first_route_sha256=sha(FIRST),
        prefix_state_is_saved_model_not_actual_cube_reconstruction=True,model_provenance=provenance,
        unchanged_native_limits_and_safety_guards=True,new_intermediate_task_contract=True,
        historical90degree_task_failures_not_relabelled=True,devices_opened=0,hardware_execution_allowed=False))

if __name__=='__main__':main()
