"""Audit and test joining the actual trial's model endpoint to the old grip.

The archived TH/MF/RF endpoint is a MODEL reference, not a certified physical
grip. IF is retained as temporary support. The cube is free throughout execution.
"""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from native_cycle import assets,restore,Guard,BOUND,motor,ticks_to_sim,sha,save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate
from screen_direct_work_grip_20261112 import transition

OUT=ROOT/'right_hand_rotation_20261112/support_first_transfer/three_finger_join'
SOURCE=OUT.parent/'continuous_mu0.3/end_state.npz'
OLD_REPORT=WORK/'next_cycle/handoff_refinement_20261110/ring_thumb_MF_support_feedback_v2/review.json'

def main():
    OUT.mkdir(exist_ok=False)
    previous=json.loads(OLD_REPORT.read_bytes());selected=[r for r in previous['rows'] if r['temporary_three_finger_hold_passed']]
    assert len(selected)==1
    ref=WORK/selected[0]['path']/'end_state.npz';saved=dict(np.load(SOURCE,allow_pickle=False));reference=dict(np.load(ref,allow_pickle=False))
    m,initial,_,_,provenance=assets('active_ccd50');d0=restore(m,saved);r0=restore(m,reference)
    start=motor(d0.ctrl);target=motor(r0.ctrl);target[4:8]=start[4:8];cube=m.geom('cube').id
    source_center=d0.geom_xpos[cube].copy();ref_center=r0.geom_xpos[cube].copy()
    audit=dict(current_model_endpoint_ticks=start.tolist(),archived_model_TH_MF_RF_target_ticks=target.tolist(),
        source_state_sha256=sha(SOURCE),archived_reference_state=ref.relative_to(ROOT).as_posix(),archived_reference_sha256=sha(ref),
        old_reference_report_sha256=sha(OLD_REPORT),old_reference_is_model_not_physical=True,
        cube_center_difference_model_mm=float(np.linalg.norm(ref_center-source_center)*1000),
        cube_orientation_difference_model_deg=orientation_error_deg(d0.geom_xmat[cube].reshape(3,3),r0.geom_xmat[cube].reshape(3,3)),
        temporary_index_goals_retained=True,native_limits_and_guards_unchanged=True,devices_opened=0)
    save_json(OUT/'target_audit.json',audit);print(json.dumps(audit),flush=True)
    results=[]
    for order in ('TH_RF_MF','RF_TH_MF'):
        folder=OUT/order;folder.mkdir();d=restore(m,saved);g=Guard(m)
        assert np.all(target>=g.lo) and np.all(target<=g.hi)
        moves=transition(start,target,order);q=np.vstack([moves,np.tile(target,(54,1))]);cert=SweepCertificate(m,g.hand,.3)
        count=supported=clear=0;loaded={f:0 for f in ('th','mf','rf')};shift=drift=0.;minimum=float('inf');task_failure=[];bad_names=[];C0=B0=None
        for i,command in enumerate(q):
            d.ctrl[:]=ticks_to_sim(command)
            for _ in range(75):
                mujoco.mj_step(m,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if i>=len(moves):
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if C0 is None:C0=C.copy();B0=B.copy()
                    sample=sample_model(m,d,g.hand,cube);count+=1;supported+=sample['supported_sample']
                    for f in loaded:
                        force=sum(v['normal_N'] for name,v in sample['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(f+'_'))
                        loaded[f]+=force>.1
                    lower,bad=cert.query(d,C,B);minimum=min(minimum,lower);clear+=not bad and lower>.3
                    shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                    if bad or lower<=.3:task_failure=['upper_certificate_first_failure'];bad_names=[m.geom(int(cert.ids[j])).name for j in bad]
                if stop or task_failure:break
            if stop or task_failure:break
        fractions={f:v/max(1,count) for f,v in loaded.items()}
        tilt=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up'])
        passed=bool(not stop and not task_failure and count==clear==4050 and supported/4050>.8 and all(v>.8 for v in fractions.values()) and shift<2 and drift<3 and tilt<5)
        row=dict(order=order,done=i+1,total=len(q),stop=stop,task_failure=task_failure,upper_bad_geoms=bad_names,
            three_finger_supported_clear_hold_passed=passed,hold_samples=count,supported_samples=supported,upper_certified_samples=clear,
            loaded_contact_fractions=fractions,minimum_upper_mm=minimum if count else None,hold_shift_mm=shift,hold_drift_deg=drift,tilt_deg=tilt,
            worst=g.worst,terminal_guard=g.last,old_native_guards_unchanged=True,devices_opened=0,physical_success=False,hardware_execution_allowed=False)
        np.savez_compressed(folder/'research_goals.npz',start=start,goals=q[:i+1]);np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
        save_json(folder/'review.json',row);results.append(row);save_json(OUT/'review.json',dict(rows=results,target_audit=audit,model_provenance=provenance,devices_opened=0))
        print(json.dumps({k:v for k,v in row.items() if k not in ('worst','terminal_guard')}),flush=True)

if __name__=='__main__':main()
