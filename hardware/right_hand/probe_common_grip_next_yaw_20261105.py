"""Can the current common grip enter a second recorded yaw without re-placement?

No cube resets, human placement or hardware adapters. A failed bridge remains a
failed connection; reversing a target path does not imply an inverse cube turn.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json
from pathlib import Path
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R,ramp
from research_rotation_body_hold_20261104 import restore,motor,angle
from canonical_grip_dynamics_20261105 import run
from canonical_grip_task_review_20261105 import task_reasons

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--matrix-tag',default='full_placement_v2');ap.add_argument('--candidate',type=Path,default=ROOT/'recentered_free/i-77_m-16_t-56/from_reference/candidate.npz');ap.add_argument('--tag',default='common_v2_to_next_yaw');ap.add_argument('--order',choices=('simultaneous','thumb_middle_index','middle_thumb_index'),default='simultaneous');a=ap.parse_args()
    folder=ROOT/a.matrix_tag/'front-10/g1_f0.3';raw=json.loads((folder.parent/'review.json').read_bytes());digest=hashlib.sha256(a.candidate.read_bytes()).hexdigest()
    if raw['candidate_sha256']!=digest:raise ValueError('candidate/source state mismatch')
    source=folder/'end_state.npz';m=mujoco.MjModel.from_binary_path(str(folder/'model.mjb'));z=np.load(source);d=restore(m,z);B0=d.xmat[m.body('cube').id].reshape(3,3).copy();C0=d.qpos[-7:-4].copy()
    old=np.load(R/'right_hand_rotation_20261104/forward_hold/candidate.npz');path=np.load(a.candidate)
    phases=old['phases'];turn=np.flatnonzero(phases=='stage_7_horizontal_yaw')
    if not len(turn):raise ValueError('no saved yaw phase')
    entry=old['goals'][turn[0]-1];start=motor(z['ctrl'])
    if a.order=='simultaneous':bridge=ramp(start,entry,40)
    else:
        groups={'thumb':list(range(0,4)),'middle':list(range(8,12)),'index':list(range(4,8)),'ring':list(range(12,16))};parts=[];previous=start.copy()
        for name in a.order.split('_')+['ring']:
            target=previous.copy();target[groups[name]]=entry[groups[name]]
            if not np.array_equal(target,previous):parts.append(ramp(previous,target,40))
            previous=target
        bridge=np.vstack(parts)
    recorded_turn=old['goals'][turn[0]:];suffix=path['goals'][len(old['goals']):]
    goals=np.vstack([bridge,recorded_turn,suffix]);forward_count=len(bridge)+len(recorded_turn)
    if np.max(abs(np.diff(np.vstack([motor(z['ctrl']),goals]),axis=0)))>2:raise ValueError('step bound')
    out=ROOT/'next_rotation'/a.tag;out.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(out/'diagnostic_path.npz',start=motor(z['ctrl']),goals=goals)
    result=run(m,z,goals,forward_count,out/'end_state.npz',out/'trajectory.npz',True)
    why=task_reasons(result);end=np.load(out/'end_state.npz');fd=restore(m,end);B=fd.xmat[m.body('cube').id].reshape(3,3);additional=angle(B,B0,z['up']);center=float(np.linalg.norm(fd.qpos[-7:-4]-C0)*1000)
    report=dict(status='continuous_common_grip_to_next_yaw_connection_probe',order=a.order,source=str(source.relative_to(R)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),candidate_sha256=digest,bridge_goals=len(bridge),recorded_turn_and_hold_goals=len(recorded_turn),additional_yaw_deg=additional,common_center_return_error_mm=center,dynamic=result,grip_task_reasons=why,diagnostic_connection_filter_passed=bool(not why and 50<additional<80 and center<2),expected_model_turn_is_only_approximately61deg=True,additional_physical90_not_verified=True,not_a_validated_quarter_turn_primitive=True,cube_pose_never_reset_after_start=True,hardware_execution_allowed=False,devices_opened=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='dynamic'}|{'safety_passed':result['safety_passed'],'stop':result['stop'],'completed_goals':result['completed_goals'],'total_goals':result['total_goals']}),flush=True)
if __name__=='__main__':main()
