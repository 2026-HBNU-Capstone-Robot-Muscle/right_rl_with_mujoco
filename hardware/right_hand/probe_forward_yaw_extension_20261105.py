"""Offline additional-yaw proposal with the original native/contact guards.

Pinned poses are used only to propose IK. The separate free-cube replay is
required; no pinned trajectory is counted as successful object motion.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R,CP,ramp
from bounded_contact_transport_proposal_20261105 import propose_bounded
from bounded_translating_contact_transport_20261105 import propose_translating
from research_rotation_body_hold_20261104 import motor,restore,angle
from canonical_grip_dynamics_20261105 import run
from canonical_grip_task_review_20261105 import task_reasons

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--weight',type=float,choices=(5.,10.),required=True);ap.add_argument('--translate',action='store_true');ap.add_argument('--from-common',action='store_true');ap.add_argument('--self-weight',type=float,choices=(500.,500000.),default=500.);a=ap.parse_args()
    out=ROOT/f'yaw90_extension/{"common_" if a.from_common else ""}normal{a.weight:g}{"_translating" if a.translate else ""}{"_separated" if a.self_weight!=500 else ""}';out.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3' if a.from_common else CP
    m=mujoco.MjModel.from_binary_path(str(folder/('model.mjb' if a.from_common else 'end_model.mjb')));source=folder/'end_state.npz';z=np.load(source);rd=restore(m,z);begin_yaw=angle(rd.xmat[m.body('cube').id].reshape(3,3),z['initial_cube_rotation'],z['up']);turn=90-begin_yaw;began=time.perf_counter();rows,records,error=propose_translating(m,z,turn,a.weight,a.self_weight) if a.translate else propose_bounded(m,z,turn,a.weight)
    report=dict(status='remaining_yaw_to90_contact_transport_proposal',source_state=str(source.relative_to(R)),source_state_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),from_common_grip=a.from_common,begin_yaw_deg=begin_yaw,requested_additional_yaw_deg=turn,normal_weight=a.weight,proposal_self_penalty_weight=a.self_weight,original_dynamic_self_contact_bound_unchanged=True,temporary_translation_allowed_in_proposal=a.translate,proposal_failed=error,proposal_steps=records,pinned_cube_for_IK_proposal_only=True,free_cube_result=None,hardware_execution_allowed=False,devices_opened=False)
    if rows is not None:
        canonical=np.load(ROOT/'tip_squeeze/id1/plus12/candidate.npz')
        if a.from_common:goals=rows;forward_count=len(rows)
        else:suffix=ramp(rows[-1],canonical['goals'][-1],134);goals=np.vstack([rows,suffix]);forward_count=len(rows)
        if np.max(abs(np.diff(np.vstack([motor(z['ctrl']),goals]),axis=0)))>2:raise ValueError('step bound')
        result=run(m,z,goals,forward_count,out/'end_state.npz',out/'trajectory.npz',True);why=task_reasons(result);report.update(free_cube_result=result,regrasp_reasons=why,whole_turn_target90_passed=bool(not why and abs(result['forward_yaw_from_placement_deg']-90)<5 and abs(result['final_yaw_from_placement_deg']-90)<5))
        np.savez_compressed(out/'diagnostic_suffix.npz',start=motor(z['ctrl']),goals=goals)
        if a.from_common:np.savez_compressed(out/'candidate.npz',start=canonical['start'],goals=np.vstack([canonical['goals'],goals]))
    report['elapsed_seconds']=time.perf_counter()-began
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('proposal_steps','free_cube_result')}|{'successful_IK_steps':len(records),'last_IK':records[-1] if records else None,'free_cube':None if report['free_cube_result'] is None else {k:report['free_cube_result'][k] for k in ('safety_passed','stop','completed_goals','total_goals','forward_yaw_from_placement_deg','final_yaw_from_placement_deg','regrasp_yaw_change_deg')}}),flush=True)
if __name__=='__main__':main()
