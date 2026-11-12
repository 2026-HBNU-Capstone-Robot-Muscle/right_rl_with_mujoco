"""Prepare a separate, shorter trial; inherit existing500s approval, no devices."""
import json
import numpy as np
from support_first_trial_plan_20261112 import *

def put(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    OUT.mkdir(exist_ok=False)
    research=ROOT/'right_hand_rotation_20261112/support_first_transfer'
    paths=[ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz',research/'middle_roots_like_index/research_goals.npz',
        research/'thumb_push_5mm/research_goals.npz',research/'middle_then_index_then_thumb/research_goals.npz']
    old=np.load(paths[0],allow_pickle=False);z=dict(np.load(SOURCE,allow_pickle=False))
    phases=list(old['phases'])+[RECEIVE]*212+[RECEIVE_HOLD]*54+[PUSH]*160+[PUSH_HOLD]*54
    settle=json.loads((paths[-1].parent/'review.json').read_bytes());previous=0
    for part in settle['segments']:
        phase={'MFupper':MIDDLE,'platformIF':INDEX,'TH':THUMB}[part['stage']]
        phases += [phase]*(part['after_goal']-previous);previous=part['after_goal']
    phases += [HOLD]*54
    assert len(phases)==len(z['goals'])==2868
    np.savez_compressed(CANDIDATE,start=z['start'],goals=z['goals'],phases=phases)
    sources=[SOURCE,*paths,research/'continuous_mu0.3/review.json',research/'continuous_mu0.2/review.json']
    put(MANIFEST,dict(candidate_sha256=sha(CANDIDATE),source_files_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in sources},
        phases=phases,loaded_goals=2868,first1417_goals_physically_observed=True,second_rotation_included=False,
        intermediate90_is_diagnostic_only=True,final_left_alignment_verified=False,physical_success=False))
    seed=ROOT/'right_hand_rotation_20261112/stage2_work_grip/request.json';old=json.loads(seed.read_bytes())
    put(REQUEST,dict(plan_version=SupportFirstTrialPlan.PLAN_VERSION,motor_ids_in_sim_order=old['motor_ids_in_sim_order'],
        off_state=old['off_state'],source=seed.relative_to(ROOT).as_posix(),source_sha256=sha(seed),
        seed_is_saved_OFF_not_current_device_measurement=True,runtime_must_remeasure_actual_OFF_before_any_write=True,hardware_execution_allowed=False))
    approval=ROOT/'right_hand_rotation_20261112/nominal_cycle_trial_preparation/operator_scope_approval.json'
    approved=json.loads(approval.read_bytes());assert approved['user_explicitly_approved'] and approved['maximum_on_seconds']==500
    put(APPROVAL,dict(maximum_on_seconds=500,candidate_sha256=sha(CANDIDATE),operator_SPACE_required=True,
        inherited_approval_path=approval.relative_to(ROOT).as_posix(),inherited_approval_sha256=sha(approval),
        scope='User-requested shorter support-first repair trial; all physical limits and500s maximum retained; no next rotation',
        user_requested_strategy_and_physical_verification_preparation=True,assistant_started_motion=False))
    p=SupportFirstTrialPlan(REQUEST);rows=p.plan(p.source_position())
    put(OUT/'proposal.json',dict(loaded_goals=len(z['goals']),whole_goals_from_saved_seed=len(rows),
        command_seconds=len(rows)*.075,conservative_budget_seconds=(len(rows)+40)*.075+20,maximum_on_seconds=500,
        second_rotation_included=False,final_left_alignment_verified=False,devices_opened=0))
    print('Separate support-first trial prepared; motion not started.')

if __name__=='__main__':main()
