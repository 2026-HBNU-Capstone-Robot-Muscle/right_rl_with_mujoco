"""Prepare the rear-contact / tip-first proposal as a distinct bounded trial."""
import json
import numpy as np
from tip_first_trial_plan_20261112 import *

def put(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
    OUT.mkdir(exist_ok=False)
    prior=ROOT/'right_hand_rotation_20261112/lower_grip_trial/candidate.npz'
    folder=ROOT/'right_hand_rotation_20261112/tip_first_transfer_v3/fraction0.5';research=folder/'research_goals.npz'
    report=json.loads((folder/'review.json').read_bytes());assert report['completed'] and not report['stop']
    old=dict(np.load(prior,allow_pickle=False));part=dict(np.load(research,allow_pickle=False))
    assert np.array_equal(old['goals'][2272],part['start'])
    phases=list(old['phases'][:2273]);mapping={'rearward_thumb_contact':REAR,'tips_first':TIPS,'next_links_second':NEXT,'transfer_hold':HOLD}
    phases += [mapping[str(p)] for p in part['phases']]
    goals=np.vstack([old['goals'][:2273],part['goals']]);np.savez_compressed(SOURCE,start=old['start'],goals=goals)
    np.savez_compressed(CANDIDATE,start=old['start'],goals=goals,phases=phases)
    sources=[SOURCE,prior,research,folder/'review.json',ROOT/'tip_first_transfer_20261112.py']
    put(MANIFEST,dict(candidate_sha256=sha(CANDIDATE),source_files_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in sources},
        phases=phases,loaded_goals=len(goals),scope='Rearward thumb contact and distal-first transfer observation only',
        ring_closing_included=False,next_rotation_included=False,maximum_curl_achieved=False,
        physical_success=False,rearward_contact_frame_is_hypothesis=True,full_three_finger_grasp_included=False))
    seed=ROOT/'right_hand_rotation_20261112/stage2_work_grip/request.json';saved=json.loads(seed.read_bytes())
    put(REQUEST,dict(plan_version=TipFirstTrialPlan.PLAN_VERSION,motor_ids_in_sim_order=saved['motor_ids_in_sim_order'],off_state=saved['off_state'],
        source=seed.relative_to(ROOT).as_posix(),source_sha256=sha(seed),seed_is_saved_OFF_not_current_device_measurement=True,
        runtime_must_remeasure_actual_OFF_before_any_write=True,hardware_execution_allowed=False))
    approval=ROOT/'right_hand_rotation_20261112/nominal_cycle_trial_preparation/operator_scope_approval.json';approved=json.loads(approval.read_bytes())
    assert approved['user_explicitly_approved'] and approved['maximum_on_seconds']==500
    put(APPROVAL,dict(maximum_on_seconds=500,candidate_sha256=sha(CANDIDATE),operator_SPACE_required=True,
        inherited_approval_path=approval.relative_to(ROOT).as_posix(),inherited_approval_sha256=sha(approval),
        scope='User-requested repaired distal-first transfer observation; prior500s scope and physical limits retained',assistant_started_motion=False))
    p=TipFirstTrialPlan(REQUEST);rows=p.plan(p.source_position());candidate()
    put(OUT/'proposal.json',dict(loaded_goals=len(goals),whole_goals_from_saved_seed=len(rows),
        conservative_budget_seconds=(len(rows)+40)*.075+20,maximum_on_seconds=500,
        next_rotation_included=False,full_cycle_ready=False,devices_opened=0))
    print('Separate distal-first packet assembled; full model screening required.')

if __name__=='__main__':main()
