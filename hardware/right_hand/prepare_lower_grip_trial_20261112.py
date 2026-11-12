"""Bind the repaired low-friction grasp observation trial; no device access."""
import json
import numpy as np
from lower_grip_trial_plan_20261112 import *

def put(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
    OUT.mkdir(exist_ok=False)
    research=ROOT/'right_hand_rotation_20261112/support_first_transfer/sequential_low_mu_v4'
    r=json.loads((research/'review.json').read_bytes());z=dict(np.load(research/'candidate.npz',allow_pickle=False))
    assert not r['stop'] and r['hold_samples']==r['supported_samples']==4050
    assert all(f>.8 for f in r['loaded_fractions'].values())
    first=np.load(ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz',allow_pickle=False)
    phases=list(first['phases'])+[RECEIVE]*652+[RECEIVE_HOLD]*54+[PUSH]*150+[RELAX]*40+[INDEX]*200+[PUSH_HOLD]*54
    mapping={'contact_acquire_rf':RING,'contact_acquire_th':THUMB,'contact_acquire_mf':MIDDLE,'final_three_finger_hold':HOLD}
    phases += [mapping[str(p)] for p in z['phases'][2567:]]
    assert len(phases)==len(z['goals'])==3194
    np.savez_compressed(SOURCE,start=z['start'],goals=z['goals'])
    np.savez_compressed(CANDIDATE,start=z['start'],goals=z['goals'],phases=phases)
    sources=[SOURCE,research/'candidate.npz',research/'review.json',ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz']
    put(MANIFEST,dict(candidate_sha256=sha(CANDIDATE),source_files_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in sources},
        phases=phases,loaded_goals=len(phases),first1417_goals_physically_observed=True,
        scope='Supervised repaired rotation-to-three-finger-support observation; no upper-layer operation or next rotation',
        next_rotation_included=False,physical_success=False,upper_space_known_pending=True,low_friction_hypotheses_not_measurements=True))
    seed=ROOT/'right_hand_rotation_20261112/stage2_work_grip/request.json';saved=json.loads(seed.read_bytes())
    put(REQUEST,dict(plan_version=LowerGripTrialPlan.PLAN_VERSION,motor_ids_in_sim_order=saved['motor_ids_in_sim_order'],off_state=saved['off_state'],
        source=seed.relative_to(ROOT).as_posix(),source_sha256=sha(seed),seed_is_saved_OFF_not_current_device_measurement=True,
        runtime_must_remeasure_actual_OFF_before_any_write=True,hardware_execution_allowed=False))
    approval=ROOT/'right_hand_rotation_20261112/nominal_cycle_trial_preparation/operator_scope_approval.json';approved=json.loads(approval.read_bytes())
    assert approved['user_explicitly_approved'] and approved['maximum_on_seconds']==500
    put(APPROVAL,dict(maximum_on_seconds=500,candidate_sha256=sha(CANDIDATE),operator_SPACE_required=True,
        inherited_approval_path=approval.relative_to(ROOT).as_posix(),inherited_approval_sha256=sha(approval),
        scope='User-requested repaired lower-grip observation; prior500s scope and physical limits retained',assistant_started_motion=False))
    p=LowerGripTrialPlan(REQUEST);rows=p.plan(p.source_position());candidate()
    put(OUT/'proposal.json',dict(loaded_goals=len(phases),whole_goals_from_saved_seed=len(rows),
        conservative_budget_seconds=(len(rows)+40)*.075+20,maximum_on_seconds=500,
        next_rotation_included=False,full_cycle_ready=False,devices_opened=0))
    print('Separate repaired-grasp packet assembled; model screen and proof required before execution.')

if __name__=='__main__':main()
