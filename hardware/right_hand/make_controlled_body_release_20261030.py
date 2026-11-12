"""Keep the exact lower-grasp prefix; retract contacts before removal, no full OPEN."""
import json,hashlib
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;P=R/'right_hand_rotation_20261030/lower_body_regrasp';O=R/'right_hand_rotation_20261030/lower_body_gap_release';O.mkdir(exist_ok=True)
def blend(a,b):
 n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*k/n for k in range(1,n+1)]).astype(int)
z=np.load(P/'candidate.npz');geo=json.loads((P/'geometry_review.json').read_bytes());end=int(z['body_hold_end_sample']);previous=z['goals'][end-1];release=[];records=[r for r in geo['records'] if r['phase']=='reclose'];knots=[r['goal'] for r in records[-2::-1]]+[[r for r in geo['records'] if r['phase']=='lower_thumb'][-1]['goal']]
for q in knots:
 q=np.asarray(q);release.extend(blend(previous,q));previous=q
goals=np.vstack([z['goals'][:end],np.asarray(release),np.tile(previous,(40,1))]);phases=np.array(z['phases'][:end].tolist()+['stage_15_body_contact_release']*len(release)+['stage_16_unclamped_remove_hold']*40);assert np.max(abs(np.diff(np.vstack([z['start'],goals]),axis=0)))<=2
np.savez_compressed(O/'candidate.npz',start=z['start'],goals=goals,phases=phases,forward_end_sample=z['forward_end_sample'],body_hold_end_sample=z['body_hold_end_sample'])
(O/'proposal.json').write_text(json.dumps(dict(status='controlled_contact_release_after_same_body_hold_research',source_candidate_sha256=hashlib.sha256((P/'candidate.npz').read_bytes()).hexdigest(),candidate_sha256=hashlib.sha256((O/'candidate.npz').read_bytes()).hexdigest(),exact_held_prefix_goals=end,release_goals=len(release),goals=len(goals),automatic_full_OPEN=False,model_gap_design_mm=3,physical_release_not_verified=True,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
s=(R/'probe_lower_body_regrasp_20261030.py').read_text(encoding='utf-8').replace("O=R/'right_hand_rotation_20261030/lower_body_regrasp'","O=R/'right_hand_rotation_20261030/lower_body_gap_release'")
s=s.replace("for torque in (0.,.005,-.005,.01,-.01):","for torque in ():")
s=s.replace("held=snapshot();heldR=rotation();heldC=d.qpos[-7:-4].copy();print", "held=snapshot();np.savez_compressed(O/'held_state.npz',**held);heldR=rotation();heldC=d.qpos[-7:-4].copy();print")
s=s.replace("body_hold_task_all_probes_passed=bool(probes and all(r['body_hold_task_passed'] for r in probes))", "body_hold_task_all_probes_passed=None,torque_probes_not_repeated_here=True,identical_held_prefix_torque_probes='right_hand_rotation_20261030/lower_body_regrasp/free_continuation_review.json'")
s=s.replace("physical_rotation_and_hold_not_verified=True", "controlled_contact_release_no_full_OPEN=True,final_released_state=contact_report(),physical_rotation_and_hold_not_verified=True")
(R/'probe_controlled_body_release_20261030.py').write_text(s,encoding='utf-8');print('Created controlled release research candidate:',len(goals),'goals;',len(release),'contact retraction goals.')
