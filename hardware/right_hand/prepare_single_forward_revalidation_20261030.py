import json,hashlib
from pathlib import Path
import numpy as np
from loaded_quarter_reentry_20261025 import limits
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/single_forward';O.mkdir(exist_ok=True)
src=R/'right_hand_rotation_20261028/observed90_prefix638/full_loaded_27/loaded_path.npz';z=np.load(src);rt=np.load(R/'right_hand_rotation_20261030/yaw_roundtrip/candidate.npz');end=int(rt['forward_end_sample'])
assert np.array_equal(z['goals'][:end],rt['goals'][:end]);mapping={'grasp':'stage_5_loaded_grasp','grasp_hold':'stage_6_grasp_hold','yaw':'stage_7_horizontal_yaw','yaw_hold':'stage_8_horizontal_hold','release':'stage_9_loaded_open','remove_hold':'stage_10_remove_hold'}
goals=z['goals'];phases=np.array([mapping[str(p)] for p in z['phases']]);lo,hi=limits();v=np.vstack([z['start'],goals]);assert np.max(abs(np.diff(v,axis=0)))<=2 and np.all(v>=lo) and np.all(v<=hi) and len(goals)==2032
np.savez_compressed(O/'candidate.npz',start=z['start'],goals=goals,phases=phases,forward_end_sample=np.array(end));np.savez_compressed(O/'whole_hand_path.npz',start=z['start'],goals=goals,phases=phases)
report=dict(status='same_physical_forward_reference_revalidation_in_progress',source=src.relative_to(R).as_posix(),source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),candidate_sha256=hashlib.sha256((O/'candidate.npz').read_bytes()).hexdigest(),
   loaded_goals=2032,nominal_loaded_s=152.4,new_joint_goal_vectors=0,automatic_inverse_removed=True,automatic_open_retained=True,
   historical_placed_to_held_estimates_deg=[88.71186086891521,84.44548087670685],exact90_not_certified=True,physical_repeatability_not_certified=True,
   initial_cube_seating_revalidation_required=True,hardware_execution_allowed=False,devices_opened=False)
(O/'proposal.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
