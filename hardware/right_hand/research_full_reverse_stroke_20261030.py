"""Expand only to existing full-reference vectors. Research, NEVER an execution handoff."""
import json,hashlib
from pathlib import Path
import numpy as np
from loaded_quarter_reentry_20261025 import limits
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/full_reverse_stroke_research'
def main():
 O.mkdir(exist_ok=True);p=R/'right_hand_rotation_20261027/yaw90_clearance_t8_early_root16/full_loaded_27/loaded_path.npz';s=np.load(p)
 old=R/'right_hand_rotation_20261030/yaw_roundtrip/candidate.npz';z=np.load(old)
 first=s['goals'][np.isin(s['phases'],['grasp','grasp_hold','yaw','yaw_hold'])];ys=s['goals'][s['phases']=='yaw']
 ph=[{'grasp':'stage_5_loaded_grasp','grasp_hold':'stage_6_grasp_hold','yaw':'stage_7_horizontal_yaw','yaw_hold':'stage_8_horizontal_hold'}[str(v)] for v in s['phases'] if v in ('grasp','grasp_hold','yaw','yaw_hold')]
 inverse=np.vstack([ys[-2::-1],s['grasp_goal']]);tail=z['goals'][np.isin(z['phases'],['stage_9_loaded_open','stage_10_remove_hold'])];tp=z['phases'][np.isin(z['phases'],['stage_9_loaded_open','stage_10_remove_hold'])]
 goals=np.vstack([first,inverse,np.tile(inverse[-1],(40,1)),tail]);phases=np.array(ph+['stage_7_horizontal_yaw']*len(inverse)+['stage_8_horizontal_hold']*40+tp.tolist())
 lo,hi=limits();v=np.vstack([s['start'],goals]);assert np.all(v>=lo) and np.all(v<=hi) and np.max(abs(np.diff(v,axis=0)))<=2
 existing={tuple(q) for q in np.vstack([s['start'],s['goals'],z['goals']])};assert all(tuple(q) in existing for q in goals)
 np.savez_compressed(O/'candidate.npz',start=s['start'],goals=goals,phases=phases,forward_end_sample=np.array(len(first)))
 prior=json.loads((R/'right_hand_rotation_20261030/yaw_roundtrip/full_loaded_seated_front_-10/review.json').read_bytes())
 report=dict(status='offline_full_reference_stroke_comparison_only',hardware_execution_allowed=False,runtime_admission='REJECTED: composed duration exceeds unchanged 300s bound; research only',new_goal_vectors=0,
    loaded_goals=len(goals),loaded_seconds=len(goals)*.075,forward_yaw_goals=len(ys),inverse_yaw_goals=len(inverse),
    candidate_sha256=hashlib.sha256((O/'candidate.npz').read_bytes()).hexdigest(),parent_paths={p.relative_to(R).as_posix():hashlib.sha256(p.read_bytes()).hexdigest(),old.relative_to(R).as_posix():hashlib.sha256(old.read_bytes()).hexdigest()},cube_seed_json=prior['cube_seed_json'],devices_opened=False)
 (O/'proposal.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':main()
