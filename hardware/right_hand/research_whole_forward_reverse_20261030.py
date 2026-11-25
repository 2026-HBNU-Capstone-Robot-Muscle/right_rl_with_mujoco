"""Restore the omitted reverse-grasp segment using the exact observed forward vectors."""
import json,hashlib
from pathlib import Path
import numpy as np
from loaded_quarter_reentry_20261025 import limits
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/whole_forward_reverse_research'
def main():
 O.mkdir(exist_ok=True);src=R/'right_hand_rotation_20261030/yaw_roundtrip/candidate.npz';z=np.load(src);e=int(z['forward_end_sample']);ph=z['phases'];g=z['goals'];grasp=g[ph=='stage_5_loaded_grasp']
 yaw=g[:e][ph[:e]=='stage_7_horizontal_yaw'];inv=np.vstack([yaw[-2::-1],grasp[-1],grasp[-2::-1],z['start']])
 goals=np.vstack([g[:e],inv,np.tile(z['start'],(60,1))]);phases=np.array(ph[:e].tolist()+['stage_7_horizontal_yaw']*len(inv)+['stage_8_horizontal_hold']*40+['stage_10_remove_hold']*20)
 lo,hi=limits();v=np.vstack([z['start'],goals]);assert np.all(v>=lo) and np.all(v<=hi) and np.max(abs(np.diff(v,axis=0)))<=2
 observed={tuple(q) for q in np.vstack([z['start'],g])};assert all(tuple(q) in observed for q in goals)
 np.savez_compressed(O/'candidate.npz',start=z['start'],goals=goals,phases=phases,forward_end_sample=np.array(e))
 report=dict(status='offline_exact_forward_path_and_reverse_including_grasp',hardware_execution_allowed=False,devices_opened=False,
   loaded_goals=len(goals),inverse_goals=len(inv),omitted_grasp_goals_now_reversed=len(grasp),nominal_loaded_seconds=len(goals)*.075,
   new_joint_goal_vectors=0,endpoint_equals_original_open_pose=True,physical_rotation_not_verified=True,
   source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),candidate_sha256=hashlib.sha256((O/'candidate.npz').read_bytes()).hexdigest(),
   scope='The inverse includes reverse-grasp/opening, not only the638 yaw vectors. Contact and object rotation must be measured dynamically before any execution handoff.')
 (O/'proposal.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':main()
