"""Small saved-result summaries; no simulator or hardware imports."""
from pathlib import Path
import json
import math
import numpy as np

OUT=Path(__file__).resolve().parent;BASE=OUT/'next_cycle/handoff_refinement_20261110'
def main():
    audit=json.loads((BASE/'first_to_next_contact_bridge_20261111/audit.json').read_text())
    options=[]
    for r in audit['rows']:
        if not r['label'].startswith('actual_policy_prefix_') or min(r['finger_normals_N'][f] for f in ('mf','th'))<.2:continue
        change=r['control_change_to_reference_deg'];qdistance=math.sqrt(sum(v*v for v in change.values()))
        options.append(dict(source=r['label'],center_mm=r['distance_to_next_start_mm'],first90_error_deg=r['orientation90_error_deg'],joint_reference_distance_deg=qdistance,MF_TH_normals_N=[r['finger_normals_N'][f] for f in ('mf','th')],rank_score=r['distance_to_next_start_mm']+.1*r['orientation90_error_deg']+.025*qdistance))
    studies=[]
    for name in ('first90_loaded_MFTH_entry1000','first90_loaded_MFTH_entry1700','first90_loaded_MFTH_entry3600','first90_loaded_cp152_retained_hybrid','actual_first_A_reconfigured_next90','actual_first_A_support_transfer_next90'):
        folder=BASE/name;p=folder/'review.json';done=p.exists()
        if not done:p=folder/'progress.json'
        if not p.exists():continue
        r=json.loads(p.read_text());ev=r['final'] if done else r['actual']
        studies.append(dict(name=name,completed=done,steps=r['steps'] if done else r['step'],error_deg=ev['error_deg'],center_mm=ev['center_target_error_mm'],MF_TH_normals_N=ev['contact_finger_normals_N'],stop=r.get('stop'),passed=r.get('conditional_bridge_and_hold_passed',False)))
    a=np.load(BASE/'initial_to_reached_workstation90/end_state.npz',allow_pickle=False);b=np.load(BASE/'resumed_next90_joined_branch/end_state.npz',allow_pickle=False)
    alternate=dict(first_end_to_known_next_end_center_mm=float(np.linalg.norm(a['qpos'][-7:-4]-b['qpos'][-7:-4])*1000),control_change_deg=np.degrees(b['ctrl']-a['ctrl']).tolist(),comparison_only_not_a_replayed_connection=True)
    print(json.dumps(dict(nearest_loaded_prefixes=sorted(options,key=lambda r:r['rank_score'])[:5],studies=studies,reached_A_posture_comparison=alternate)),flush=True)


if __name__=='__main__':main()
