"""Find fresh saved results and review the completed EMPTY command sequence."""
import json,datetime as dt
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
def main():
    recent=sorted([p for p in ROOT.iterdir() if p.is_dir()],key=lambda p:p.stat().st_mtime,reverse=True)[:18]
    print('RECENT',json.dumps([dict(folder=p.name,result=(p/'result.json').exists()) for p in recent],ensure_ascii=False))
    folder=ROOT/'thumb_clear_root_empty_20261026_125659_859144';r=json.loads((folder/'result.json').read_bytes())
    print('STATE_KEYS',list(r))
    for key in ('voltage_events','read_failures','goal_readback_confirmation_events','off_preload_snapshots','off_start_observations','final_off_reference_valid','final_off_endpoint_drift_ticks','final_off_snapshots'):
        if key in r:print(key,json.dumps(r[key],ensure_ascii=False)[:9000])
    f=json.loads((folder/'camera/frames.json').read_bytes())['frames'];selected=[]
    for phase in ('stage_5_empty_start_hold','stage_6_empty_quarter_reference','stage_7_empty_known_root_finish','stage_8a_empty_thumb_lift_clearance','stage_9_empty_thumb_retreat','stage_10_empty_index_middle_lift','stage_11_empty_new_pose_hold'):
        group=[v for v in f if v['phase']==phase]
        if group:selected.append(dict(phase=phase,first=group[0]['image'],middle=group[len(group)//2]['image'],last=group[-1]['image']))
    print('PHASE_IMAGES',json.dumps(selected))
    print('CANDIDATE_HELPERS',[p.name for p in ROOT.glob('*.py') if any(t in p.name for t in ('slow_thumb_root','proposal_inputs','slow_proposal','proposal_readiness'))])
if __name__=='__main__':main()
