"""Summarize operator saved failures without opening any device."""
import json
from collections import Counter
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def main():
    folders=sorted(ROOT.glob('horizontal_quarter_trial_20261027_*'))[-2:]
    for folder in folders:
        r=json.loads((folder/'result.json').read_bytes());samples=r.get('samples',[])
        summary={k:r.get(k) for k in ('status','error','failed_phase','all_torque_off_verified','settings_restored','current_start_screen_accepted','current_start_screen','cube_present','cube_placement_confirmed','cube_removal_confirmed','fixed_hold_fault','progress_fault','goal_readback_policy','goal_readback_failure','camera_save_error','final_check_error','final_off_reference_valid','off_reference_capture_error') if k in r}
        summary.update(folder=folder.name,sample_count=len(samples),phases=dict(Counter(v.get('phase') for v in samples)),voltage_events=r.get('voltage_events'),read_failures=r.get('read_failures'),first_sample_keys=list(samples[0]) if samples else [],last_samples=samples[-2:])
        for name in ('write_events','writes','goal_writes','off_preload_snapshots','enable_snapshots'):
            if name in r:summary[name+'_count']=len(r[name])
        for name in ('initial','final_on_state','final_off_snapshots','initial_off_state'):
            if name in r:summary[name]=r[name]
        camera=r.get('camera_recording',{});summary['camera_summary']={k:v for k,v in camera.items() if not isinstance(v,(list,dict))};summary['report_keys']=list(r)
        (folder/'stop_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(summary,ensure_ascii=False))
if __name__=='__main__':main()
