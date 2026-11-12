import json,hashlib
from pathlib import Path
from collections import Counter
import numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/finish_empty_completion_2345';OUT.mkdir(exist_ok=True)
def main():
    summaries=[]
    for folder in sorted(ROOT.glob('quarter_finish_empty_20261025_2345*')):
        p=folder/'result.json';r=json.loads(p.read_bytes());samples=r.get('samples',[])
        row={k:r.get(k) for k in ('status','phase','failed_phase','error','all_torque_off_verified','settings_restored','hardware_authorized','last_issued_command','final_off_reference_valid','final_off_endpoint_drift_ticks','on_to_off_position_change_ticks','current_start_screen_accepted','camera_first_space_waits')}
        row.update(source=folder.name,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),sample_count=len(samples),phases=dict(Counter(s['phase'] for s in samples)),write_count=len(r.get('write_events',[])),camera=r.get('camera_recording'))
        if samples:
            row.update(max_tracking_rad=float(max(max(s['tracking_rad']) for s in samples)),peak_current_ma=int(max(max(abs(np.array(s['current_ma']))) for s in samples)),min_voltage_v=min(min(s['voltage_v']) for s in samples),max_temperature_c=max(max(s['temperature_c']) for s in samples),last_on_sample=samples[-1])
            row['final_goal_error_ticks']=(np.array(samples[-1]['actual'])-np.array(samples[-1]['goal'])).tolist()
        row['final_off_snapshots']=r.get('final_off_snapshots',[])
        summaries.append(row)
    (OUT/'completion_review.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(summaries,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
