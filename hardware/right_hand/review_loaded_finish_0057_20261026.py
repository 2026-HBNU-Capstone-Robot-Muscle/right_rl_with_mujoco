import json,hashlib
from collections import Counter
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'loaded_quarter_finish_trial_20261026_005744_776918';OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026';OUT.mkdir(exist_ok=True)
def main():
    p=SOURCE/'result.json';r=json.loads(p.read_bytes());s=r.get('samples',[])
    review={k:r.get(k) for k in ('status','phase','error','all_torque_off_verified','settings_restored','cube_placement_confirmed','cube_removal_confirmed','cube_present','physical_flip_verified','reference_and_finish_commanded','last_issued_command','camera_first_space_waits','final_off_reference_valid','final_off_endpoint_drift_ticks','thumb_mcp_release_measured_verified','thumb_mcp_release_actual_ticks','thumb_mcp_release_limit_ticks','prepared_empty_photo','placement_photo','rotation_observation_photo','placement_position_ticks','rotation_position_ticks')}
    review.update(source=SOURCE.name,source_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),sample_count=len(s),phase_counts=dict(Counter(x['phase'] for x in s)),motor_write_events=len(r.get('write_events',[])),
        max_tracking_rad=float(max(max(x['tracking_rad']) for x in s)),peak_current_ma=int(max(np.max(abs(np.array(x['current_ma']))) for x in s)),min_voltage_v=min(min(x['voltage_v']) for x in s),max_temperature_c=max(max(x['temperature_c']) for x in s),camera_recording=r.get('camera_recording'))
    for phase in ('stage_6_loaded_quarter_reference','stage_7_loaded_root_finish','stage_8_loaded_observation'):
        a=[x for x in s if x['phase']==phase]
        if a:review[phase]=dict(samples=len(a),first_actual=a[0]['actual'],last_actual=a[-1]['actual'],last_goal=a[-1]['goal'],last_goal_error_ticks=(np.array(a[-1]['actual'])-np.array(a[-1]['goal'])).tolist(),peak_current_ma=int(max(np.max(abs(np.array(x['current_ma']))) for x in a)))
    (OUT/'initial_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(review,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
