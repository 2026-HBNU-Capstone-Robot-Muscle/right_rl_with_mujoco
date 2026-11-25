"""Review the latest EMPTY thumb/root run locally; no device access."""
import json
from pathlib import Path
from collections import Counter
import numpy as np
ROOT=Path(__file__).resolve().parent
def main():
    runs=sorted(p for p in ROOT.glob('thumb_clear_root_empty_*') if p.is_dir())
    if not runs:raise SystemExit('No saved run folder')
    folder=runs[-1];p=folder/'result.json'
    if not p.exists():raise SystemExit('Result not yet saved: '+str(folder))
    r=json.loads(p.read_bytes());s=r.get('samples',[])
    keys=('status','error','failed_phase','phase','all_torque_off_verified','settings_restored','hardware_authorized',
          'current_start_screen_accepted','last_issued_phase','last_issued_command','fixed_hold_fault','progress_fault',
          'camera_recording','final_off_reference_valid','final_off_endpoint_drift_ticks')
    out={k:r.get(k) for k in keys};out.update(folder=folder.name,samples=len(s),writes=len(r.get('write_events',[])),
        sample_phases=dict(Counter(x['phase'] for x in s)),report_keys=list(r))
    if s:
        out.update(last_sample=s[-1],min_voltage=min(min(v['voltage_v']) for v in s),
            peak_current_ma=int(max(np.max(abs(np.array(v['current_ma']))) for v in s)),
            max_tracking_rad=max(max(v['tracking_rad']) for v in s))
    log=folder/'current_start_screen.log'
    if log.exists():out['screen_log_tail']=log.read_text(encoding='utf-8',errors='replace').splitlines()[-18:]
    q=folder/'fresh_off_request.json'
    if q.exists():out['start_ticks']=json.loads(q.read_bytes())['off_state']['position']
    proof=folder/'current_start_screen.json'
    if proof.exists():
        v=json.loads(proof.read_bytes());out['proof_summary']={k:v.get(k) for k in ('plan_version','accepted_for_supervised_empty_preparation','accepted_for_supervised_grasp_approach','elapsed_seconds')}
        fields=('book_front_mm','gain_scale','max_self_after_release_mm','max_support_after_lift_mm','native_range_violation_rad','self_escape_max_unapproved_overlap_mm','support_escape_max_unapproved_overlap_mm','max_tracking_after_clearance_rad')
        out['dynamic']=[{k:z.get(k) for k in fields} for z in v.get('dynamic_sensitivity',[])]
    dest=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454'/('review_'+folder.name)
    dest.mkdir(exist_ok=True);(dest/'review.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in out.items() if k!='report_keys'},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
