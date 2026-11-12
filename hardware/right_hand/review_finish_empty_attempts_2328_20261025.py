import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    rows=[]
    for folder in sorted(ROOT.glob('quarter_finish_empty_20261025_23*')):
        p=folder/'result.json'
        if not p.exists():continue
        r=json.loads(p.read_bytes());s=r.get('samples',[])
        row={k:r.get(k) for k in ('status','phase','failed_phase','error','all_torque_off_verified','settings_restored','current_start_screen_accepted','last_issued_command','on_to_off_position_change_ticks','final_off_reference_valid','final_off_endpoint_drift_ticks')}
        row.update(folder=folder.name,source_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),sample_count=len(s),write_count=len(r.get('writes',[])),keys=list(r))
        for k in ('off_start_observations','initial','final_off_snapshots','camera_recording'):
            v=r.get(k)
            if k=='camera_recording' and v:row[k]={key:v.get(key) for key in ('status','error','frame_count','image_signal_usable_frames')}
            elif isinstance(v,list) and v:row[k]=v[-1]
            elif v:row[k]=v
        request=folder/'fresh_off_request.json'
        if request.exists():row['fresh_off_state']=json.loads(request.read_bytes()).get('off_state')
        proof=folder/'current_start_screen.json'
        if proof.exists():
            pr=json.loads(proof.read_bytes());row['screen']={k:pr.get(k) for k in ('accepted_for_supervised_empty_preparation','code_unchanged','extra_code_unchanged','elapsed_seconds','start_ticks','last_goal_ticks','native_placement_hold_samples','plan_version')}
            for group in ('static_screens','dynamic_sensitivity'):
                a=pr.get(group,[])
                row['screen'][group+'_summary']={k:max((x[k] for x in a if k in x and x[k] is not None),default=None) for k in ('max_self_after_release_mm','max_support_after_lift_mm','native_range_violation_rad','support_escape_max_unapproved_overlap_mm','self_escape_max_unapproved_overlap_mm','max_tracking_after_clearance_rad','max_tracking_rad')}
        log=folder/'current_start_screen.log'
        if log.exists():row['screen_log_tail']=log.read_text(encoding='utf-8',errors='replace').splitlines()[-8:]
        rows.append(row)
    out=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/finish_empty_2328_attempts_review.json';out.write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(rows,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
