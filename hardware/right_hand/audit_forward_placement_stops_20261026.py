"""Read saved trial failures only; never opens cameras or motor ports."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    folders=sorted([p for pat in ('forward_cube_placement_*','loaded_quarter_finish_trial_*') for p in ROOT.glob(pat) if p.is_dir()],key=lambda p:p.name.rsplit('_',3)[-3:],reverse=True)
    reports=[]
    for p in folders[:8]:
        f=p/'result.json'
        if not f.exists():continue
        r=json.loads(f.read_bytes());s=r.get('samples',[]);w=r.get('write_events',[])
        row=dict(folder=p.name,**{k:r.get(k) for k in ('timestamp','status','error','failed_phase','phase','hardware_authorized','all_torque_off_verified','settings_restored','current_start_screen_accepted','cube_placement_confirmed','cube_removal_confirmed','cube_present','physical_flip_verified','final_off_reference_valid','final_off_endpoint_drift_ticks','last_issued_command')},sample_count=len(s),motor_write_events=len(w))
        if s:row['last_samples']=s[-3:]
        req=p/'fresh_off_request.json'
        if req.exists():
            q=json.loads(req.read_bytes());row['fresh_off_state']=q.get('off_state',q)
        proof=p/'current_start_screen.json'
        if proof.exists():
            q=json.loads(proof.read_bytes());row['screen_summary']={k:v for k,v in q.items() if not isinstance(v,(list,dict))}
            static=q.get('static_screens',[]);dynamic=q.get('dynamic_sensitivity',[])
            row['screen_case_counts']=[len(static),len(dynamic)]
            keys=('book_front_mm','gain_scale','offset_ticks','self_escape_complete','support_escape_complete','self_escape_complete_at','self_escape_deadline','support_escape_complete_at','support_escape_deadline','self_escape_max_unapproved_overlap_mm','support_escape_max_unapproved_overlap_mm','max_self_after_release_mm','max_support_after_lift_mm','max_tracking_rad','max_tracking_after_clearance_rad','native_range_violation_rad','unmodified_joint_range_violation_rad')
            bad=[]
            for a in static+dynamic:
                if (not a.get('self_escape_complete',True) or not a.get('support_escape_complete',True)
                        or a.get('self_escape_max_unapproved_overlap_mm',0)>1e-6 or a.get('support_escape_max_unapproved_overlap_mm',0)>1e-6
                        or a.get('max_self_after_release_mm',0)>1e-6 or a.get('max_support_after_lift_mm',0)>.15):bad.append({k:a.get(k) for k in keys if k in a})
            row['first_bad_screen_cases']=bad[:5];row['bad_screen_case_count']=len(bad)
        log=p/'current_start_screen.log'
        if log.exists():row['screen_log_tail']=log.read_text(encoding='utf-8',errors='replace').splitlines()[-12:]
        camera=r.get('camera_recording',{})
        row['camera']={k:camera.get(k) for k in ('status','frame_count','image_signal_usable_frames','image_signal_blank_frames')}
        reports.append(row)
    out=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/forward_repeated_stop_review.json'
    out.write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
    for row in reports:
        print(json.dumps({k:v for k,v in row.items() if k not in ('last_samples','fresh_off_state','screen_summary')},ensure_ascii=False))
    print('Saved',out)
if __name__=='__main__':main()
