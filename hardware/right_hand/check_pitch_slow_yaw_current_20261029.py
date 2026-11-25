"""Validate fresh READ-only snapshots against the ready plan; no device I/O."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from pitch_slow_yaw_plan_20261029 import ROOT,OUT,REQUEST,PitchSlowYawPlan
from pitch_slow_yaw_gate_20261029 import run_screen
def main():
    parser=argparse.ArgumentParser();parser.add_argument('source');args=parser.parse_args()
    source=ROOT/args.source;r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and r['servo_writes']==0 and r['torque_changes']==0
    request=json.loads(REQUEST.read_bytes());request.update(off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    file=OUT/'latest_readonly_request.json';file.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    p=PitchSlowYawPlan(file);reference=PitchSlowYawPlan(REQUEST)
    comparison=dict(source=args.source,source_position=r['snapshots'][-1]['position'],
        maximum_difference_from_review_ticks=int(np.max(abs(p.raw_off_ticks-reference.source_position()))),
        reviewed_center_reused=p.review_center_reused,devices_opened=False,servo_writes=0)
    if comparison['maximum_difference_from_review_ticks']<=4:
        run_screen(file,OUT/'latest_readonly_cached_review.json');comparison['cached_review_verified']=True
    else:comparison['cached_review_verified']=False
    (OUT/'latest_readonly_comparison.json').write_text(json.dumps(comparison,indent=2)+'\n',encoding='utf-8')
    visual=ROOT/'right_hand_rotation_20261028/two_axis_connection/actual_handoff_yaw_diagnostic/continuous_visual_review'
    comparison['model_frames_written']=len(list((visual/'frames').glob('frame_*.jpg')))
    comparison['model_video_exists']=(visual/'model_only_pitch_then_slow_yaw_3x.mp4').is_file()
    print(json.dumps(comparison))
if __name__=='__main__':main()
