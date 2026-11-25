"""Retain existing physics/health limits and bind the actual four-reply test."""
import hashlib,json
from pathlib import Path
import numpy as np
import reference_speed_yaw_gate_20261029 as base
from four_reply_yaw_plan_20261029 import FourReplyYawPlan,ROOT,REQUEST,PROOF,BENCH,hashes
from fast_health_read4_20261029 import POLICY
from fast_health_window_20261029 import WINDOW_POLICY as PREVIOUS_WINDOW
WINDOW_POLICY=dict(PREVIOUS_WINDOW,version='fast_health4_original_OFF16_QPC_v1_20261029',full_health_group_size=4,
    OFF_configuration_health_reply_group_size=8)
def bench_review():
    r=json.loads(BENCH.read_bytes())
    if r['status']!='fast4_readonly_comparison_complete' or r['servo_writes'] or r['torque_changes']:raise RuntimeError('Four-reply READ-only evidence rejected')
    if r['candidate_source_sha256']!=hashlib.sha256((ROOT/'fast_health_read4_20261029.py').read_bytes()).hexdigest() or r['candidate_policy']!=POLICY:raise RuntimeError('Four-reply transport differs from its actual test')
    if set(r['instruction_counts'])-{'1','2','138'} or len(r['firmware'])!=16 or any(x['model']!=1240 or x['firmware']<46 for x in r['firmware']):raise RuntimeError('READ-only instruction or firmware mismatch')
    if len(r['comparison'])!=20 or [b['group_size'] for b in r['blocks']]!=[8,4,4,8]:raise RuntimeError('Four-reply comparison is incomplete')
    if any(b['requested']!=150 or b['complete']!=150 or b['failures'] for b in r['blocks']):raise RuntimeError('OFF camera-load read failure')
    if r.get('camera_recording',{}).get('status')!='complete':raise RuntimeError('Preview/recording evidence incomplete')
    episodes=r['host_corruption_episodes']
    if len(episodes)!=3 or any(not e['incomplete_snapshot_discarded'] or not e['two_fresh_complete_OFF_snapshots'] or e['reply_groups_received']!=10 or e['elapsed_ms']>=200 for e in episodes):raise RuntimeError('Four-reply fault recovery rejected')
    return dict(path=BENCH.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(BENCH.read_bytes()).hexdigest(),policy=POLICY,
        compared_pairs=20,camera_preview_and_JPEG_recording=True,OFF_camera_load_reads_each_method=300,
        actual_partial_receive_failures_in_comparison=0,host_only_CRC_faults=3,
        motion_ON_stability_not_yet_verified=True,group4_advantage_over_group8_not_demonstrated=True,
        exact_same_CRC_ID_device_status_decoder=True,motor_writes=0)
def make_review(p,empty,before):
    r=base.make_review(p,empty,base.hashes());after=hashes()
    if before!=after:raise RuntimeError('Source changed during four-reply review')
    r.update(status='supervised_four_reply_reference_speed_review',code_hashes=after,four_reply_readonly_evidence=bench_review(),
        sync_receive_window_policy=WINDOW_POLICY,verified_serial_link_policy=dict(r['verified_serial_link_policy'],full_health_reply_group_size=4),
        active_health_read_policy=POLICY,torque_OFF_verification_unchanged=True,OFF_setting_readbacks_use_completed8_device_implementation=True)
    return r
def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Four-reply proof/source/limit mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=FourReplyYawPlan(request);reference=FourReplyYawPlan(REQUEST);r=validate_review(reference,PROOF)
    if np.array_equal(p.source_position(),reference.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    base.process_screen(request,output,checker=ROOT/'screen_four_reply_yaw_20261029.py',failure_details=lambda p:'four-reply current empty recovery rejected',**kwargs)
    return validate_review(p,output)
