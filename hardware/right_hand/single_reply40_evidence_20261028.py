"""Source-bound read-only timing evidence, not physical reliability certification."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'single_reply_profile_20261028_054656_033637/result.json'
def review():
    r=json.loads(SOURCE.read_bytes());rows=r.get('observations',[]);packets=r.get('packet_latencies',[])
    if r.get('status')!='readonly_window_comparison_complete' or r.get('motor_writes')!=0 or r.get('torque_changes')!=0 or r.get('baud')!=1000000:raise RuntimeError('Missing READ-only link-window evidence')
    if len(rows)!=200 or len(packets)!=3200 or any(v.get('window_ms')!=40. or v.get('comm')!=0 or v.get('error')!=0 for v in rows):raise RuntimeError('Incomplete or failing40ms full snapshots')
    if any(v.get('window_ms')!=40. or v.get('comm')!=0 or len(v.get('requested_ids',[]))!=1 or not 0<=v.get('receive_ms',-1)<40 for v in packets):raise RuntimeError('Single-reply latency evidence rejected')
    from collections import Counter
    if Counter(v['requested_ids'][0] for v in packets)!=Counter({i:200 for i in range(1,17)}):raise RuntimeError('Missing motor reply coverage')
    if r.get('frames',0)<100 or r.get('unusable_frames')!=0:raise RuntimeError('Missing concurrent normal camera evidence')
    return dict(source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        health_reply_window_ms=40.,torque_off_window_ms=80.,full_snapshots=200,single_replies=3200,failures=0,
        maximum_observed_reply_ms=max(v['receive_ms'] for v in packets),camera_frames=r['frames'],bad_camera_frames=0,
        shared_sample_budget_ms=200,required_clean_snapshots_after_recovery=2,
        motion_or_setting_write_retries=0,physical_motion_link_reliability_verified=False,motor_writes=0)
