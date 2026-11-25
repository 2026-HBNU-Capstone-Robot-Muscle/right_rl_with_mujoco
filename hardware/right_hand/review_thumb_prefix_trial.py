"""Summarize saved user-run telemetry; no devices opened."""
import argparse
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('folder');args=parser.parse_args()
    folder=ROOT/args.folder;r=json.loads((folder/'result.json').read_bytes())
    a=r['initial']['position'];b=r['final_on_state']['position'];g=r['final_on_state']['goal']
    samples=r['samples']
    frames=json.loads((folder/'camera/frames.json').read_text())
    groups={}
    for frame in frames['frames']:groups.setdefault(frame['phase'],[]).append(frame)
    report=dict(source_sha256=hashlib.sha256((folder/'result.json').read_bytes()).hexdigest(),
        timestamp=r['timestamp'],status=r['status'],torque_off=r['all_torque_off_verified'],
        restored=r['settings_restored'],min_voltage=min(min(x['voltage_v']) for x in samples),
        max_current=max(max(abs(i) for i in x['current_ma']) for x in samples),
        max_tracking=max(max(x['tracking_rad']) for x in samples),
        moving_joints=[dict(id=i+1,start=a[i],goal=g[i],actual=b[i],
            commanded_deg=(g[i]-a[i])*360/4096,actual_deg=(b[i]-a[i])*360/4096,
            final_error_deg=(g[i]-b[i])*360/4096) for i in (0,1,2,3,7,11,15)],
        camera_status=frames['status'],camera_frames=len(frames['frames']),
        phase_images={k:dict(count=len(v),first=v[0]['image'],last=v[-1]['image']) for k,v in groups.items()},
        physical_contact_verified=False,full_thumb_preparation_verified=False,physical_cube_rotation_verified=False)
    report['final_on_position_ticks']=b
    report['final_on_goal_ticks']=g
    report['final_off_reference_valid']=r.get('final_off_reference_valid',False)
    report['on_to_off_position_change_ticks']=r.get('on_to_off_position_change_ticks')
    report['final_off_endpoint_drift_ticks']=r.get('final_off_endpoint_drift_ticks')
    if r.get('final_off_snapshots'):
        report['final_off_position_ticks']=r['final_off_snapshots'][-1]['position']
    (folder/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
