"""Offline partial-yaw and correct ID12 ROOT tracking review; no device access."""
import collections,hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;F=ROOT/'paced_retained_yaw_20261029_044827_659675';O=F/'offline_review';O.mkdir(exist_ok=True)
r=json.loads((F/'result.json').read_bytes());frames=json.loads((F/'camera/frames.json').read_bytes())['frames'];yaw=[s for s in r['samples'] if s['phase']=='stage_7_horizontal_yaw']
def brief(s):return dict(t=s['t'],goal=s['goal'][11],actual=s['actual'][11],error_ticks=s['goal'][11]-s['actual'][11],current_ma=s['current_ma'][11],voltage_v=s['voltage_v'][11])
selected=[]
for phase in ('loaded_countdown','stage_6_grasp_hold','stage_7_horizontal_yaw'):
    group=[f for f in frames if f['phase']==phase]
    for i in sorted(set([0,len(group)//2,max(0,len(group)-2)])):
        if group:selected.append(group[i])
report=dict(source=F.name,source_sha256=hashlib.sha256((F/'result.json').read_bytes()).hexdigest(),status=r['status'],
    error=r['error'],failed_phase=r['failed_phase'],torque_OFF_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
    cube_removal_confirmed=False,cube_may_still_be_present=r.get('cube_present'),camera_frame_count=len(frames),
    minimum_voltage_v=min(min(s['voltage_v']) for s in r['samples']),camera_blank_frames=r['camera_recording']['image_signal_blank_frames'],
    motor_id12_joint='mf_mcp (middle root), NOT middle fingertip',yaw_rows=len(yaw),
    id12_yaw_trace=[brief(s) for s in yaw[::50]]+[brief(yaw[-1])],
    selected_frames=selected,transport_recoveries=len(r['read_recovery_confirmation_events']),
    transport_recoveries_all_confirmed=all(e['status']=='confirmed' for e in r['read_recovery_confirmation_events']),
    contact_or_load_root_cause_unverified=True,motor_writes=0)
(O/'stop_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('selected_frames',)},ensure_ascii=False))
print('FRAMES',json.dumps(selected,ensure_ascii=False))
