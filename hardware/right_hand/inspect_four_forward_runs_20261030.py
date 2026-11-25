import json,hashlib
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/four_forward_review';O.mkdir(exist_ok=True)
reports=[]
for F in sorted(R.glob('single_forward_trial_*'))[-4:]:
 p=F/'result.json';r=json.loads(p.read_bytes());ss=r.get('samples',[]);cam=r.get('camera_recording',{})
 summary=dict(folder=F.name,status=r.get('status'),error=r.get('error'),stop=r.get('stop_reason'),all_torque_off_verified=r.get('all_torque_off_verified'),settings_restored=r.get('settings_restored'),cube_present=r.get('cube_present'),cube_placement_confirmed=r.get('cube_placement_confirmed'),cube_removal_confirmed=r.get('cube_removal_confirmed'),
  sample_count=len(ss),phase_counts={v:sum(s['phase']==v for s in ss) for v in dict.fromkeys(s['phase'] for s in ss)},
  min_voltage_v=min((min(s['voltage_v']) for s in ss),default=None),peak_current_ma=max((max(map(abs,s['current_ma'])) for s in ss),default=None),max_tracking_rad=max((max(s['tracking_rad']) for s in ss),default=None),
  camera_frames=cam.get('frame_count'),blank=cam.get('image_signal_blank_frames'),read_failures=len(r.get('read_failures',[])),result_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),photos=[f.name for f in F.glob('*.jpg')])
 reports.append(summary);print(json.dumps(summary,ensure_ascii=False),flush=True)
 print('CONTROL_FIELDS',F.name,{k:v for k,v in r.items() if ('stop' in k or 'exception' in k or 'error' in k) and not isinstance(v,(list,dict))},flush=True)
 if r.get('write_events'):print('WRITE_KEYS',list(r['write_events'][0]),flush=True)
(O/'run_summary.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
