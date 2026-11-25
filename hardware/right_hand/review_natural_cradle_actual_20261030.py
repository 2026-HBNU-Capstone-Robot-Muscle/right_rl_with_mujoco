"""Review saved operator runs; no device access or control."""
import json,hashlib
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/natural_cradle_grasp/actual_review';O.mkdir(exist_ok=True);rows=[]
for name in ('natural_cradle_grasp_20261030_105943_010618','natural_cradle_grasp_20261030_110443_655705'):
 f=R/name;r=json.loads((f/'result.json').read_bytes());samples=r.get('samples',[]);frames=json.loads((f/'camera/frames.json').read_bytes())['frames'];phases={};photo_keys={k:v for k,v in r.items() if 'photo' in k and isinstance(v,dict)}
 for s in samples:phases[s['phase']]=phases.get(s['phase'],0)+1
 health={}
 for key in ('voltage_v','current_ma','temperature_c','tracking_rad'):
  vals=[np.asarray(s[key]) for s in samples if key in s];health[key]=[float(min(np.min(v) for v in vals)),float(max(np.max(v) for v in vals))] if vals else None
 hold=[s for s in samples if s['phase']=='stage_6_grasp_hold'];placed=[v for v in frames if v['phase']=='place_cube'];held=[v for v in frames if v['phase']=='stage_6_grasp_hold'];opened=[v for v in frames if v['phase']=='opened_before_remove']
 row=dict(folder=name,status=r['status'],error=r.get('error'),all_torque_off_verified=r.get('all_torque_off_verified'),settings_restored=r.get('settings_restored'),cube_placement_confirmed=r.get('cube_placement_confirmed'),cube_removal_confirmed=r.get('cube_removal_confirmed'),physical_grasp_verified=r.get('physical_grasp_verified'),cube_present=r.get('cube_present'),health_extrema=health,photos=photo_keys,camera=r.get('camera_recording'),phases=phases,samples=len(samples),read_failures=len(r.get('read_failures',[])),final_off_reference_valid=r.get('final_off_reference_valid'),last_frame=frames[-1] if frames else None,result_sha256=hashlib.sha256((f/'result.json').read_bytes()).hexdigest())
 if hold:
  rows_actual=np.asarray([v['actual'] for v in hold]);currents=np.asarray([v['current_ma'] for v in hold]);row['grasp_hold']=dict(samples=len(hold),actual_first=rows_actual[0].tolist(),actual_last=rows_actual[-1].tolist(),actual_span_ticks=np.ptp(rows_actual,axis=0).tolist(),current_median_ma=np.median(currents,axis=0).tolist(),max_tracking_rad=max(max(v['tracking_rad']) for v in hold),sample_keys=list(hold[0]))
 row['selected_frames']={phase:[str(f/'camera'/v['image']) for v in group[::max(1,len(group)//5)][:5]] for phase,group in (('place',placed[-20:]),('hold',held),('open',opened))}
 rows.append(row)
(O/'run_review.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(rows,ensure_ascii=False,indent=2))
