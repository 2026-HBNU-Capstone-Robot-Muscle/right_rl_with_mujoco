"""Camera-derived object progress versus exact indexed actual target packets."""
import json,math
from pathlib import Path
import numpy as np
from measure_roundtrip_orange_grid_20261030 import mean_rotation
from measure_successful_quarter_20261028 import relative
R=Path(__file__).resolve().parent;F=R/'yaw_roundtrip_trial_20261030_011416_269699';O=F/'offline_review'
def main():
 r=json.loads((F/'result.json').read_bytes());pr=json.loads((O/'physical_and_angle_review.json').read_bytes());trans=json.loads((O/'side3_to_green_rotation_review.json').read_bytes())
 base,_=mean_rotation([np.array(x['rotation']) for x in trans['before']]);reference={'rotation':base.tolist()}
 z=np.load(R/'right_hand_rotation_20261030/yaw_roundtrip/candidate.npz');end=int(z['forward_end_sample']);indices=np.flatnonzero(z['phases'][:end]=='stage_7_horizontal_yaw');q=z['goals'][indices];lookup={tuple(v):i+1 for i,v in enumerate(q)}
 lookup[tuple(z['goals'][np.flatnonzero(z['phases']=='stage_6_grasp_hold')[-1]])]=0
 data={'forward_yaw':[],'inverse_yaw':[]};seen=False;inverse=False
 for s in r['samples']:
  if s['phase']=='stage_7_horizontal_yaw' and seen:inverse=True
  if s['phase']=='stage_8_horizontal_hold':seen=True
  if s['phase']!='stage_7_horizontal_yaw':continue
  leg='inverse_yaw' if inverse else 'forward_yaw';index=lookup.get(tuple(s['goal']))
  if index is not None:data[leg].append(dict(s,reference_index=index,progress=(len(q)-index if inverse else index)/len(q)))
 trace=[]
 for row in pr['rows']:
  leg=row['leg_label']
  if leg not in data or not row.get('accepted'):continue
  sample=min(data[leg],key=lambda s:abs(s['t']-row['t']));lag=abs(sample['t']-row['t'])
  if lag>.25:continue
  trace.append(dict(image=row['image'],leg=leg,t=row['t'],progress=sample['progress'],reference_index=sample['reference_index'],angle_deg=relative(reference,row)['twist_deg'],time_distance_s=lag,
      up_axis_change_deg=relative(reference,row)['up_axis_change_deg'],current_ma=sample['current_ma'],tracking_rad=sample['tracking_rad'],goal=sample['goal'],actual=sample['actual']))
 summaries={}
 for leg in data:
  ff=[x for x in trace if x['leg']==leg];bins=[]
  for a,b in zip(np.arange(0,1,.2),np.arange(.2,1.01,.2)):
   aa=[x for x in ff if a<=x['progress']<=b]
   if aa:bins.append(dict(progress_range=[float(a),float(b)],frames=len(aa),angle_median_deg=float(np.median([v['angle_deg'] for v in aa])),angle_min_max_deg=[float(min(v['angle_deg'] for v in aa)),float(max(v['angle_deg'] for v in aa))]))
  summaries[leg]=dict(frames=len(ff),bins=bins)
 a=[v for v in trace if v['leg']=='inverse_yaw' and v['progress']>=.75];start=min(a,key=lambda v:v['progress']);stop=max(a,key=lambda v:v['progress'])
 goal_travel=np.abs(np.array(start['goal'])-stop['goal']);actual_travel=np.abs(np.array(start['actual'])-stop['actual'])
 report=dict(status='actual_object_progress_compared_to_exact_goal_index',camera_relative_only=True,nearest_motor_sample_max_seconds=.25,source=F.name,
   actual_queue_previously_verified=True,summary=summaries,last_inverse_quarter=dict(camera_frames=len(a),observed_first_last_angle_deg=[start['angle_deg'],stop['angle_deg']],observed_angle_change_deg=stop['angle_deg']-start['angle_deg'],goal_travel_ticks=goal_travel.tolist(),actual_travel_ticks=actual_travel.tolist()),
   force_not_calibrated=True,contact_cause_not_identified_from_current_alone=True,rows=trace,devices_opened=False)
 (O/'object_motor_progress_trace.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
if __name__=='__main__':main()
