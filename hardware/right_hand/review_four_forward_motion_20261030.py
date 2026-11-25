"""Read saved four attempts, compare actual goal packets and visible Cube motion."""
import json,hashlib
from pathlib import Path
import numpy as np
from single_forward_plan_20261030 import SingleForwardPlan
from trace_adaptive_loaded_grid_20261029 import fit
from measure_inverse_side_marker_20261029 import poses
from measure_successful_quarter_20261028 import relative
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/four_forward_review'
def compare(a,b):
 values=[relative(x,y) for x in a for y in b];out=dict(before=len(a),after=len(b))
 for k in ('twist_deg','total_deg','up_axis_change_deg'):
  out[k+'_p05_median_p95']=np.percentile([v[k] for v in values],[5,50,95]).tolist() if values else None
 return out
def main():
 reports=[]
 for F in sorted(R.glob('single_forward_trial_*'))[-4:]:
  r=json.loads((F/'result.json').read_bytes());frames=json.loads((F/'camera/frames.json').read_bytes())['frames'];out=F/'offline_review';out.mkdir(exist_ok=True)
  ss=r.get('samples',[])
  if not ss:continue
  p=SingleForwardPlan(F/'fresh_off_request.json');rows=p.plan(np.asarray(r['accepted_enable_goal'],int));expected=[q.tolist() for _,q in rows];written=[e['requested'] for e in r['write_events'] if e['address']==116]
  matches=[]
  for skip in range(min(12,len(written))):
   v=written[skip:]
   if len(v)<=len(expected) and v==expected[:len(v)]:matches.append((skip,len(v)))
  if not matches:raise RuntimeError('Actual queue prefix mismatch '+F.name)
  skip,n=max(matches,key=lambda x:x[1]);selected=[]
  for phase in ('loaded_countdown','stage_6_grasp_hold','stage_7_horizontal_yaw','stage_8_horizontal_hold','stage_10_remove_hold','opened_before_remove'):
   ff=[(i,f) for i,f in enumerate(frames) if f['phase']==phase];stride=8 if phase=='stage_7_horizontal_yaw' else 2
   selected.extend(ff[::stride][:90])
  selected.sort();fits=[]
  for i,f in selected:
   v=fit(F/'camera'/f['image']);v.update(phase=f['phase'],frame_index=i,t=f['camera_read_monotonic']-r['enabled_at'])
   if v.get('accepted') and 'palm40_corners' in v:
    pp=poses(np.array(v['palm40_corners']),.0282)
    if pp:v['body_rotation']=(pp[0]['rotation'].T@np.array(v['rotation'])).tolist()
   fits.append(v)
  groups={name:[v for v in fits if v['phase'] in phases and v.get('accepted')] for name,phases in {'placed':['loaded_countdown'],'grasp':['stage_6_grasp_hold'],'turn_hold':['stage_8_horizontal_hold'],'open':['stage_10_remove_hold','opened_before_remove']}.items()}
  comparisons={}
  for name,a,b in (('placed_to_grasp','placed','grasp'),('placed_to_turn','placed','turn_hold'),('grasp_to_turn','grasp','turn_hold'),('turn_to_open','turn_hold','open')):
   comparisons[name]=dict(camera_relative=compare(groups[a],groups[b]),decoded_palm_compensated=compare([dict(v,rotation=v['body_rotation']) for v in groups[a] if 'body_rotation' in v],[dict(v,rotation=v['body_rotation']) for v in groups[b] if 'body_rotation' in v]))
  hold_stats={}
  for phase in ('stage_6_grasp_hold','stage_8_horizontal_hold'):
   q=[s for s in ss if s['phase']==phase][-20:]
   if q:hold_stats[phase]=dict(median_current_ma=np.median([v['current_ma'] for v in q],axis=0).tolist(),median_actual_ticks=np.median([v['actual'] for v in q],axis=0).tolist(),goal=q[-1]['goal'],median_error_ticks=(np.median([v['actual'] for v in q],axis=0)-q[-1]['goal']).tolist())
  report=dict(status='saved_forward_attempt_review',source=F.name,execution_status=r['status'],error=r.get('error'),goal_prefix_matches=True,goal_writes=n,expected_goals=len(expected),initial_preload_writes_excluded=skip,
   full_queue_completed=n==len(expected),all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],comparisons=comparisons,hold_stats=hold_stats,
   marker_grid_accepted=sum(v.get('accepted',False) for v in fits),source_images_unchanged=True,metric_registration_verified=False,current_not_calibrated_contact_force=True,assistant_motor_writes=0,rows=fits)
  (out/'forward_motion_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');reports.append({k:v for k,v in report.items() if k!='rows'});print(json.dumps({k:v for k,v in report.items() if k not in ('rows','hold_stats')}),flush=True)
 (O/'motion_comparison.json').write_text(json.dumps(reports,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
