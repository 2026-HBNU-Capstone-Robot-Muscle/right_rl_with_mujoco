"""Reproduce the rejected path mathematically; no device access."""
import json
import numpy as np
from loaded_quarter_trial_plan_20261025 import ROOT,LoadedQuarterTrialPlan,motion_goals,empty_return_goals,RING_RELEASE,LIFT,RETURN_PHASE,START_HOLD,TURN_PHASE,END_HOLD
from grasp_recovery_plan import interpolate
folder=ROOT/'loaded_quarter_trial_20261025_165036_298713'
p=LoadedQuarterTrialPlan(folder/'fresh_off_request.json');a=p.source_position();q=motion_goals()
anchor=int(np.argmin(np.max(abs(q-a),axis=1)));back=empty_return_goals(anchor);join=back[0]
release=a.copy();release[14]-=64;release[0]-=64;roots=np.array([7,11]);release[roots]+=np.minimum(64,np.maximum(0,join[roots]-a[roots]))
rows=[];interpolate(rows,a,release,RING_RELEASE);interpolate(rows,release,join,LIFT)
rows.extend((RETURN_PHASE,v.copy()) for v in back[1:]);rows.extend((START_HOLD,q[0].copy()) for _ in range(40))
rows.extend((TURN_PHASE,v.copy()) for v in q[1:]);rows.extend((END_HOLD,q[-1].copy()) for _ in range(60))
values=np.vstack([a]+[v for _,v in rows]);limits=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
lo=np.asarray(limits['lower_ticks']);hi=np.asarray(limits['upper_ticks']);bad=np.argwhere((values<lo)|(values>hi))
print('SOURCE',a.tolist(),'ANCHOR',anchor,'RELEASE',release.tolist(),'JOIN',join.tolist())
print('MAX_STEP',abs(np.diff(values,axis=0)).max(),'ROWS',len(rows),'NOMINAL',(len(rows)+20)*.05,'BUDGET',(len(rows)+40)*.06+2)
print('VIOLATIONS',len(bad))
for i,j in bad[:8]:print('BAD',int(i),int(j+1),int(values[i,j]),'BOUNDS',float(lo[j]),float(hi[j]))
r=json.loads((folder/'result.json').read_bytes());print('REPORT',{k:r.get(k) for k in ('status','failed_phase','all_torque_off_verified','settings_restored')},'WRITES',len(r.get('write_events',[])))
