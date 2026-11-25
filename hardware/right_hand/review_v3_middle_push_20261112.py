"""Inspect only the latest saved v3 execution; no device access."""
import hashlib
import json
from collections import Counter
from pathlib import Path
import cv2 as cv
import numpy as np
ROOT=Path(__file__).resolve().parent
RUN=ROOT/'nominal_two_turn_trial_20261112_070236_897589'
OUT=ROOT/'right_hand_rotation_20261112/nominal_cycle_actual_review/v3_middle_push';OUT.mkdir(exist_ok=True)
r=json.loads((RUN/'result.json').read_bytes());ss=r.get('samples',[])
path=ROOT/'right_hand_rotation_20261112/nominal_cycle_trial_preparation/index8_support_v3/candidate.npz'
q=np.load(path,allow_pickle=False)['goals']
enabled=r.get('enabled_at',0)
frames=json.loads((RUN/'camera/frames.json').read_bytes())['frames']
def point(s):
    matches=np.flatnonzero(np.all(q==np.asarray(s['goal']),axis=1))
    return dict(t=s['t'],phase=s['phase'],source_goal_indices_1based=(matches+1).tolist(),
                MF_tip_to_palm_IDS=[9,11,10,12],MF_actual=[s['actual'][i-1] for i in (9,11,10,12)],
                MF_goal=[s['goal'][i-1] for i in (9,11,10,12)])
row={k:r.get(k) for k in ('status','error','failed_phase','primary_motion_error','all_torque_off_verified','settings_restored','second_rotation_attempted')}
row.update(run=RUN.name,samples=len(ss),phase_counts=dict(Counter(s['phase'] for s in ss)),
           candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
           recorded_candidate_hash_present=hashlib.sha256(path.read_bytes()).hexdigest() in (RUN/'result.json').read_text(encoding='utf-8'),
           scalar_extra={k:v for k,v in r.items() if not isinstance(v,(dict,list)) and any(p in k for p in ('error','stop','version'))})
if ss:
    row['last']=point(ss[-1]);row['last_temperature_max']=max(ss[-1]['temperature_c'])
loaded=[s for s in ss if s['phase']=='stage_13_cycle_first_rotation']
picks=[]
for n in (2050,2188,2300,2400,2600,2880,3015):
    if loaded:
        s=min(loaded,key=lambda s:np.max(abs(np.asarray(s['goal'])-q[n-1])))
        if np.array_equal(s['goal'],q[n-1]):picks.append(point(s))
row['points']=picks
if loaded:
    tail=[s for s in loaded if s['t']>loaded[-1]['t']-30]
    s=min(tail,key=lambda s:abs(s['actual'][8]-2880));row['middle_ID9_near2880_on_late_run']=point(s)
det=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),cv.aruco.DetectorParameters())
start=loaded[-1]['t']-80 if loaded else 0
observations=[]
for f in [f for f in frames if start<=f['camera_read_monotonic']-enabled<=ss[-1]['t']][::5]:
    image=RUN/'camera'/f['image'];im=cv.imdecode(np.frombuffer(image.read_bytes(),np.uint8),cv.IMREAD_COLOR)
    corners,ids,_=det.detectMarkers(im);found=[]
    if ids is not None:
        for ident,pts in zip(ids.ravel(),corners):
            if 0<=int(ident)<=5 and abs(cv.contourArea(pts.reshape(4,2)))>=150:found.append(int(ident))
    observations.append(dict(t=f['camera_read_monotonic']-enabled,image=f['image'],cube_marker_ids=found))
visible=[o for o in observations if o['cube_marker_ids']]
row['last_marker_visible']=visible[-3:]
times=[f['camera_read_monotonic']-enabled for f in frames]
row['images_to_review']=[]
if visible:
    t=visible[-1]['t']
    for target in (t-2,t,t+.5,t+1,t+2,ss[-1]['t']-.2):
        j=min(range(len(times)),key=lambda j:abs(times[j]-target));f=frames[j]
        row['images_to_review'].append(dict(t=times[j],path=(RUN/'camera'/f['image']).relative_to(ROOT).as_posix(),sample=point(min(ss,key=lambda s:abs(s['t']-times[j])))))
row['fixed_time_images']=[]
for target in (236.,239.,241.,243.,245.,247.,249.):
    j=min(range(len(times)),key=lambda j:abs(times[j]-target));f=frames[j]
    row['fixed_time_images'].append(dict(t=times[j],path=(RUN/'camera'/f['image']).relative_to(ROOT).as_posix(),sample=point(min(ss,key=lambda s:abs(s['t']-times[j])))))
(OUT/'review.json').write_text(json.dumps(row,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'last':row['last'],'near2880':row['middle_ID9_near2880_on_late_run'],
                  'fixed_images':[{k:v for k,v in p.items() if k!='sample'} for p in row['fixed_time_images']]},ensure_ascii=False))
