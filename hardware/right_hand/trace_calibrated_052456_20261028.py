"""Offline orientation trace; do not command a robot from a saved estimate."""
import bisect,json,math
from pathlib import Path
import cv2 as cv,numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'calibrated_quarter_trial_20261028_052456_066430'
OUT=ROOT/'right_hand_rotation_20261028/completed_calibrated_052456';OUT.mkdir(exist_ok=True)
r=json.loads((FOLDER/'result.json').read_bytes());all_frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
cal=np.load(ROOT/'camera1_720.npz');K,D=cal['K'],cal['dist']
obj=np.float64([[-.00535,.00535,0],[.00535,.00535,0],[.00535,-.00535,0],[-.00535,-.00535,0]])
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),required_cube_count=1)
def poses(corners):
    good,rs,ts,errs=cv.solvePnPGeneric(obj,corners.astype(np.float64),K,D,flags=cv.SOLVEPNP_IPPE_SQUARE)
    result=[]
    for rv,tv in zip(rs,ts):
        if float(tv[2,0])<=0:continue
        projected=cv.projectPoints(obj,rv,tv,K,D)[0].reshape(4,2)
        error=float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=1))))
        if error<1.:result.append(dict(R=cv.Rodrigues(rv)[0],rmse=error))
    return sorted(result,key=lambda v:v['rmse'])
def rotation_angle(a,b):return float(np.linalg.norm(cv.Rodrigues(a.T@b)[0]))
def twist(a,b):
    v=cv.Rodrigues(a.T@b)[0].ravel();theta=np.linalg.norm(v)
    qz=v[2]/theta*math.sin(theta/2) if theta>1e-9 else 0.
    return math.degrees(2*math.atan2(qz,math.cos(theta/2)))
def detection(frame):
    im=cv.imdecode(np.frombuffer((FOLDER/'camera'/frame['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    corners,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
    return {} if ids is None else {int(i):poses(np.asarray(c).reshape(4,2)) for i,c in zip(ids.ravel(),corners) if int(i)<6}
before=[f for f in all_frames if f['phase']=='loaded_countdown'][-18:]
baseline=[]
for frame in before:
    found=detection(frame)
    if found.get(5):baseline.append(found[5][0]['R'])
if len(baseline)<5:raise RuntimeError('Too few reliable initial green-marker observations')
B=min(baseline,key=lambda a:float(np.median([rotation_angle(a,b) for b in baseline])))
phases=('stage_5_loaded_grasp','stage_6_grasp_hold','stage_7_horizontal_yaw','stage_8_horizontal_hold','stage_9_loaded_open','stage_10_remove_hold')
selected=[]
for phase in phases:
    items=[f for f in all_frames if f['phase']==phase]
    selected.extend(items[::4] if phase in ('stage_5_loaded_grasp','stage_7_horizontal_yaw','stage_9_loaded_open') else items[-8:])
selected.sort(key=lambda f:f['camera_read_monotonic'])
times=[s['t']+r['enabled_at'] for s in r['samples']];yaw_indices=[i for i,s in enumerate(r['samples']) if s['phase']=='stage_7_horizontal_yaw']
prior=B;last_stamp=before[-1]['camera_read_monotonic'];rows=[]
for n,frame in enumerate(selected):
    stamp=frame['camera_read_monotonic'];found=detection(frame);candidates=found.get(5,[])
    row=dict(image=frame['image'],phase=frame['phase'],time_after_enabled_s=stamp-r['enabled_at'],ids=list(found),accepted=False)
    if candidates:
        for c in candidates:
            c['step']=math.degrees(rotation_angle(prior,c['R']))
            c['tilt']=math.degrees(math.acos(float(np.clip(B[:,2]@c['R'][:,2],-1,1))))
        possible=[c for c in candidates if c['tilt']<25 and c['step']<max(15,12*(stamp-last_stamp))]
        if possible:
            c=min(possible,key=lambda p:p['step']+.5*p['rmse']);angle=twist(B,c['R'])
            i=max(0,bisect.bisect_right(times,stamp)-1);sample=r['samples'][i]
            row.update(accepted=True,angle_deg=angle,top_tilt_deg=c['tilt'],rmse_px=c['rmse'],sample_index=i,
                command_ticks=sample['goal'],actual_ticks=sample['actual'],
                yaw_goals_already_observed=bisect.bisect_left(yaw_indices,i) if sample['phase']=='stage_7_horizontal_yaw' else None)
            prior=c['R'];last_stamp=stamp
    rows.append(row)
    if n%80==0:print(json.dumps(dict(processed=n,total=len(selected))),flush=True)
valid=[v for v in rows if v['accepted']]
yaw=[v for v in valid if v['phase']=='stage_7_horizontal_yaw']
targets={str(target):min(yaw,key=lambda p:abs(p['angle_deg']-target)) for target in (80,85,90,95,100)} if yaw else {}
report=dict(source=FOLDER.name,status='offline_camera_relative_trace',baseline_frames=len(baseline),
    estimated_from_same_green_ID5=True,certified_robot_frame=False,
    branch_selection='positive depth, reprojection<1px, continuity and top-normal tilt<25deg',
    rows=rows,accepted_frames=len(valid),total_selected_frames=len(rows),
    closest_observed_targets=targets,holding_motors_after_truncation_not_yet_reviewed=True,
    cannot_be_used_as_motor_command_or_proof=True,motor_writes=0)
(OUT/'rotation_trace.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
print(json.dumps(dict(accepted=len(valid),total=len(rows),
    targets={k:dict(angle=v['angle_deg'],yaw_prefix=v['yaw_goals_already_observed'],image=v['image']) for k,v in targets.items()})))
