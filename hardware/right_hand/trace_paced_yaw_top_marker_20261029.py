"""Same GREEN side marker orientation in saved images; provisional camera estimate.

The RED top marker did not supply five poses at the existing1px threshold.
Use full3D relative rotation of ID5, not twist about the side marker normal.
"""
import bisect,hashlib,json,math
from pathlib import Path
import cv2 as cv,numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent;F=ROOT/'paced_retained_yaw_20261029_044827_659675';O=F/'offline_review'
r=json.loads((F/'result.json').read_bytes());frames=json.loads((F/'camera/frames.json').read_bytes())['frames']
cal=np.load(ROOT/'camera1_720.npz');K,D=cal['K'],cal['dist']
obj=np.float64([[-.00535,.00535,0],[.00535,.00535,0],[.00535,-.00535,0],[-.00535,-.00535,0]])
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),required_cube_count=1)
def angle(a,b):return math.degrees(float(np.linalg.norm(cv.Rodrigues(a.T@b)[0])))
def candidates(corners):
    good,rs,ts,_=cv.solvePnPGeneric(obj,corners.astype(np.float64),K,D,flags=cv.SOLVEPNP_IPPE_SQUARE);rows=[]
    for rv,tv in zip(rs,ts):
        if float(tv[2,0])<=0:continue
        projected=cv.projectPoints(obj,rv,tv,K,D)[0].reshape(4,2);err=float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=1))))
        if err<1.:rows.append(dict(R=cv.Rodrigues(rv)[0],rmse=err))
    return sorted(rows,key=lambda x:x['rmse'])
def detect(f):
    im=cv.imdecode(np.frombuffer((F/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    cs,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY));out={}
    if ids is not None:
        for c,i in zip(cs,ids.ravel()):
            if int(i)==5:out[5]=candidates(np.asarray(c).reshape(4,2))
    return out.get(5,[])
baseline=[]
for f in [x for x in frames if x['phase']=='stage_6_grasp_hold'][-12:]:
    c=detect(f)
    if c:baseline.append(c[0]['R'])
if len(baseline)<5:raise RuntimeError('Insufficient GREEN side marker baseline')
B=min(baseline,key=lambda a:float(np.median([angle(a,b) for b in baseline])));prior=B
selected=[x for x in frames if x['phase']=='stage_7_horizontal_yaw'];selected=selected[::3]+selected[-3:]
times=[s['t']+r['enabled_at'] for s in r['samples']];rows=[]
for f in selected:
    found=detect(f);row=dict(image=f['image'],phase=f['phase'],accepted=False)
    choices=[]
    for c in found:
        tilt=math.degrees(math.acos(float(np.clip(B[:,2]@c['R'][:,2],-1,1))))
        if angle(prior,c['R'])<15:choices.append(dict(c,tilt=tilt,step=angle(prior,c['R'])))
    if choices:
        c=min(choices,key=lambda x:x['step']+.5*x['rmse']);v=cv.Rodrigues(B.T@c['R'])[0].ravel();theta=np.linalg.norm(v)
        qz=v[2]/theta*math.sin(theta/2) if theta>1e-9 else 0.;twist=math.degrees(2*math.atan2(qz,math.cos(theta/2)))
        i=max(0,bisect.bisect_right(times,f['camera_read_monotonic'])-1);s=r['samples'][i]
        row.update(accepted=True,camera_relative_full_rotation_deg=angle(B,c['R']),side_normal_rotation_deg=c['tilt'],rmse_px=c['rmse'],
            t=f['camera_read_monotonic']-r['enabled_at'],id12_goal=s['goal'][11],id12_actual=s['actual'][11]);prior=c['R']
    rows.append(row)
valid=[v for v in rows if v['accepted']]
late=[v for v in valid if v['t']>=valid[-1]['t']-1.] if valid else []
report=dict(status='provisional_same_green_ID5_camera_trace',source=F.name,baseline_frames=len(baseline),
    source_result_sha256=hashlib.sha256((F/'result.json').read_bytes()).hexdigest(),accepted_frames=len(valid),selected_frames=len(rows),
    reference='grasp hold before yaw',robot_axis_angle_certified=False,real_robot_cube_registration_verified=False,
    marker_size_mm=10.7,calibration='camera1_720.npz',marker_pose_branch_ambiguity_remaining=True,
    red_top_marker_trace_rejected_for_insufficient_reliable_poses=True,
    last_valid_frame=valid[-1]['image'] if valid else None,last_valid_t=valid[-1]['t'] if valid else None,
    last_eight_valid_time_range_s=[valid[-8]['t'],valid[-1]['t']] if len(valid)>=8 else None,
    last_eight_median_angle_deg=float(np.median([v['camera_relative_full_rotation_deg'] for v in valid[-8:]])) if valid else None,
    final_window_seconds=1.,final_window_accepted_frames=len(late),
    final_median_angle_deg=float(np.median([v['camera_relative_full_rotation_deg'] for v in late])) if late else None,
    final_median_side_normal_change_deg=float(np.median([v['side_normal_rotation_deg'] for v in valid[-8:]])) if valid else None,rows=rows,motor_writes=0)
(O/'side_marker_rotation_trace.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
old=json.loads((ROOT/'calibrated_quarter_trial_20261028_174723_390729/result.json').read_bytes())
oldyaw=[s for s in old['samples'] if s['phase']=='stage_7_horizontal_yaw'];newyaw=[s for s in r['samples'] if s['phase']=='stage_7_horizontal_yaw']
comparison=[]
for index in range(0,min(255,len(oldyaw)),25):
    a=oldyaw[index];b=newyaw[min(index*2,len(newyaw)-1)]
    comparison.append(dict(reference_yaw_index=index,old_goal=a['goal'][11],old_actual=a['actual'][11],old_current=a['current_ma'][11],
        new_goal=b['goal'][11],new_actual=b['actual'][11],new_current=b['current_ma'][11]))
(O/'successful_vs_current_ID12.json').write_text(json.dumps(dict(earlier_run='calibrated_quarter_trial_20261028_174723_390729',rows=comparison),indent=2)+'\n',encoding='utf-8')
print('ID12_COMPARISON',json.dumps(comparison))
print('SUCCESSFUL_ID12_PEAK',json.dumps(dict(error_ticks=max(abs(s['goal'][11]-s['actual'][11]) for s in old['samples']),
    current_ma=max(abs(s['current_ma'][11]) for s in old['samples']),min_voltage_v=min(s['voltage_v'][11] for s in old['samples']))))
