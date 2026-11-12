"""Actual loaded result and provisional planar-marker orientation analysis."""
import hashlib,json,sys
from pathlib import Path
import cv2 as cv,numpy as np
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'loaded_quarter_trial_20261025_171607_019752'
OUT=ROOT/'right_hand_rotation_20261024/actual_loaded_review_171607';OUT.mkdir(exist_ok=True)
r=json.loads((SOURCE/'result.json').read_bytes());samples=r['samples'];turn=[s for s in samples if s['phase']=='stage_6_loaded_quarter_turn']
summary=dict(source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256((SOURCE/'result.json').read_bytes()).hexdigest(),
    status=r['status'],all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
    cube_placement_confirmed=r.get('cube_placement_confirmed'),cube_removal_confirmed=r.get('cube_removal_confirmed'),
    full_loaded_command_samples=len(turn),maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),
    peak_current_ma=max(max(abs(x) for x in s['current_ma']) for s in samples),
    min_voltage_v=min(min(s['voltage_v']) for s in samples),max_temperature_c=max(max(s['temperature_c']) for s in samples),
    user_observation='Cube appeared caught in a groove at approximately 45deg, not flipped; recollection approximate.',
    physical_ninety_degree_rotation_verified=False,rotation_task_success=False,continuous_loaded_command_completed=True)
empty=np.load(ROOT/'right_hand_rotation_20261024/completed_empty_path_review_20261025/goal_indexed_actual_path.npz')
goals=empty['goal_ticks'];e=empty['actual_ticks'];diff=[];lastidx=0
for s in turn:
    ids=np.flatnonzero(np.all(goals==s['goal'],axis=1));ids=ids[ids>=lastidx]
    if not len(ids):continue
    index=int(ids[0]);lastidx=index;diff.append(np.asarray(s['actual'])-e[index])
summary['loaded_minus_prior_empty_median_ticks_by_id']=np.median(diff,axis=0).tolist()
summary['loaded_minus_prior_empty_maximum_absolute_ticks_by_id']=np.max(abs(np.asarray(diff)),axis=0).tolist()
calpath=ROOT/'camera1_720.npz'
with np.load(calpath) as cal:K=np.asarray(cal['K'],np.float64);dist=np.asarray(cal['dist'],np.float64);print('CALIB',cal.files,K.tolist())
sys.path.insert(0,str(ROOT.parent/'work/Leap_Test/vision/rubik_center'));import cube_board
rot=tuple(json.loads((ROOT/'cube_face_rot_result.json').read_bytes())['face_rot'])
obj,ids=cube_board.object_points(cube_mm=55.,marker_mm=10.7,layout='centers',face_rot=rot)
board=cv.aruco.Board(obj,cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),ids)
params=cv.aruco.DetectorParameters();params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_SUBPIX
det=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),params)
def detect(path):
    im=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR);cs,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
    return {int(i):c for i,c in zip(ids.ravel(),cs)} if ids is not None else {}
def solve(visible,selected,intrinsic,flag):
    cs=[visible[i] for i in selected];obj,img=board.matchImagePoints(cs,np.asarray(selected,np.int32).reshape(-1,1))
    obj=np.asarray(obj,np.float64);img=np.asarray(img,np.float64)
    if flag==cv.SOLVEPNP_IPPE:
        _,rvs,tvs,_=cv.solvePnPGeneric(obj,img,intrinsic,dist,flags=flag)
    else:
        ok,rv,tv=cv.solvePnP(obj,img,intrinsic,dist,flags=flag)
        if not ok:return []
        rvs=[rv];tvs=[tv]
    out=[]
    for rv,tv in zip(rvs,tvs):
        pr,_=cv.projectPoints(obj,rv,tv,intrinsic,dist);err=float(np.linalg.norm(pr.reshape(-1,2)-img.reshape(-1,2),axis=1).mean())
        out.append(dict(R=cv.Rodrigues(rv)[0],t=tv.ravel(),reprojection_px=err))
    return sorted(out,key=lambda s:s['reprojection_px'])
before=detect(SOURCE/'cube_before_turn.jpg');after=detect(SOURCE/'cube_after_turn.jpg');poses=[]
for scale in (.8,1.,1.2):
    intrinsic=K.copy();intrinsic[0,0]*=scale;intrinsic[1,1]*=scale
    ref=solve(before,[3,5],intrinsic,cv.SOLVEPNP_SQPNP)[0]
    single_before=solve(before,[5],intrinsic,cv.SOLVEPNP_IPPE)
    same_face_ref=min(single_before,key=lambda s:np.linalg.norm(cv.Rodrigues(s['R']@ref['R'].T)[0]))
    candidates=solve(after,[5],intrinsic,cv.SOLVEPNP_IPPE)
    for n,pose in enumerate(candidates):
        angle=float(np.linalg.norm(cv.Rodrigues(pose['R']@ref['R'].T)[0])*180/np.pi)
        same_face_angle=float(np.linalg.norm(cv.Rodrigues(pose['R']@same_face_ref['R'].T)[0])*180/np.pi)
        poses.append(dict(focal_scale=scale,branch=n,reference_reprojection_px=ref['reprojection_px'],after_reprojection_px=pose['reprojection_px'],
            estimated_orientation_change_deg=angle,same_green_marker_change_deg=same_face_angle,
            before_green_reprojection_px=same_face_ref['reprojection_px'],after_rotation=pose['R'].tolist(),after_translation=pose['t'].tolist()))
summary.update(provisional_marker_poses=poses,calibration_source=calpath.name,calibration_camera_identity_reverified_this_run=False,
    camera_or_base_motion_compensated=False,after_single_face_pose_is_provisional=True)
(OUT/'loaded_completion_review.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False))
