"""Offline cube rotation from green-grid geometry indexed by the same ID5.

Image row sorting alone would lose90deg rotations. The decoded ID5 orientation
labels sticker-grid axes consistently across every view. Only saved data used.
"""
import json,math
from pathlib import Path
import cv2 as cv,numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'horizontal_quarter_trial_20261028_012305_894520'
OUT=ROOT/'right_hand_rotation_20261028/completed_quarter_012305';OUT.mkdir(parents=True,exist_ok=True)
KDATA=np.load(ROOT/'camera1_720.npz');K,DIST=KDATA['K'],KDATA['dist']
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),required_cube_count=1)
MARKER=np.float32([[-.5,.5],[.5,.5],[.5,-.5],[-.5,-.5]])
PITCH=.055/3;RATIO=PITCH/.0107

def read(path):return cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)
def fit(path):
    im=read(path);gray=cv.cvtColor(im,cv.COLOR_BGR2GRAY);corners,ids,_=det.detectMarkers(gray)
    found={} if ids is None else {int(i):np.asarray(c).reshape(4,2) for i,c in zip(ids.ravel(),corners)}
    result=dict(image=str(path.relative_to(FOLDER)),ids=list(found))
    if 5 not in found:return result
    q=found[5]
    hsv=cv.cvtColor(im,cv.COLOR_BGR2HSV);mask=cv.inRange(hsv,np.array([35,55,40]),np.array([100,255,255]))
    roi=np.zeros(mask.shape,np.uint8);roi[45:290,300:940]=255;mask=cv.bitwise_and(mask,roi)
    n,labels,stats,centers=cv.connectedComponentsWithStats(mask);assigned={};valid=[]
    for i in range(1,n):
        x,y,w,h,area=stats[i]
        if not (150<=area<=8000 and 12<=w<=150 and 7<=h<=110):continue
        valid.append(i)
    clean=np.isin(labels,valid).astype(np.uint8)
    ys,xs=np.nonzero(clean)
    if len(xs)<100:return result
    hull=cv.convexHull(np.column_stack([xs,ys]).astype(np.float32));quad=None
    for fraction in (.01,.015,.02,.025,.03,.04):
        approx=cv.approxPolyDP(hull,fraction*cv.arcLength(hull,True),True)
        if len(approx)==4:quad=approx.reshape(4,2);break
    if quad is None:return result
    center=quad.mean(axis=0);quad=quad[np.argsort(np.arctan2(quad[:,1]-center[1],quad[:,0]-center[0]))]
    quad=np.roll(quad,-int(np.argmin(quad.sum(axis=1))),axis=0)
    H=cv.getPerspectiveTransform(quad,np.float32([[-1,1],[1,1],[1,-1],[-1,-1]]))
    markerlocal=cv.perspectiveTransform(q.astype(np.float32)[None,:,:],H)[0]
    edge=markerlocal[1]-markerlocal[0];turn=int(np.rint(np.arctan2(edge[1],edge[0])/(np.pi/2)))
    angle=-turn*np.pi/2;rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    for i in valid:
        point=centers[i];local=rotation@cv.perspectiveTransform(np.float32(point)[None,None,:],H)[0,0]/(2/3)
        key=tuple(np.rint(local).astype(int));error=float(np.linalg.norm(local-key))
        if key==(0,0) or max(abs(v) for v in key)>1 or error>.45:continue
        if key not in assigned or error<assigned[key][0]:assigned[key]=(error,point)
    result['sticker_count']=len(assigned)
    if len(assigned)<6:return result
    keys=sorted(assigned);obj=np.float64([[x*PITCH,y*PITCH,0.] for x,y in keys]);img=np.float64([assigned[k][1] for k in keys])
    undistorted=cv.undistortPoints(img[:,None,:],K,DIST,P=K).reshape(-1,2)
    _,inliers=cv.findHomography(obj[:,:2],undistorted,cv.RANSAC,1.5,maxIters=5000,confidence=.999)
    if inliers is None or int(inliers.sum())<6:return result
    keep=inliers.ravel().astype(bool);obj=obj[keep];img=img[keep];keys=[k for k,used in zip(keys,keep) if used]
    result.update(green_face_quad=quad.tolist(),grid_inliers=int(keep.sum()),marker_axis_quarter_index=turn)
    ok,rvs,tvs,errs=cv.solvePnPGeneric(obj,img,K,DIST,flags=cv.SOLVEPNP_IPPE)
    candidates=[]
    for rv,tv in zip(rvs,tvs):
        rv,tv=cv.solvePnPRefineLM(obj,img,K,DIST,rv,tv)
        projection=cv.projectPoints(obj,rv,tv,K,DIST)[0].reshape(-1,2)
        error=float(np.sqrt(np.mean(np.sum((projection-img)**2,axis=1))))
        if float(tv[2,0])>0:candidates.append(dict(rotation=cv.Rodrigues(rv)[0].tolist(),translation=tv.ravel().tolist(),rmse_px=error))
    if candidates:
        best=min(candidates,key=lambda p:p['rmse_px']);result.update(best)
        result.update(grid_keys=[[int(v) for v in k] for k in keys],grid_centers_px=img.tolist(),marker5_corners=q.tolist())
    return result

def relative(b,a):
    rb=np.array(b['rotation']);ra=np.array(a['rotation']);r=rb.T@ra;rv=cv.Rodrigues(r)[0].ravel();theta=np.linalg.norm(rv)
    qz=(rv[2]/theta*math.sin(theta/2)) if theta>1e-10 else 0.;qw=math.cos(theta/2)
    return dict(twist_deg=math.degrees(2*math.atan2(qz,qw)),total_deg=math.degrees(theta),
        up_axis_change_deg=math.degrees(math.acos(float(np.clip(rb[:,2]@ra[:,2],-1,1)))))

def main():
    run=json.loads((FOLDER/'result.json').read_bytes());frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    selected=[]
    for phase in ('loaded_countdown','stage_6_grasp_hold','stage_8_horizontal_hold','stage_10_remove_hold'):
        rows=[f for f in frames if f['phase']==phase][-18:]
        selected.extend(rows)
    results=[]
    for index,f in enumerate(selected):
        row=fit(FOLDER/'camera'/f['image']);row.update(phase=f['phase'],camera_read_monotonic=f['camera_read_monotonic']);results.append(row)
    accepted=[v for v in results if v.get('rmse_px',100)<2.]
    comparisons={}
    for name,phase0,phase1 in (('placement_to_turn','loaded_countdown','stage_8_horizontal_hold'),
        ('grasp_to_turn','stage_6_grasp_hold','stage_8_horizontal_hold'),
        ('placement_to_open','loaded_countdown','stage_10_remove_hold')):
        before=[v for v in accepted if v['phase']==phase0];after=[v for v in accepted if v['phase']==phase1]
        values=[relative(b,a) for b in before for a in after]
        item=dict(before_frames=len(before),after_frames=len(after),pairs=len(values))
        if values:
            for key in ('twist_deg','total_deg','up_axis_change_deg'):item[key+'_p05_median_p95']=np.percentile([v[key] for v in values],[5,50,95]).tolist()
        comparisons[name]=item
    samples=run['samples']
    report=dict(source=FOLDER.name,status='saved_grid_relative_orientation_diagnostic',comparisons=comparisons,
        fitted_frames=len(accepted),requested_frames=len(selected),fits=results,
        marker5_identity_axes_preserved=True,whole_readonly_analysis=True,
        all_torque_off_verified=run['all_torque_off_verified'],settings_restored=run['settings_restored'],
        minimum_voltage_v=min(min(v['voltage_v']) for v in samples),
        peak_current_ma=max(max(abs(x) for x in v['current_ma']) for v in samples),
        maximum_tracking_rad=max(max(v['tracking_rad']) for v in samples),
        read_failures=len(run.get('read_failures',[])),all_recorded_read_failures_recovered=all(v['recovered'] for v in run.get('read_failures',[])),
        certified_robot_registration=False,precise_world_yaw_calibration_verified=False,
        limitation='Relative green-face pose uses saved camera calibration and sticker centroids. Camera/body movement and planar ambiguity must also be checked. Values are estimates, not a certified angle tolerance.',motor_writes=0)
    if any(v['pairs']==0 for v in comparisons.values()):
        report.update(status='rejected_incomplete_green_grid_fit',used_for_angle_claim=False)
    (OUT/'grid_rotation_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='fits'}))
if __name__=='__main__':main()
