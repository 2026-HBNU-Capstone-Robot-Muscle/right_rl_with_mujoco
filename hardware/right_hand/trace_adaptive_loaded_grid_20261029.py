"""ID5-labelled green sticker geometry and fixed-palm image checks, offline."""
import hashlib,json,math
from pathlib import Path
import cv2 as cv,numpy as np
from marker_detector import MarkerDetector
from measure_successful_quarter_20261028 import relative
ROOT=Path(__file__).resolve().parent;F=ROOT/'adaptive_yaw_loaded_20261029_160451_416563';OUT=F/'offline_review';OUT.mkdir(exist_ok=True)
cal=np.load(ROOT/'camera1_720.npz');K,D=cal['K'],cal['dist']
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),required_cube_count=1)
unit=np.float32([[-.5,.5],[.5,.5],[.5,-.5],[-.5,-.5]])
pitch=.055/3;ratio=pitch/.0107;extent=.055/.0107*.5
def fit(path):
    im=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)
    corners,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
    found={} if ids is None else {int(i):np.asarray(c).reshape(4,2) for i,c in zip(ids.ravel(),corners)}
    row=dict(image=path.name,ids=list(found),accepted=False)
    if 40 in found:row['palm40_corners']=found[40].tolist()
    if 5 not in found:return row
    q=found[5].astype(np.float32);H=cv.getPerspectiveTransform(q,unit)
    # A projected face ROI follows this marker; it excludes unrelated green background.
    local=np.float32([[-extent,extent],[extent,extent],[extent,-extent],[-extent,-extent]])*1.15
    boundary=cv.perspectiveTransform(local[None],np.linalg.inv(H))[0]
    if not np.isfinite(boundary).all():return row
    roi=np.zeros(im.shape[:2],np.uint8);cv.fillConvexPoly(roi,np.rint(boundary).astype(np.int32),255)
    hsv=cv.cvtColor(im,cv.COLOR_BGR2HSV);mask=cv.bitwise_and(cv.inRange(hsv,np.array([35,55,40]),np.array([100,255,255])),roi)
    n,labels,stats,centers=cv.connectedComponentsWithStats(mask);assigned={}
    for i in range(1,n):
        x,y,w,h,area=stats[i]
        if not (150<=area<=8000 and 12<=w<=150 and 7<=h<=110):continue
        local=cv.perspectiveTransform(np.float32(centers[i])[None,None],H)[0,0]/ratio
        key=tuple(np.rint(local).astype(int));err=float(np.linalg.norm(local-key))
        if key==(0,0) or max(abs(v) for v in key)>1 or err>.45:continue
        if key not in assigned or err<assigned[key][0]:assigned[key]=(err,centers[i])
    row['sticker_count']=len(assigned)
    if len(assigned)<6:return row
    keys=sorted(assigned);obj=np.float64([[x*pitch,y*pitch,0] for x,y in keys]);img=np.float64([assigned[k][1] for k in keys])
    undistorted=cv.undistortPoints(img[:,None],K,D,P=K).reshape(-1,2)
    _,inliers=cv.findHomography(obj[:,:2],undistorted,cv.RANSAC,1.5,maxIters=5000,confidence=.999)
    if inliers is None or int(inliers.sum())<6:return row
    keep=inliers.ravel().astype(bool);obj=obj[keep];img=img[keep]
    good,rs,ts,_=cv.solvePnPGeneric(obj,img,K,D,flags=cv.SOLVEPNP_IPPE);candidates=[]
    for rv,tv in zip(rs,ts):
        if float(tv[2,0])<=0:continue
        rv,tv=cv.solvePnPRefineLM(obj,img,K,D,rv,tv)
        proj=cv.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)
        error=float(np.sqrt(np.mean(np.sum((proj-img)**2,axis=1))))
        if error<2.:candidates.append(dict(rotation=cv.Rodrigues(rv)[0].tolist(),rmse_px=error))
    if candidates:row.update(min(candidates,key=lambda x:x['rmse_px']),accepted=True,grid_inliers=int(keep.sum()))
    return row
def main():
    run=json.loads((F/'result.json').read_bytes());frames=json.loads((F/'camera/frames.json').read_bytes())['frames'];selected=[]
    for phase in ('loaded_countdown','stage_5_loaded_grasp','stage_6_grasp_hold','stage_7_horizontal_yaw','stage_8_horizontal_hold','stage_9_loaded_open','stage_10_remove_hold'):
        items=[f for f in frames if f['phase']==phase]
        selected.extend(items[::5] if phase in ('stage_5_loaded_grasp','stage_7_horizontal_yaw','stage_9_loaded_open') else items)
    selected.sort(key=lambda f:f['camera_read_monotonic']);rows=[]
    for i,f in enumerate(selected):
        row=fit(F/'camera'/f['image']);row.update(phase=f['phase'],t=f['camera_read_monotonic']-run['enabled_at']);rows.append(row)
        if i%100==0:print(json.dumps(dict(processed=i,selected=len(selected))),flush=True)
    comparisons={}
    for name,phase0,phase1 in (('placement_to_turn','loaded_countdown','stage_8_horizontal_hold'),('grasp_to_turn','stage_6_grasp_hold','stage_8_horizontal_hold'),('placement_to_open','loaded_countdown','stage_10_remove_hold')):
        a=[v for v in rows if v['phase']==phase0 and v['accepted']];b=[v for v in rows if v['phase']==phase1 and v['accepted']]
        values=[relative(x,y) for x in a for y in b];c=dict(before_frames=len(a),after_frames=len(b),pairs=len(values))
        if values:
            for key in ('twist_deg','total_deg','up_axis_change_deg'):c[key+'_p05_median_p95']=np.percentile([v[key] for v in values],[5,50,95]).tolist()
        comparisons[name]=c
    palm=[v for v in rows if 'palm40_corners' in v];start=np.median([v['palm40_corners'] for v in palm if v['phase']=='loaded_countdown'],axis=0)
    shifts=[float(np.sqrt(np.mean(np.sum((np.asarray(v['palm40_corners'])-start)**2,axis=1)))) for v in palm]
    report=dict(status='offline_marker_labelled_grid_trace',source=F.name,source_result_sha256=hashlib.sha256((F/'result.json').read_bytes()).hexdigest(),
        grid_RMSE_limit_px=2.,minimum_grid_inliers=6,source_images_unchanged=True,comparisons=comparisons,
        fitted_frames=sum(v['accepted'] for v in rows),selected_frames=len(rows),palm_corner_check_frames=len(palm),
        palm_corner_RMS_displacement_median_p95_max_px=np.percentile(shifts,[50,95,100]).tolist() if shifts else None,
        camera_and_robot_motion_not_precisely_calibrated=True,absolute_robot_axis_angle_not_certified=True,
        distribution_is_frame_spread_not_absolute_accuracy=True,rows=rows,motor_writes=0)
    (OUT/'marker_labelled_grid_trace.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
if __name__=='__main__':main()
