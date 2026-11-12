"""Use decoded OrangeID3 for the initial face when topID5 is unreadable.

The Orange/Green frame relation is learned ONLY from simultaneous same-run
grid fits. No missing ID5 decode is invented or used by a hardware gate.
"""
import json,math
from pathlib import Path
import numpy as np,cv2 as cv
from marker_detector import MarkerDetector
from measure_inverse_side_marker_20261029 import K,D,poses
from measure_successful_quarter_20261028 import relative
ROOT=Path(__file__).resolve().parent;F=ROOT/'yaw_roundtrip_trial_20261030_011416_269699';O=F/'offline_review'
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),required_cube_count=1)
def distance(a,b):return float(np.degrees(np.linalg.norm(cv.Rodrigues(np.array(a).T@np.array(b))[0])))
def mean_rotation(rotations):
    arrays=np.array(rotations);medoid=min(range(len(arrays)),key=lambda i:np.median([distance(arrays[i],b) for b in arrays]))
    keep=[r for r in arrays if distance(arrays[medoid],r)<8.]
    if len(keep)<3:raise RuntimeError('Insufficient consistent same-run two-face grid fits')
    u,_,v=np.linalg.svd(np.sum(keep,axis=0));r=u@np.diag([1.,1.,np.linalg.det(u@v)])@v
    return r,keep
def orange_fit(path):
    im=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR);cs,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
    found={} if ids is None else {int(i):np.asarray(c).reshape(4,2) for i,c in zip(ids.ravel(),cs)};row=dict(image=path.name,ids=list(found),accepted=False)
    if 3 not in found:return row,found
    q=found[3].astype(np.float32);unit=np.float32([[-.5,.5],[.5,.5],[.5,-.5],[-.5,-.5]]);H=cv.getPerspectiveTransform(q,unit)
    extent=.055/.0107*.5*1.15;boundary=cv.perspectiveTransform(np.float32([[-extent,extent],[extent,extent],[extent,-extent],[-extent,-extent]])[None],np.linalg.inv(H))[0]
    roi=np.zeros(im.shape[:2],np.uint8);cv.fillConvexPoly(roi,np.rint(boundary).astype(np.int32),255)
    hsv=cv.cvtColor(im,cv.COLOR_BGR2HSV);mask=cv.bitwise_and(cv.inRange(hsv,np.array([0,70,60]),np.array([25,255,255])),roi)
    n,labels,stats,centers=cv.connectedComponentsWithStats(mask);assigned={};pitch=.055/3;ratio=pitch/.0107
    for i in range(1,n):
        x,y,w,h,area=stats[i]
        if not 150<=area<=8000 or not 12<=w<=150 or not 7<=h<=110:continue
        p=cv.perspectiveTransform(np.float32(centers[i])[None,None],H)[0,0]/ratio;key=tuple(np.rint(p).astype(int));err=float(np.linalg.norm(p-key))
        if key==(0,0) or max(abs(v) for v in key)>1 or err>.45:continue
        if key not in assigned or err<assigned[key][0]:assigned[key]=(err,centers[i])
    row['sticker_count']=len(assigned)
    if len(assigned)<6:return row,found
    keys=sorted(assigned);obj=np.float64([[x*pitch,y*pitch,0.] for x,y in keys]);points=np.float64([assigned[k][1] for k in keys]);und=cv.undistortPoints(points[:,None],K,D,P=K).reshape(-1,2)
    _,inl=cv.findHomography(obj[:,:2],und,cv.RANSAC,1.5,maxIters=5000,confidence=.999)
    if inl is None or inl.sum()<6:return row,found
    valid=inl.ravel().astype(bool);obj=obj[valid];points=points[valid];_,rs,ts,_=cv.solvePnPGeneric(obj,points,K,D,flags=cv.SOLVEPNP_IPPE);candidates=[]
    for rv,tv in zip(rs,ts):
        if tv[2,0]<=0:continue
        rv,tv=cv.solvePnPRefineLM(obj,points,K,D,rv,tv);proj=cv.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2);err=float(np.sqrt(np.mean(np.sum((proj-points)**2,axis=1))))
        if err<2:candidates.append(dict(rotation=cv.Rodrigues(rv)[0].tolist(),rmse_px=err))
    if candidates:row.update(min(candidates,key=lambda a:a['rmse_px']),accepted=True)
    return row,found
def main():
    primary=json.loads((O/'physical_and_angle_review.json').read_bytes());fitrows=[];relations=[];side_relations={'forward_hold':[],'inverse_hold':[]}
    for g in primary['rows']:
        if 3 not in g.get('ids',[]) and g['leg_label'] not in ('forward_hold','inverse_hold'):continue
        result,found=orange_fit(F/'camera'/g['image']);result.update(leg_label=g['leg_label'],t=g['t']);fitrows.append(result)
        if result.get('accepted') and g.get('accepted'):
            relation=np.array(result['rotation']).T@np.array(g['rotation']);relations.append(relation)
        if g['leg_label'] in side_relations and g.get('accepted') and 0 in found:
            up=np.array(g['rotation'])[:,2];options=poses(found[0],.0107);near=[p for p in options if p['rmse_px']<=options[0]['rmse_px']+.25] if options else []
            if near:
                best=min(near,key=lambda p:math.acos(float(np.clip((p['rotation']@[-1.,0,0])@up,-1,1))))
                err=math.degrees(math.acos(float(np.clip((best['rotation']@[-1.,0,0])@up,-1,1))))
                if err<12:side_relations[g['leg_label']].append(best['rotation'].T@np.array(g['rotation']))
    if len(relations)<3:
        (O/'orange_grid_transfer_rejection.json').write_text(json.dumps(dict(status='rejected_insufficient_simultaneous_orange_green_grid_fits',
            simultaneous_grid_pairs=len(relations),orange_grid_rows=fitrows,used_for_angle_claim=False,motor_writes=0),indent=2)+'\n',encoding='utf-8')
        print('REJECTED: simultaneous grid calibration insufficient; no angle claim.');return
    relation,consistent=mean_rotation(relations)
    if abs(relation[2,2])>.2:raise RuntimeError('Learned Orange/Green face normals are not near perpendicular')
    before=[dict(r,rotation=(np.array(r['rotation'])@relation).tolist()) for r in fitrows if r['leg_label']=='placed' and r.get('accepted')]
    after=[r for r in primary['rows'] if r['leg_label']=='forward_hold' and r.get('accepted')]
    if len(before)<3 or len(after)<3:raise RuntimeError('Before/after frame support insufficient for placed-to-forward claim')
    values=[relative(a,b) for a in before for b in after];comparisons={key:np.percentile([v[key] for v in values],[5,50,95]).tolist() for key in ('twist_deg','total_deg','up_axis_change_deg')}
    layers=[distance(a,b) for a in side_relations['forward_hold'] for b in side_relations['inverse_hold']]
    report=dict(status='same_run_orange3_grid_to_green5_frame_transfer',source=F.name,calibration_raw_pairs=len(relations),calibration_consistent_pairs=len(consistent),
        green_in_orange_rotation=relation.tolist(),calibration_spread_deg_p50_p95=np.percentile([distance(relation,x) for x in consistent],[50,95]).tolist(),
        placed_frames=len(before),forward_frames=len(after),placement_to_forward_camera_relative_deg_p05_median_p95=comparisons,
        green_white_relative_change_after_up_axis_pose_disambiguation_deg_p05_median_p95=np.percentile(layers,[5,50,95]).tolist() if layers else None,
        side_pose_green_up_axis_in_ID0=[-1,0,0],green_top_marker_not_decoded_at_placement=True,no_missing_marker_detection_fabricated=True,
        metric_robot_registration_verified=False,body_motion_not_yet_compensated=True,frame_spread_not_absolute_accuracy=True,
        source_images_unchanged=True,motor_writes=0,orange_grid_rows=fitrows)
    (O/'orange_to_green_rotation_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='orange_grid_rows'}),flush=True)
if __name__=='__main__':main()
