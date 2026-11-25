"""Saved ID0 poses, Green-up inferred from simultaneous ID5 grid observations.

Palm ID40 is optically tracked only for offline body-motion compensation when
its glare prevents decoding. Tracked points are never counted as decoded IDs.
No devices, motion commands, or image edits are performed.
"""
import hashlib,json,math
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent;F=ROOT/'gapped_inverse_trial_20261029_184252_272166';O=F/'offline_review'
cal=np.load(ROOT/'camera1_720.npz');K,D=cal['K'],cal['dist']
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[0,5],required_cube_count=1)

def poses(points,size):
    h=size/2;obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],float)
    ok,rs,ts,_=cv.solvePnPGeneric(obj,np.asarray(points,float),K,D,flags=cv.SOLVEPNP_IPPE_SQUARE)
    out=[]
    for rv,tv in zip(rs,ts):
        if tv[2,0]<=0:continue
        rv,tv=cv.solvePnPRefineLM(obj,np.asarray(points,float),K,D,rv,tv)
        projected=cv.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)
        error=float(np.sqrt(np.mean(np.sum((projected-points)**2,axis=1))))
        if error<=1.5:out.append(dict(rotation=cv.Rodrigues(rv)[0],translation=tv.ravel(),rmse_px=error))
    return sorted(out,key=lambda p:p['rmse_px'])

def marker_points(points):
    h=.0282/2
    q=cv.undistortPoints(np.asarray(points,np.float64)[:,None],K,D,P=K).reshape(-1,2)
    H=cv.getPerspectiveTransform(q.astype(np.float32),np.array([[-h,h],[h,h],[h,-h],[-h,-h]],np.float32))
    return H

def twist(R,axis):
    rv=cv.Rodrigues(R)[0].ravel();theta=float(np.linalg.norm(rv))
    vector=rv/theta*math.sin(theta/2) if theta>1e-10 else rv*0
    return math.degrees(2*math.atan2(float(vector@axis),math.cos(theta/2)))

def main():
    run=json.loads((F/'result.json').read_bytes());frames=json.loads((F/'camera/frames.json').read_bytes())['frames']
    trace=json.loads((O/'marker_labelled_grid_trace.json').read_bytes())['rows']
    green={Path(r['image']).name:r for r in trace if r.get('accepted')}
    start=next(i for i,f in enumerate(frames) if f['phase']=='loaded_countdown')
    end=max(i for i,f in enumerate(frames) if f['phase']=='stage_10_remove_hold')
    rows=[];previous_gray=None;pixels=objects=None;last_side=None;lost=0
    for index in range(start,end+1,3):
        frame=frames[index];path=F/'camera'/frame['image']
        image=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR);gray=cv.cvtColor(image,cv.COLOR_BGR2GRAY)
        cs,ids,_=det.detectMarkers(gray)
        found={} if ids is None else {int(i):np.asarray(c).reshape(4,2) for i,c in zip(ids.ravel(),cs)}
        side=poses(found[0],.0107) if 0 in found else []
        row=dict(image=Path(frame['image']).name,phase=frame['phase'],t=frame['camera_read_monotonic']-run['enabled_at'],
            decoded_ids=list(found))
        if side:
            # Use continuity to disambiguate comparable planar fits only.
            near=[p for p in side if p['rmse_px']<=side[0]['rmse_px']+.25]
            best=min(near,key=lambda p:np.linalg.norm(cv.Rodrigues(last_side.T@p['rotation'])[0])) if last_side is not None else side[0]
            last_side=best['rotation'];row.update(side_rotation=best['rotation'].tolist(),side_RMSE_px=best['rmse_px'])
            g=green.get(row['image'])
            if g is not None:
                axis=best['rotation'].T@np.asarray(g['rotation'])[:,2]
                row['green_axis_in_ID0']=axis.tolist()
        # Track only features inside the already decoded rigid palm marker.
        if previous_gray is not None and pixels is not None and len(pixels)>=8:
            advanced,st,err=cv.calcOpticalFlowPyrLK(previous_gray,gray,pixels,None,winSize=(21,21),maxLevel=3)
            back,bs,_=cv.calcOpticalFlowPyrLK(gray,previous_gray,advanced,None,winSize=(21,21),maxLevel=3)
            valid=st.ravel().astype(bool)&bs.ravel().astype(bool)&(np.linalg.norm(back-pixels,axis=2).ravel()<1.)
            pixels=advanced[valid];objects=objects[valid]
        if 40 in found:
            q=found[40];mask=np.zeros(gray.shape,np.uint8);cv.fillConvexPoly(mask,np.rint(q).astype(np.int32),255)
            features=cv.goodFeaturesToTrack(gray,80,.01,3,mask=mask)
            if features is not None and len(features)>=8:
                pixels=features
                und=cv.undistortPoints(pixels.astype(np.float64),K,D,P=K).astype(np.float32)
                xy=cv.perspectiveTransform(und,marker_points(q)).reshape(-1,2)
                objects=np.column_stack([xy,np.zeros(len(xy))]).astype(np.float64)
            palm=poses(q,.0282)
            if palm:row.update(palm_rotation=palm[0]['rotation'].tolist(),palm_method='decoded_ID40',palm_RMSE_px=palm[0]['rmse_px'])
        elif pixels is not None and len(pixels)>=8:
            hull_area=cv.contourArea(cv.convexHull(pixels.astype(np.float32)))
            if hull_area>=300:
                ok,rv,tv,inliers=cv.solvePnPRansac(objects,pixels.astype(np.float64),K,D,iterationsCount=150,
                    reprojectionError=1.5,confidence=.999,flags=cv.SOLVEPNP_ITERATIVE)
                if ok and inliers is not None and len(inliers)>=8:
                    keep=inliers.ravel();rv,tv=cv.solvePnPRefineLM(objects[keep],pixels[keep].astype(np.float64),K,D,rv,tv)
                    proj=cv.projectPoints(objects[keep],rv,tv,K,D)[0]
                    error=float(np.sqrt(np.mean(np.sum((proj-pixels[keep])**2,axis=2))))
                    if error<=1. and tv[2,0]>0:
                        row.update(palm_rotation=cv.Rodrigues(rv)[0].tolist(),palm_method='offline_LK_from_decoded_ID40',
                            palm_inliers=len(keep),palm_RMSE_px=error)
        if 'palm_rotation' not in row:lost+=1
        rows.append(row);previous_gray=gray
        if len(rows)%75==0:print(json.dumps(dict(frames=len(rows),missing_palm=lost)),flush=True)
    axes=[np.asarray(r['green_axis_in_ID0']) for r in rows if r['phase']=='stage_8_horizontal_hold' and 'green_axis_in_ID0' in r]
    if len(axes)<3:raise RuntimeError('Insufficient simultaneous white marker / green-grid axis observations')
    axis=np.mean(axes,axis=0);axis/=np.linalg.norm(axis)
    axis_spread=[math.degrees(math.acos(float(np.clip(a@axis,-1,1)))) for a in axes]
    comparisons={}
    for name,before,after in (('placement_to_turn','loaded_countdown','stage_8_horizontal_hold'),
        ('grasp_to_turn','stage_6_grasp_hold','stage_8_horizontal_hold'),
        ('placement_to_open','loaded_countdown','stage_10_remove_hold')):
        a=[r for r in rows if r['phase']==before and 'side_rotation' in r]
        b=[r for r in rows if r['phase']==after and 'side_rotation' in r]
        values=[];corrected=[];tilts=[]
        for x in a:
          for y in b:
            rx,ry=np.asarray(x['side_rotation']),np.asarray(y['side_rotation'])
            values.append(twist(rx.T@ry,axis))
            if 'palm_rotation' in x and 'palm_rotation' in y:
                cx=np.asarray(x['palm_rotation']).T@rx;cy=np.asarray(y['palm_rotation']).T@ry
                corrected.append(twist(cx.T@cy,axis))
                tilts.append(math.degrees(math.acos(float(np.clip((cx@axis)@(cy@axis),-1,1)))))
        comparisons[name]=dict(before_frames=len(a),after_frames=len(b),camera_pairs=len(values),palm_pairs=len(corrected),
            camera_twist_p05_median_p95_deg=np.percentile(values,[5,50,95]).tolist() if values else None,
            palm_relative_twist_p05_median_p95_deg=np.percentile(corrected,[5,50,95]).tolist() if corrected else None,
            palm_relative_up_axis_change_p05_median_p95_deg=np.percentile(tilts,[5,50,95]).tolist() if tilts else None)
    report=dict(status='saved_same_ID0_inverse_rotation_diagnostic',source=F.name,
        source_result_sha256=hashlib.sha256((F/'result.json').read_bytes()).hexdigest(),
        green_axis_in_ID0=axis.tolist(),simultaneous_axis_observations=len(axes),
        axis_spread_p50_p95_max_deg=np.percentile(axis_spread,[50,95,100]).tolist(),comparisons=comparisons,
        missing_palm_rows=lost,rows=rows,initial_top_ID5_not_decoded=True,
        tracked_palm_is_not_counted_as_a_decoded_marker=True,certified_robot_registration=False,
        marker0_uniform_size_affects_translation_not_pose_rotation=True,
        uncertainty_not_identified_from_frame_spread=True,source_images_unchanged=True,motor_writes=0)
    (O/'side_marker_rotation_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)

if __name__=='__main__':main()
