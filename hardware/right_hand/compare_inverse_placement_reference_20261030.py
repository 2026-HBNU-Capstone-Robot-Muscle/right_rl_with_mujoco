"""Compare the forward end and inverse start in the SAME rigid palm frame.

All metric estimates retain marker-size/camera uncertainty. These are NOT
registered robot coordinates and cannot approve a motor path.
"""
import json,hashlib
from pathlib import Path
import cv2 as cv,numpy as np
from measure_inverse_side_marker_20261029 import poses,K,D,marker_points
from marker_detector import MarkerDetector
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[0],hand_id=40,required_cube_count=1)
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261030/placement_comparison'
CASES=[('forward_end','adaptive_yaw_loaded_20261029_160451_416563','stage_8_horizontal_hold'),
       ('inverse_placed','gapped_inverse_trial_20261029_184252_272166','loaded_countdown'),
       ('inverse_closed','gapped_inverse_trial_20261029_184252_272166','stage_6_grasp_hold'),
       ('new32_placed','supported_inverse_trial_20261029_231345_341471','loaded_countdown')]
def tracked_palm(folder,frames,target_index,image):
    gray=cv.cvtColor(image,cv.COLOR_BGR2GRAY)
    for prior_index in range(target_index-1,max(-1,target_index-1200),-20):
        source=ROOT/folder/'camera'/frames[prior_index]['image'];before=cv.imdecode(np.frombuffer(source.read_bytes(),np.uint8),cv.IMREAD_GRAYSCALE)
        cs,ids,_=det.detectMarkers(before)
        if ids is None or 40 not in ids:continue
        corners=np.asarray(cs[list(ids.ravel()).index(40)]).reshape(4,2);mask=np.zeros(before.shape,np.uint8);cv.fillConvexPoly(mask,np.rint(corners).astype(np.int32),255)
        old=cv.goodFeaturesToTrack(before,100,.01,3,mask=mask)
        if old is None or len(old)<8:continue
        new,st,_=cv.calcOpticalFlowPyrLK(before,gray,old,None,winSize=(31,31),maxLevel=4)
        back,bs,_=cv.calcOpticalFlowPyrLK(gray,before,new,None,winSize=(31,31),maxLevel=4)
        valid=st.ravel().astype(bool)&bs.ravel().astype(bool)&(np.linalg.norm(old-back,axis=2).ravel()<1.)
        if valid.sum()<8 or cv.contourArea(cv.convexHull(new[valid]))<300:continue
        und=cv.undistortPoints(old[valid].astype(float),K,D,P=K).astype(np.float32)
        xy=cv.perspectiveTransform(und,marker_points(corners)).reshape(-1,2);obj=np.c_[xy,np.zeros(len(xy))].astype(float)
        ok,rv,tv,inl=cv.solvePnPRansac(obj,new[valid].astype(float),K,D,iterationsCount=150,reprojectionError=1.5,confidence=.999,flags=cv.SOLVEPNP_ITERATIVE)
        if not ok or inl is None or len(inl)<8:continue
        ii=inl.ravel();rv,tv=cv.solvePnPRefineLM(obj[ii],new[valid][ii].astype(float),K,D,rv,tv);projected=cv.projectPoints(obj[ii],rv,tv,K,D)[0]
        error=float(np.sqrt(np.mean(np.sum((projected-new[valid][ii])**2,axis=2))))
        if error>1 or tv[2,0]<=0:continue
        h=.0282/2;quad=cv.projectPoints(np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]]),rv,tv,K,D)[0].reshape(4,2)
        return dict(rotation=cv.Rodrigues(rv)[0],translation=tv.ravel(),rmse_px=error,method='offline_LK_not_decoded',source=source.relative_to(ROOT).as_posix()),quad
    return None,None
def main():
    OUT.mkdir(parents=True,exist_ok=True);rows=[];groups={}
    for name,folder,phase in CASES:
        frames=json.loads((ROOT/folder/'camera/frames.json').read_bytes())['frames'];selected=[f for f in frames if f['phase']==phase][-30::3];items=[]
        for f in selected:
            path=ROOT/folder/'camera'/f['image'];im=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)
            cs,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY));found={} if ids is None else {int(i):np.asarray(c).reshape(4,2) for i,c in zip(ids.ravel(),cs)}
            if 0 not in found:continue
            white=poses(found[0],.0107);palm=poses(found[40],.0282) if 40 in found else []
            if palm:palm[0]['method']='decoded_ID40';quad=found[40]
            else:
                tracked,quad=tracked_palm(folder,frames,frames.index(f),im);palm=[tracked] if tracked else []
            if not white or not palm:continue
            p,w=palm[0],white[0];R=p['rotation'];C=w['translation']-w['rotation'][:,2]*.0275
            rel=R.T@(C-p['translation']);rr=R.T@w['rotation']
            row=dict(case=name,image=path.relative_to(ROOT).as_posix(),palm_relative_cube_center_mm=(rel*1000).tolist(),
                     relative_white_rotation=rr.tolist(),palm_rotation=R.tolist(),palm_translation=p['translation'].tolist(),
                     white_rotation=w['rotation'].tolist(),white_translation=w['translation'].tolist(),
                     palm_corners=quad.tolist(),palm_method=p['method'],side_center_pixels=found[0].mean(0).tolist(),RMSE=[p['rmse_px'],w['rmse_px']])
            items.append(row);rows.append(row)
        groups[name]=items
    if not groups['forward_end']:raise RuntimeError('No decoded or independently tracked forward ID0+ID40 reference')
    reference=groups['forward_end'][len(groups['forward_end'])//2];reference_center=np.median([r['palm_relative_cube_center_mm'] for r in groups['forward_end']],axis=0)
    summaries=[]
    for name,items in groups.items():
        if not items:summaries.append(dict(case=name,frames=0));continue
        values=np.array([r['palm_relative_cube_center_mm'] for r in items]);delta=values-reference_center
        pixel=[]
        for r in items:
            H=cv.getPerspectiveTransform(np.array(r['palm_corners'],np.float32),np.array(reference['palm_corners'],np.float32))
            p=cv.perspectiveTransform(np.array(r['side_center_pixels'],np.float32)[None,None,:],H)[0,0]
            pixel.append((p-np.array(reference['side_center_pixels'])).tolist())
        # Common marker-scale errors are varied together in both frames.
        sensitivity=[]
        for s0 in (.0105,.0107):
          for s40 in (.0278,.0286):
            def recalc(r):
                R=np.array(r['palm_rotation']);C=np.array(r['white_translation'])*(s0/.0107)-np.array(r['white_rotation'])[:,2]*.0275
                return R.T@(C-np.array(r['palm_translation'])*(s40/.0282))*1000
            base=np.median([recalc(r) for r in groups['forward_end']],0)
            d=np.median([recalc(r) for r in items],0)-base;sensitivity.append(dict(side_mm=s0*1000,palm_mm=s40*1000,delta_palm_frame_mm=d.tolist()))
        summaries.append(dict(case=name,frames=len(items),median_center_mm=np.median(values,0).tolist(),
            delta_from_forward_end_mm=np.median(delta,0).tolist(),delta_in_reference_camera_mm=(np.array(reference['palm_rotation'])@np.median(delta,0)).tolist(),
            homologous_palm_plane_side_marker_shift_pixels=np.median(pixel,0).tolist(),common_size_sensitivity=sensitivity,
            spread_p05_p95_mm=np.percentile(delta,[5,95],axis=0).tolist()))
    r=dict(status='marker_and_explicit_LK_placement_comparison_not_robot_registration',rigid_reference_marker=40,rigid_marker_assumed_mm=28.2,
        tracked_points_not_decoded_or_execution_evidence=True,cases=summaries,rows=rows,
        metric_accuracy_not_certified=True,palm_homography_has_height_parallax=True,side_marker_size_not_exactly_identified=True,
        hardware_execution_allowed=False,devices_opened=False)
    (OUT/'review.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in r.items() if k!='rows'}))
if __name__=='__main__':main()
