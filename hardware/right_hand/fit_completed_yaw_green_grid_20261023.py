"""Relative pose diagnostic from the same visible nine green sticker centers.

Uses only saved pre-removal images. No marker-to-hand calibration is claimed.
"""
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'grasped_yaw_trial_20261023_234707_886202'
BASE='stage_4_fixed_grasp_observation';AFTER='stage_6_yaw_fixed_observation'


def load(f):
    return cv.imdecode(np.frombuffer((FOLDER/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)


def main():
    r=json.loads((FOLDER/'result.json').read_bytes())
    fs=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    selected=[f for f in fs if f['phase'] in (BASE,AFTER) and f['camera_read_monotonic']<=r['rotation_evidence_cutoff_camera_monotonic']]
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    pitch=.055/3
    objects=np.array([[x,y,0.] for y in (pitch,0.,-pitch) for x in (-pitch,0.,pitch)],np.float64)
    fits=[]
    for f in selected:
        im=load(f);hsv=cv.cvtColor(im,cv.COLOR_BGR2HSV)
        mask=cv.inRange(hsv,np.array([35,55,35]),np.array([100,255,255]))
        roi=np.zeros(mask.shape,np.uint8);roi[75:225,480:870]=255;mask=cv.bitwise_and(mask,roi)
        n,labels,stats,centers=cv.connectedComponentsWithStats(mask)
        blobs=[]
        for i in range(1,n):
            x,y,w,h,area=stats[i]
            if 500<=area<=6000 and 25<=w<=120 and 15<=h<=65:blobs.append(centers[i])
        row=dict(image=f['image'],phase=f['phase'],blobs=len(blobs))
        if len(blobs)!=9:fits.append(row);continue
        b=np.asarray(blobs);b=b[np.argsort(b[:,1])]
        grid=np.vstack([tier[np.argsort(tier[:,0])] for tier in np.split(b,3)])
        count,rvs,tvs,errs=cv.solvePnPGeneric(objects,grid,K,dist,flags=cv.SOLVEPNP_IPPE)
        candidates=[]
        for rv,tv in zip(rvs,tvs):
            rv,tv=cv.solvePnPRefineLM(objects,grid,K,dist,rv,tv)
            projection=cv.projectPoints(objects,rv,tv,K,dist)[0].reshape(-1,2)
            e=np.linalg.norm(projection-grid,axis=1)
            candidates.append(dict(R=cv.Rodrigues(rv)[0],t=tv.ravel(),rmse=float(np.sqrt(np.mean(e**2)))))
        best=min(candidates,key=lambda x:x['rmse'])
        row.update(grid_centers_px=grid.tolist(),rotation=best['R'].tolist(),top_center_camera_m=best['t'].tolist(),rmse_px=best['rmse'])
        fits.append(row)
    good=[f for f in fits if f.get('rmse_px',100)<2.]
    before=[f for f in good if f['phase']==BASE][-12:];after=[f for f in good if f['phase']==AFTER][-10:]
    if len(before)<5 or len(after)<5:
        rejected=dict(status='rejected_incomplete_or_inconsistent_green_sticker_grid',
            baseline_accepted_frames=len(before),after_accepted_frames=len(after),fits=fits,
            physical_rotation_verified=False,robot_registration_verified=False)
        (FOLDER/'green_grid_relative_motion.json').write_text(json.dumps(rejected,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in rejected.items() if k!='fits'},indent=2))
        return
    relative=[]
    for b in before:
        for a in after:
            rotation=np.array(b['rotation']).T@np.array(a['rotation'])
            rv=cv.Rodrigues(rotation)[0].ravel()
            relative.append(dict(local_rotation_vector_deg=np.degrees(rv).tolist(),
                total_rotation_deg=float(np.degrees(np.linalg.norm(rv))),
                top_center_translation_camera_mm=((np.array(a['top_center_camera_m'])-b['top_center_camera_m'])*1000).tolist()))
    # Check static palm features for camera motion. These features are not a decoded ID40.
    b0=load(next(f for f in selected if f['image']==before[-1]['image']))
    a0=load(next(f for f in selected if f['image']==after[-1]['image']))
    feature_mask=np.zeros((720,1280),np.uint8);feature_mask[395:640,375:570]=255
    sift=cv.SIFT_create(nfeatures=1000)
    kp0,de0=sift.detectAndCompute(cv.cvtColor(b0,cv.COLOR_BGR2GRAY),feature_mask)
    kp1,de1=sift.detectAndCompute(cv.cvtColor(a0,cv.COLOR_BGR2GRAY),feature_mask)
    matches=cv.BFMatcher().knnMatch(de0,de1,k=2)
    matches=[m for m,n in matches if m.distance<.65*n.distance]
    camera=dict(ratio_matches=len(matches),direct_ID40_decode=False,registration_verified=False)
    if len(matches)>=10:
        p=np.float32([kp0[m.queryIdx].pt for m in matches]);q=np.float32([kp1[m.trainIdx].pt for m in matches])
        H,inliers=cv.findHomography(p,q,cv.RANSAC,1.5)
        if H is not None:
            valid=inliers.ravel().astype(bool);error=np.linalg.norm(q[valid]-p[valid],axis=1)
            camera.update(inliers=int(valid.sum()),matched_feature_shift_median_px=float(np.median(error)),
                matched_feature_shift_95_px=float(np.percentile(error,95)),homography=H.tolist())
    rotations=np.array([x['local_rotation_vector_deg'] for x in relative])
    translations=np.array([x['top_center_translation_camera_mm'] for x in relative])
    result=dict(status='same_scene_green_grid_relative_pose_diagnostic',cube_side_m=.055,
        baseline_frames=len(before),after_frames=len(after),fits=fits,
        local_rotation_vector_median_deg=np.median(rotations,axis=0).tolist(),
        local_rotation_vector_percentiles_5_95_deg=np.percentile(rotations,[5,95],axis=0).tolist(),
        total_rotation_deg_percentiles=np.percentile([x['total_rotation_deg'] for x in relative],[5,50,95]).tolist(),
        top_center_translation_camera_median_mm=np.median(translations,axis=0).tolist(),
        static_palm_feature_camera_check=camera,physical_rotation_verified=False,
        robot_registration_verified=False,live_control_allowed=False,
        limitation='Green sticker centroid geometry, glare, camera calibration and planar pose ambiguity can bias small rotation estimates. This is not a certified target-direction measurement.')
    (FOLDER/'green_grid_relative_motion.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='fits'},indent=2))


if __name__=='__main__':main()
