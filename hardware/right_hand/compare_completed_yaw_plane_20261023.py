"""Saved optical-flow comparison of cube top plane and static palm features."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'grasped_yaw_trial_20261023_234707_886202'


def load(name):
    return cv.imdecode(np.frombuffer((FOLDER/name).read_bytes(),np.uint8),cv.IMREAD_COLOR)


def track(a,b,mask):
    p=cv.goodFeaturesToTrack(a,maxCorners=400,qualityLevel=.01,minDistance=4,mask=mask)
    q,status,_=cv.calcOpticalFlowPyrLK(a,b,p,None,winSize=(21,21),maxLevel=3)
    back,reverse,_=cv.calcOpticalFlowPyrLK(b,a,q,None,winSize=(21,21),maxLevel=3)
    valid=(status.ravel()==1)&(reverse.ravel()==1)&(np.linalg.norm(p-back,axis=2).ravel()<.4)
    p=p[valid].reshape(-1,2);q=q[valid].reshape(-1,2)
    H,inliers=cv.findHomography(p,q,cv.RANSAC,.7)
    assert H is not None
    keep=inliers.ravel().astype(bool);projected=cv.perspectiveTransform(p[:,None,:],H).reshape(-1,2)
    error=np.linalg.norm(projected-q,axis=1)
    return H,p[keep],q[keep],dict(tracked_points=len(p),inliers=int(keep.sum()),
        inlier_span_px=np.ptp(p[keep],axis=0).tolist(),
        residual_median_px=float(np.median(error[keep])),
        image_translation_median_px=np.median(q[keep]-p[keep],axis=0).tolist())


def main():
    a=load('baseline_last.jpg');b=load('after_last.jpg')
    a=cv.cvtColor(a,cv.COLOR_BGR2GRAY);b=cv.cvtColor(b,cv.COLOR_BGR2GRAY)
    mask=np.zeros(a.shape,np.uint8)
    cv.fillPoly(mask,[np.array([[558,126],[778,126],[819,204],[568,215]],np.int32)],255)
    H,p,q,top=track(a,b,mask)
    palm_mask=np.zeros(a.shape,np.uint8);palm_mask[395:640,380:570]=255
    Hp,pp,pq,palm=track(a,b,palm_mask)
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    pu=cv.undistortPoints(p[:,None,:],K,dist,P=K).reshape(-1,2)
    qu=cv.undistortPoints(q[:,None,:],K,dist,P=K).reshape(-1,2)
    Hu,_=cv.findHomography(pu,qu,0)
    n,rotations,translations,normals=cv.decomposeHomographyMat(Hu,K)
    candidates=[]
    for R,t,normal in zip(rotations,translations,normals):
        rv=cv.Rodrigues(R)[0].ravel()
        candidates.append(dict(total_rotation_deg=float(np.degrees(np.linalg.norm(rv))),
            rotation_vector_camera_deg=np.degrees(rv).tolist(),plane_normal=normal.ravel().tolist(),
            scaled_translation=t.ravel().tolist()))
    # The ambiguity is kept, not resolved by claiming a calibrated palm frame.
    result=dict(source=FOLDER.name,status='pre_removal_cube_plane_optical_flow_diagnostic',
        top_face_tracking=top,static_palm_tracking=palm,top_face_homography=H.tolist(),
        competing_homography_pose_candidates=candidates,
        target_degrees_was_model_only=-5.,physical_target_yaw_verified=False,
        actual_cube_motion_visible=True,robot_registration_verified=False,
        all_source_frames_before_removal=True,
        limitation='Small-angle homography decomposition has multiple physical solutions. Image-plane translation is reliable here; exact 3D target yaw is not certified.')
    (FOLDER/'cube_plane_optical_flow.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
