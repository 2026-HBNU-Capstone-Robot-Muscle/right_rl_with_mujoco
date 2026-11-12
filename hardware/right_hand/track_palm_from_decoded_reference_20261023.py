"""Offline planar reference tracking; never counted as directly decoded ID40.

Only palm-image features are matched. Current paired encoder readings are
retained. The old image provides a visual palm template, never joint values.
"""
import hashlib
import itertools
import json
from pathlib import Path

import cv2 as cv
import numpy as np

from hand_registration_geometry import candidate
from review_prepared_cube_20261022 import marker_solutions

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_pose_readonly_20261023_165647_488518'
REFERENCE=ROOT/'camera_exposure_sweep_20261023_161815_427701'


def read(path):
    return cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)


def main():
    source=(FOLDER/'result.json').read_bytes();trial=json.loads(source)
    exposure=json.loads((REFERENCE/'exposure_review.json').read_bytes())
    photos=next(p for p in exposure['profiles'] if p['name']=='manual_2')['details']
    refs=[r for r in photos if 40 in r['ids']]
    bounds=np.array([[375,390],[566,390],[594,626],[355,636]],np.int32)
    mask=np.zeros((720,1280),np.uint8);cv.fillPoly(mask,[bounds],255)
    sift=cv.SIFT_create(nfeatures=1000)
    def features(image):return sift.detectAndCompute(cv.cvtColor(image,cv.COLOR_BGR2GRAY),mask)
    for ref in refs:
        ref['features']=features(read(REFERENCE/ref['image']))
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    reg=candidate();Rwm=np.array(reg['hand_to_world_candidate']);gm=np.array(reg['grasp_site_in_marker_m_candidate'])
    matcher=cv.BFMatcher();rows=[];hypotheses=[]
    for pair in trial['pairs']:
        keypoints,desc=features(read(FOLDER/pair['image']));tracked=[]
        for ref in refs:
            kp0,de0=ref['features'];matches=matcher.knnMatch(de0,desc,k=2)
            good=[a for a,b in matches if a.distance<.65*b.distance]
            if len(good)<18:continue
            a=np.float32([kp0[m.queryIdx].pt for m in good]);b=np.float32([keypoints[m.trainIdx].pt for m in good])
            held=np.arange(len(good))%3==0
            H,inliers=cv.findHomography(a[~held],b[~held],cv.RANSAC,1.5)
            if H is None:continue
            projected=cv.perspectiveTransform(a[held,None,:],H).reshape(-1,2)
            error=np.linalg.norm(projected-b[held],axis=1)
            training_a=a[~held][inliers.ravel().astype(bool)]
            span=np.ptp(training_a,axis=0) if len(training_a) else np.zeros(2)
            accepted=bool(inliers.sum()>=12 and np.sum(error<1.5)>=6 and np.mean(error<1.5)>=.7
                and np.median(error)<1 and span[0]>100 and span[1]>100)
            if not accepted:continue
            old=np.array(ref['corners']['40'],np.float32)
            corners=cv.perspectiveTransform(old[:,None,:],H).reshape(4,2)
            tracked.append(dict(reference_image=ref['image'],reference_image_sha256=hashlib.sha256((REFERENCE/ref['image']).read_bytes()).hexdigest(),
                descriptor_ratio_matches=len(good),ransac_training_inliers=int(inliers.sum()),
                heldout_matches=len(error),heldout_median_error_px=float(np.median(error)),
                heldout_good_fraction=float(np.mean(error<1.5)),training_inlier_span_px=span.tolist(),
                corners_marker40_px=corners.tolist(),homography=H.tolist()))
        spread=None
        if len(tracked)>=3:
            all_corners=np.array([r['corners_marker40_px'] for r in tracked])
            mean=np.median(all_corners,axis=0)
            spread=float(np.max(np.linalg.norm(all_corners-mean,axis=2)))
            if spread<2:
                p40=marker_solutions([mean],[40],40,.0282,K,dist)
                p5=marker_solutions([np.array(pair['markers']['corners']['5'])],[5],5,.0107,K,dist)
                for (ip,(Rp,tp,ep)),(ic,(Rc,tc,ec)) in itertools.product(enumerate(p40),enumerate(p5)):
                    center=tc-Rc[:,2]*.0275
                    Rcw=Rp@Rwm.T;tcw=tp+Rp@gm-Rcw@np.array([.11,0,.03])
                    hypotheses.append(dict(image=pair['image'],palm_branch=ip,cube_branch=ic,
                        marker_reprojection_sum_px=ep+ec,
                        cube_center_marker40_m=(Rp.T@(center-tp)).tolist(),
                        cube_top_rotation_marker40=(Rp.T@Rc).tolist(),
                        cube_center_world_m=(Rcw.T@(center-tcw)).tolist(),cube_rotation_world=(Rcw.T@Rc).tolist(),
                        world_to_camera_rotation=Rcw.tolist(),world_to_camera_translation=tcw.tolist(),
                        measured_motor_ticks=pair['before']['snapshot']['position_ticks'],
                        palm_reference_source='offline_planar_feature_tracking_from_4_decoded_templates',
                        directly_decoded_palm_in_current_frame=False,
                        old_joint_values_used=False))
        rows.append(dict(image=pair['image'],tracked_references=tracked,
            maximum_reference_corner_disagreement_px=spread))
        print(pair['image'],'validated refs',len(tracked),'corner disagreement',spread,flush=True)
    result=dict(status='offline_palm_template_tracking_diagnostic',source_sha256=hashlib.sha256(source).hexdigest(),
        source_encoder_pairs=len(trial['pairs']),decoded_reference_count=len(refs),
        palm_plane_feature_mask_px=bounds.tolist(),tracking_rows=rows,hypotheses=hypotheses,
        directly_decoded_current_palm_frames=0,old_joint_values_used=False,
        marker_to_model_calibration_verified=False,hardware_execution_allowed=False,
        servo_writes_sent=0,real_camera_or_serial_opened=False,
        limitation='Visual planar tracking is a diagnostic, not a new decoded marker or a verified marker-to-model transform. Reflection changes and both planar pose branches remain possible.')
    (FOLDER/'palm_template_tracking.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    if hypotheses:
        centers=np.array([h['cube_center_marker40_m'] for h in hypotheses if h['palm_branch']==h['cube_branch']==0])
        print('Primary branch median cube center in marker40 mm:',np.median(centers,axis=0)*1000,flush=True)
    print('Diagnostic hypotheses',len(hypotheses),flush=True)


if __name__=='__main__':main()
