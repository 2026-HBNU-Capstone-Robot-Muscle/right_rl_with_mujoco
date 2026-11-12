"""Saved frame/encoder review, with an explicit operator correction sidecar.

No devices. The raw report is preserved, including its SPACE-based removal flag.
Image-plane diagnostics do not establish calibrated 3-D target yaw.
"""
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from compare_completed_yaw_plane_20261023 import track

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'book_small_yaw_trial_20261024_215736_594865'
BASE='stage_4_book_grasp_hold';TURN='stage_5_book_small_clockwise';AFTER='stage_6_book_yaw_hold'


def load(f):
    return cv.imdecode(np.frombuffer((FOLDER/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)


def main():
    cv.setRNGSeed(20261024)
    blob=(FOLDER/'result.json').read_bytes();r=json.loads(blob)
    frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    cutoff=r['rotation_evidence_cutoff_camera_monotonic']
    groups={p:[f for f in frames if f['phase']==p and f['camera_read_monotonic']<=cutoff] for p in (BASE,TURN,AFTER)}
    if not r['rotation_command_path_completed'] or not all(groups.values()):raise ValueError('Incomplete before-OFF source')
    correction=dict(source_sha256=hashlib.sha256(blob).hexdigest(),
        user_statement='마지막엔 큐브 안빼고 스페이스 누르긴했음',
        raw_removal_flag_means_operator_SPACE_only=True,physical_cube_removal_confirmed=False,
        instructed_PLACE_REMOVE_protocol_completed=False,
        cube_retained_at_final_SPACE=True,post_OFF_cube_motion_is_success_evidence=False,
        source_result_modified=False,do_not_reuse_post_OFF_readings_as_empty_start=True)
    (FOLDER/'operator_correction.json').write_text(json.dumps(correction,ensure_ascii=False,indent=2),encoding='utf-8')
    before_samples=[s for s in r['samples'] if s['phase']==BASE][-20:]
    after_samples=[s for s in r['samples'] if s['phase']==AFTER][-20:]
    before=np.median([s['actual'] for s in before_samples],axis=0)
    after=np.median([s['actual'] for s in after_samples],axis=0)
    baseline_frame=groups[BASE][-1];after_frame=groups[AFTER][-1]
    a=cv.cvtColor(load(baseline_frame),cv.COLOR_BGR2GRAY)
    b=cv.cvtColor(load(after_frame),cv.COLOR_BGR2GRAY)
    top_mask=np.zeros(a.shape,np.uint8)
    cv.fillPoly(top_mask,[np.array([[442,121],[681,117],[722,207],[452,216]],np.int32)],255)
    palm_mask=np.zeros(a.shape,np.uint8);palm_mask[406:533,291:441]=255
    H,p,q,top=track(a,b,top_mask);Hp,pp,pq,palm=track(a,b,palm_mask)
    # The palm patch lies below the cube. Extrapolating an unconstrained
    # projective homography outside that patch spuriously amplified its noise.
    # Report translation and constrained similarity alternatives explicitly.
    camera_translation=np.median(pq-pp,axis=0)
    affine,keep=cv.estimateAffinePartial2D(pp,pq,method=cv.RANSAC,ransacReprojThreshold=.7)
    if affine is None:raise ValueError('No constrained static-palm camera fit')
    inverse=cv.invertAffineTransform(affine)
    compensated=cv.transform(q[:,None,:],inverse).reshape(-1,2)
    shift=compensated-p
    Hrelative,_=cv.findHomography(p,compensated,0)
    center=p.mean(axis=0);probe=np.float32([center,center+[100,0],center+[0,100]])
    transformed=cv.perspectiveTransform(probe[:,None,:],Hrelative).reshape(-1,2)
    x_axis=transformed[1]-transformed[0]
    angle=float(np.degrees(np.arctan2(x_axis[1],x_axis[0])))
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    pitch=.055/3
    objects=np.array([[x,y,0.] for y in (pitch,0.,-pitch) for x in (-pitch,0.,pitch)],np.float64)
    fits=[]
    for phase in (BASE,AFTER):
        for f in groups[phase][-12:]:
            im=load(f);hsv=cv.cvtColor(im,cv.COLOR_BGR2HSV)
            mask=cv.inRange(hsv,np.array([35,55,35]),np.array([100,255,255]))
            roi=np.zeros(mask.shape,np.uint8);roi[105:220,425:750]=255
            mask=cv.bitwise_and(mask,roi)
            n,labels,stats,centers=cv.connectedComponentsWithStats(mask)
            blobs=[centers[i] for i in range(1,n) if 350<=stats[i,4]<=6000
                and 25<=stats[i,2]<=120 and 12<=stats[i,3]<=65]
            row=dict(image=f['image'],phase=phase,blobs=len(blobs))
            if len(blobs)==9:
                points=np.array(blobs);points=points[np.argsort(points[:,1])]
                grid=np.vstack([tier[np.argsort(tier[:,0])] for tier in np.split(points,3)])
                count,rvs,tvs,errs=cv.solvePnPGeneric(objects,grid,K,dist,flags=cv.SOLVEPNP_IPPE)
                branches=[]
                for rv,tv in zip(rvs,tvs):
                    rv,tv=cv.solvePnPRefineLM(objects,grid,K,dist,rv,tv)
                    error=np.linalg.norm(cv.projectPoints(objects,rv,tv,K,dist)[0].reshape(-1,2)-grid,axis=1)
                    branches.append(dict(rotation=cv.Rodrigues(rv)[0].tolist(),top_center_camera_m=tv.ravel().tolist(),
                        rmse_px=float(np.sqrt(np.mean(error**2)))))
                best=min(branches,key=lambda s:s['rmse_px'])
                row.update(**best,grid_centers_px=grid.tolist())
            fits.append(row)
    before_fits=[f for f in fits if f['phase']==BASE and f.get('rmse_px',100)<2]
    after_fits=[f for f in fits if f['phase']==AFTER and f.get('rmse_px',100)<2]
    relative=[]
    for old in before_fits:
        for new in after_fits:
            R=np.array(old['rotation']).T@np.array(new['rotation'])
            rv=cv.Rodrigues(R)[0].ravel()
            relative.append(np.degrees(rv))
    grid_result=dict(accepted_before_frames=len(before_fits),accepted_after_frames=len(after_fits),fits=fits)
    if relative:
        grid_result.update(local_rotation_vector_median_deg=np.median(relative,axis=0).tolist(),
            local_rotation_vector_percentiles_5_95_deg=np.percentile(relative,[5,95],axis=0).tolist())
    chosen=[groups[BASE][0],groups[BASE][-1],groups[TURN][len(groups[TURN])//2],groups[TURN][-1],groups[AFTER][0],groups[AFTER][-1]]
    panels=[]
    for f in chosen:
        panel=cv.resize(load(f),(640,360));cv.rectangle(panel,(0,0),(640,29),(0,0,0),-1)
        label={BASE:'BASE',TURN:'TURN',AFTER:'AFTER'}[f['phase']]+f" {f['camera_read_monotonic']-r['enabled_at']:.2f}s"
        cv.putText(panel,label,(8,22),cv.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
        panels.append(panel)
    sheet=np.vstack([np.hstack(panels[:3]),np.hstack(panels[3:])])
    ok,jpg=cv.imencode('.jpg',sheet,[cv.IMWRITE_JPEG_QUALITY,95])
    if not ok:raise RuntimeError('Comparison image save failed')
    (FOLDER/'before_OFF_comparison.jpg').write_bytes(jpg.tobytes())
    review=dict(source=FOLDER.name,source_sha256=hashlib.sha256(blob).hexdigest(),
        raw_status=r['status'],raw_error=r.get('error'),operator_correction=correction,
        all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
        video_frames=r['camera_recording']['frame_count'],
        before_OFF_phase_frame_counts={k:len(v) for k,v in groups.items()},
        actual_baseline_ticks=before.tolist(),actual_after_ticks=after.tolist(),
        actual_joint_change_ticks=(after-before).tolist(),
        goal_change_ticks=(np.array(after_samples[-1]['goal'])-before_samples[-1]['goal']).tolist(),
        minimum_voltage_v=min(min(s['voltage_v']) for s in r['samples']),
        peak_current_ma=max(abs(v) for s in r['samples'] for v in s['current_ma']),
        max_tracking_rad=max(max(s['tracking_rad']) for s in r['samples']),
        baseline_frame=baseline_frame,after_frame=after_frame,top_face_flow=top,static_palm_flow=palm,
        static_palm_similarity_matrix=affine.tolist(),
        translation_only_top_feature_shift_median_px=np.median(q-p-camera_translation,axis=0).tolist(),
        camera_compensated_top_feature_shift_median_px=np.median(shift,axis=0).tolist(),
        camera_compensated_top_feature_shift_magnitude_median_px=float(np.median(np.linalg.norm(shift,axis=1))),
        local_image_x_axis_change_degrees=angle,green_grid=grid_result,
        physical_target_yaw_verified=False,post_OFF_motion_used_as_rotation_evidence=False,
        physical_grasp_force_verified=False,hardware_commands_sent=0,
        limitation='Subpixel image motion and a planar sticker-grid pose are diagnostics only; glare and camera calibration can dominate a small inferred rotation.')
    (FOLDER/'actual_yaw_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in review.items() if k not in ('green_grid','baseline_frame','after_frame','operator_correction')},ensure_ascii=False))
    print(json.dumps({k:v for k,v in grid_result.items() if k!='fits'}))
    print(json.dumps(dict(grid_blob_counts=[f['blobs'] for f in fits],
        grid_rmse_px=[f.get('rmse_px') for f in fits])))


if __name__=='__main__':main()
