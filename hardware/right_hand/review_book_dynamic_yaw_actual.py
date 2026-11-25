"""Review the completed revised trial before REMOVE/OFF, without device access."""
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from compare_completed_yaw_plane_20261023 import track

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'book_dynamic_yaw_trial_20261024_222132_806567'
BASE='stage_4_book_grasp_hold';TURN='stage_5_book_small_clockwise';AFTER='stage_6_book_yaw_hold'


def main():
    cv.setRNGSeed(20261024)
    blob=(FOLDER/'result.json').read_bytes();r=json.loads(blob)
    frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    cutoff=r['rotation_evidence_cutoff_camera_monotonic']
    groups={p:[f for f in frames if f['phase']==p and f['camera_read_monotonic']<=cutoff] for p in (BASE,TURN,AFTER)}
    if not r.get('rotation_command_path_completed') or not all(groups.values()):raise ValueError('Incomplete source')
    def load(f):return cv.imdecode(np.frombuffer((FOLDER/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    top_mask=np.zeros((720,1280),np.uint8)
    cv.fillPoly(top_mask,[np.array([[454,120],[687,116],[730,200],[465,210]],np.int32)],255)
    palm_mask=np.zeros_like(top_mask);palm_mask[406:528,300:443]=255
    paired=[]
    for old,new in zip(groups[BASE][-8:],groups[AFTER][-8:]):
        a=cv.cvtColor(load(old),cv.COLOR_BGR2GRAY);b=cv.cvtColor(load(new),cv.COLOR_BGR2GRAY)
        H,p,q,top=track(a,b,top_mask);Hp,pp,pq,palm=track(a,b,palm_mask)
        affine,keep=cv.estimateAffinePartial2D(pp,pq,method=cv.RANSAC,ransacReprojThreshold=.7)
        if affine is None:continue
        compensated=cv.transform(q[:,None,:],cv.invertAffineTransform(affine)).reshape(-1,2)
        shift=compensated-p
        relative,inliers=cv.findHomography(p,compensated,0)
        probe=np.float32([p.mean(0),p.mean(0)+[100,0]])
        mapped=cv.perspectiveTransform(probe[:,None,:],relative).reshape(-1,2);axis=mapped[1]-mapped[0]
        paired.append(dict(before=old['image'],after=new['image'],top_flow=top,palm_flow=palm,
            similarity_camera_compensated_shift_px=np.median(shift,axis=0).tolist(),
            image_x_axis_change_deg=float(np.degrees(np.arctan2(axis[1],axis[0])))))
    before=np.median([s['actual'] for s in r['samples'] if s['phase']==BASE][-20:],axis=0)
    after=np.median([s['actual'] for s in r['samples'] if s['phase']==AFTER][-20:],axis=0)
    chosen=[groups[BASE][0],groups[BASE][-1],groups[TURN][len(groups[TURN])//2],groups[TURN][-1],groups[AFTER][0],groups[AFTER][-1]]
    panels=[]
    for f in chosen:
        panel=cv.resize(load(f),(640,360));cv.rectangle(panel,(0,0),(640,29),(0,0,0),-1)
        label={BASE:'BEFORE',TURN:'TURN',AFTER:'AFTER'}[f['phase']]+f" {f['camera_read_monotonic']-r['enabled_at']:.2f}s"
        cv.putText(panel,label,(8,22),cv.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1);panels.append(panel)
    sheet=np.vstack([np.hstack(panels[:3]),np.hstack(panels[3:])]);ok,jpg=cv.imencode('.jpg',sheet)
    if not ok:raise RuntimeError('Review image save failed')
    (FOLDER/'before_OFF_comparison.jpg').write_bytes(jpg.tobytes())
    moving=(np.array(r['rotation_goal_ticks'])-np.array(next(s['goal'] for s in r['samples'] if s['phase']==BASE))).tolist()
    review=dict(source=FOLDER.name,source_sha256=hashlib.sha256(blob).hexdigest(),
        raw_status=r['status'],all_turn_commands_completed=r['rotation_command_path_completed'],
        all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
        valid_camera_frames=r['camera_recording']['image_signal_usable_frames'],
        minimum_voltage_v=min(min(s['voltage_v']) for s in r['samples']),
        peak_current_ma=max(abs(v) for s in r['samples'] for v in s['current_ma']),
        max_tracking_rad=max(max(s['tracking_rad']) for s in r['samples']),
        baseline_actual_ticks=before.tolist(),after_actual_ticks=after.tolist(),
        actual_joint_change_ticks=(after-before).tolist(),command_joint_change_ticks=moving,
        before_AFTER_flow_pairs=paired,
        image_shift_median_px=np.median([p['similarity_camera_compensated_shift_px'] for p in paired],axis=0).tolist(),
        image_x_axis_change_median_deg=float(np.median([p['image_x_axis_change_deg'] for p in paired])),
        image_x_axis_change_range_deg=[min(p['image_x_axis_change_deg'] for p in paired),max(p['image_x_axis_change_deg'] for p in paired)],
        user_reports_possible_middle_ring_contact=True,physical_middle_ring_contact_verified=False,
        physical_target_cube_rotation_verified=False,physical_zero_motion_proven=False,
        desired_rotation_task_succeeded=False,hardware_commands_sent=0,
        post_removal_or_OFF_motion_used_as_success_evidence=False,
        limitation='Image-plane motion can include translation and tilt. It is not a calibrated gravity-axis yaw angle. The rear finger contact point is occluded.')
    (FOLDER/'actual_yaw_review.json').write_text(json.dumps(review,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in review.items() if k!='before_AFTER_flow_pairs'}))


if __name__=='__main__':main()
