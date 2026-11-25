"""Same-scene saved exposure comparison; no camera or motor access."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'camera_exposure_sweep_20261023_220318_878174'
OUT=ROOT/'right_hand_rotation_20261023'


def main():
    data=(SOURCE/'result.json').read_bytes();r=json.loads(data)
    assert r['status']=='camera_exposure_diagnosis_complete'
    assert r['restoration']['restored_original_readbacks']
    profiles=[];panels=[]
    for label in ('manual_0','manual_1','manual_2'):
        p=next(x for x in r['profiles'] if x['name']==label)
        metrics=[]
        for row in p['frames']:
            frame=cv.imdecode(np.frombuffer((SOURCE/row['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
            assert frame.shape==(720,1280,3)
            gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
            metrics.append(dict(image=row['image'],mean_gray=float(np.mean(gray)),
                median_gray=float(np.median(gray)),mean_hand_region_gray=float(np.mean(gray[50:690,50:850]))))
        profiles.append(dict(name=label,exposure=p['settings']['readback']['exposure'],
            cube_ID5_frames=sum(5 in row['ids'] for row in p['frames']),
            palm_ID40_frames=sum(40 in row['ids'] for row in p['frames']),
            decoded_ids=dict(Counter(str(row['ids']) for row in p['frames'])),
            samples=len(p['frames']),metrics=metrics))
        raw=cv.imdecode(np.frombuffer((SOURCE/p['frames'][0]['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
        panel=cv.resize(raw,(640,360));cv.rectangle(panel,(0,0),(640,28),(0,0,0),-1)
        cv.putText(panel,f"EXPOSURE {p['settings']['readback']['exposure']:g}",(10,21),
            cv.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
        panels.append(panel)
    ok,encoded=cv.imencode('.jpg',np.hstack(panels),[cv.IMWRITE_JPEG_QUALITY,95]);assert ok
    (OUT/'yaw_exposure_comparison.jpg').write_bytes(encoded.tobytes())
    report=dict(status='saved_scene_brightness_profile_selected',source=SOURCE.name,
        source_sha256=hashlib.sha256(data).hexdigest(),selected_exposure=-5.,
        basis='Brighter visible black hand detail, ID5 decoded in all six original and six manual -5 frames.',
        profiles=profiles,palm_glare_still_present=True,
        simultaneous_palm_cube_pose_registration_verified=False,
        selected_profile_is_only_for_GraspedYawPlan=True,
        other_runtime_camera_default_exposure=-7.,
        autofocus_or_gain_changed=False,motion_targets_changed=False,motor_limits_changed=False,
        rotation_verified=False,read_failure_stop_policy_changed=False,
        previous_operator_statement='세번다 없었어 다시했어',
        previous_capture_conditions_not_used_for_profile_selection=True,
        further_camera_sweep_required=False,assistant_opened_real_devices=False)
    (OUT/'yaw_exposure_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='profiles'},ensure_ascii=False,indent=2))
    for p in profiles:print(p['exposure'], 'cube',p['cube_ID5_frames'],'palm',p['palm_ID40_frames'],
        'mean gray',round(float(np.mean([m['mean_gray'] for m in p['metrics']])),1))


if __name__=='__main__':main()
