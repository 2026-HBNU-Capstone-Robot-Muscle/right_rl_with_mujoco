"""Audit saved operator runs and replay marker detection; no devices."""
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from cube_marker_readiness import make_detector,inspect_frame

ROOT=Path(__file__).resolve().parent


def main():
    detector=make_detector();reviews=[]
    for name in ('slow_grasp_demo_20261023_175203_740582','slow_grasp_demo_20261023_175232_779550'):
        folder=ROOT/name;blob=(folder/'result.json').read_bytes();r=json.loads(blob)
        samples=r.get('samples',[]);phases={}
        for s in samples:phases[s['phase']]=phases.get(s['phase'],0)+1
        evidence=[]
        for relative in ('camera/frame_00000.jpg','ring_released.jpg','open_before_cube.jpg'):
            path=folder/relative
            if not path.is_file():continue
            frame=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)
            q=inspect_frame(frame,detector)
            evidence.append(dict(image=relative,decoded_ids=q['ids'],cube_detected=any(i in range(6) for i in q['ids'])))
        review=dict(folder=name,source_sha256=hashlib.sha256(blob).hexdigest(),
            status=r['status'],error=r.get('error'),failed_phase=r.get('failed_phase'),
            all_torque_off_verified=r.get('all_torque_off_verified'),settings_restored=r.get('settings_restored'),
            phases=phases,loaded_grasp_started='stage_3_loaded_approach' in phases,
            fixed_grasp_observation_started='stage_4_fixed_grasp_observation' in phases,
            photos=evidence,
            operator_report='Cube was already on the hand before EMPTY preparation.',
            empty_hand_precondition_satisfied=False,physical_grasp_or_rotation_verified=False)
        if samples:
            review['measured']=dict(minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
                peak_absolute_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
                maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),
                hardware_error_seen=any(any(s['error']) for s in samples))
        reviews.append(review)
    result=dict(runs=reviews,assistant_opened_devices=False)
    (ROOT/'right_hand_rotation_20261023/loaded_demo_start_review.json').write_text(
        json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
