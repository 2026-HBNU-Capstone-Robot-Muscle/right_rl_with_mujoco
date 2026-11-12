"""Read saved side photos/encoder pairs without accessing devices."""
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/middle_ring_side_review'
FOLDERS=('pose_pair_readonly_20261024_230512_916136','pose_pair_readonly_20261024_230555_418562')


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    rows=[];images=[]
    for foldername in FOLDERS:
        folder=ROOT/foldername;blob=(folder/'result.json').read_bytes();r=json.loads(blob)
        assert r['status']=='photo_and_readings_saved'
        assert r['servo_write_commands_sent']==r['torque_changes_requested']==0
        assert r['reference_label']=='middle_ring_existing_side'
        assert r['pair_check']['stationary_endpoints']
        positions=[]
        for snapshot in r['snapshots']:
            motors=snapshot['motors'];assert len(motors)==16
            assert all(m['torque_enabled']==m['torque_enabled_end']==m['hardware_error']==m['bus_watchdog_raw']==0 for m in motors)
            positions.append([next(m['position_ticks_signed'] for m in motors if m['id']==i) for i in range(1,17)])
        raw=np.mean(positions,axis=0)
        photo=(folder/'paired_photo.png').read_bytes()
        image=cv.imdecode(np.frombuffer(photo,np.uint8),cv.IMREAD_COLOR)
        rows.append(dict(folder=foldername,result_sha256=hashlib.sha256(blob).hexdigest(),
            image_sha256=hashlib.sha256(photo).hexdigest(),raw_ticks=raw.tolist(),
            pair_maximum_drift_ticks=r['pair_check']['maximum_drift_ticks'],
            minimum_voltage_v=min(m['voltage_v'] for s in r['snapshots'] for m in s['motors']),
            maximum_temperature_c=max(m['temperature_c'] for s in r['snapshots'] for m in s['motors']),
            burst_frame_count=len(r.get('photo_burst',{}).get('samples',[])),
            geometric_calibration_verified=False))
        images.append(image)
    delta=np.array(rows[1]['raw_ticks'])-np.array(rows[0]['raw_ticks'])
    summary=dict(captures=rows,between_capture_delta_ticks=delta.tolist(),
        maximum_between_capture_delta_ticks=float(np.max(abs(delta))),
        same_encoder_pose_within_3_ticks=bool(np.max(abs(delta))<=3),
        physical_contact_force_measured=False,current_motor_interface_opened=False,hardware_commands_sent=0,
        user_report_same_posture_twice=True)
    (OUT/'paired_capture_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    panel=np.vstack([cv.resize(im,(960,540)) for im in images])
    for i,row in enumerate(rows):
        cv.rectangle(panel,(0,i*540),(960,i*540+38),(0,0,0),-1)
        cv.putText(panel,row['folder']+' | torque OFF | saved photo + encoders',(8,i*540+24),cv.FONT_HERSHEY_SIMPLEX,.50,(255,255,255),1)
    (OUT/'two_saved_side_views.jpg').write_bytes(cv.imencode('.jpg',panel,[cv.IMWRITE_JPEG_QUALITY,92])[1].tobytes())
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
