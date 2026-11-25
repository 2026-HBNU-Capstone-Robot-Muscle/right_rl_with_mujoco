"""Summarize the user's approximate dimension without changing live geometry."""
import hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/saved_marker_registration'
def main():
    rows=[]
    for size in (10.5,10.6,10.7):
        p=OUT/f'saved_after_flip_red_{size:.1f}mm_centered.json';r=json.loads(p.read_bytes());rows.append(dict(size_mm=size,median_center_palm_mm=r['median_center_palm_mm'],frame_center_peak_to_peak_mm=r['center_peak_to_peak_mm'],qualified_frames=r['qualified_frames'],evidence_sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    v=np.asarray([r['median_center_palm_mm'] for r in rows]);report=dict(status='approximate_user_red_marker_range_evaluated',user_reply='10.5?10.7? 그쯤 맞는데',input_range_mm=[10.5,10.7],nominal_sensitivity_value_mm=10.6,precise_measurement=False,photo_9_8mm_adopted_as_physical_size=False,cases=rows,between_size_median_shift_palm_mm=np.ptp(v,axis=0).tolist(),between_size_median_shift_norm_mm=float(np.linalg.norm(v[-1]-v[0])),same_size_frame_scatter_still_present=True,absolute_registration_certified=False,live_camera_detector_changed=False,motor_settings_changed=False,extra_measurement_requested=False,real_devices_opened=False)
    p=ROOT/'right_hand_rotation_20261026/red_marker_range_sensitivity.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
