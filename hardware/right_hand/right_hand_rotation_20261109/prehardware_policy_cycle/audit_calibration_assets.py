"""Inventory existing camera calibration without importing any capture code."""
from pathlib import Path
import hashlib
import json
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    cameras = []
    for name in ('camera0.npz', 'camera1.npz', 'camera1_720.npz'):
        path = ROOT / name
        if not path.exists():
            continue
        with np.load(path, allow_pickle=False) as data:
            arrays = {key: np.asarray(data[key]) for key in data.files}
            K = arrays.get('K'); dist = arrays.get('dist')
            valid = K is not None and K.shape == (3,3) and np.isfinite(K).all() and K[0,0] > 0 and K[1,1] > 0
            valid = bool(valid and dist is not None and np.isfinite(dist).all())
            metadata = {}
            for key, value in arrays.items():
                if key in ('K','dist') or value.size > 8:
                    continue
                metadata[key] = value.tolist()
            cameras.append(dict(path=name, sha256=sha(path), keys=list(arrays),
                                matrix_shapes={key:list(v.shape) for key,v in arrays.items()},
                                finite_positive_intrinsic_candidate=valid, small_metadata=metadata,
                                reused_for_current_camera=False))
    registration = json.loads((ROOT / 'hand_registration_measurements.json').read_text(encoding='utf-8'))
    cross = json.loads((ROOT / 'sac_cross_session_latest_registration_result.json').read_text(encoding='utf-8'))
    sources = ('hand_registration_measurements.json', 'hand_registration_geometry.py',
               'sac_registration_from_anchor.py', 'sac_cross_session_latest_registration_result.json')
    report = dict(status='saved_calibration_asset_review_not_physical_registration',
                  camera_intrinsic_candidates=cameras,
                  hand_registration_status=registration['status'],
                  hand_orientation_verified=registration['orientation_verified'],
                  registration_uncertainty_mm=registration['measurement_uncertainty_mm'],
                  registration_assumptions=registration['assumptions_requiring_validation'],
                  cross_session_status=cross['status'], cross_session_limit=cross['limitation'],
                  older_anchor_helper_dimensions=dict(ID40_m=.0282, ID41_m=.030, cube_mm=54.5, cube_marker_mm=10.6),
                  current_handoff_dimensions=dict(ID41_black_square_approx_mm=27, cube_mm=55, cube_marker_approx_mm=10.7),
                  dimensions_are_not_interchangeable_without_review=True,
                  old_helper_was_read_not_executed=True,
                  static_camera_registration_reuse_not_ruled_out_by_occlusion=True,
                  current_camera_to_policy_sensor_transform_verified=False,
                  physical_camera_registration_verified=False,
                  no_new_metric_pose_or_mm_error_claimed_from_pixels=True,
                  source_hashes={name:sha(ROOT/name) for name in sources},
                  devices_opened=0, hardware_execution_allowed=False)
    path = OUT / 'saved_calibration_asset_review.json'
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(intrinsic_files=len(cameras), registration_status=report['hand_registration_status'],
                          physical_camera_registration_verified=False, devices_opened=0), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
