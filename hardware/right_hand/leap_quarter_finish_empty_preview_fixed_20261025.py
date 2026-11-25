"""Same reviewed EMPTY motor path with a separately reviewed preview fix."""
import hashlib,json,sys
from pathlib import Path
import leap_quarter_finish_empty_20261025 as original
from quarter_finish_empty_camera_20261025 import QuarterFinishEmptyCamera
from quarter_finish_empty_plan_20261025 import QuarterFinishEmptyPlan,ROOT,FIXTURE
REVIEW=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/finish_preview_fix_2328/review.json'
def require_camera_mode_review():
    r=json.loads(REVIEW.read_bytes())
    expected={
        'wrapper_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'camera_module_sha256':hashlib.sha256((ROOT/'quarter_finish_empty_camera_20261025.py').read_bytes()).hexdigest(),
        'native_motion_proof_sha256':hashlib.sha256((FIXTURE/'screen.json').read_bytes()).hexdigest()}
    if (any(r.get(k)!=v for k,v in expected.items()) or r.get('program_tests_passed')!=11
            or r.get('motor_path_changed') is not False or r.get('configuration_and_on_camera_stops_unchanged') is not True):
        raise RuntimeError('Preview-only mode review mismatch; no devices opened')
    return original.require_offline_review()
def main():
    if '--execute' in sys.argv:
        try:require_camera_mode_review()
        except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
            print('OFFLINE REVIEW NOT READY:',str(exc),'. No devices opened.',flush=True);return 2
    QuarterFinishEmptyPlan.CAMERA_TYPE=QuarterFinishEmptyCamera
    print('EMPTY for the ENTIRE run. Cube outside hand AND camera. No PLACE prompt.',flush=True)
    print('One SPACE in TERMINAL. If the first camera frames are still loading, the same SPACE waits automatically.',flush=True)
    print('Known reference plus new 3.2s finish; automatic OFF. Q/Esc/Ctrl+C stops.',flush=True)
    return original.main(QuarterFinishEmptyPlan,bus_type=original.EmptyFinishBus,screener=original.run_screen)
if __name__=='__main__':raise SystemExit(main())
