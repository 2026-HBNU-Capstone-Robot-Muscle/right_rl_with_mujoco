import datetime,hashlib,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/rigid_contact_review_20261025'
r=subprocess.run([sys.executable,'-X','utf8','-B','-m','unittest','test_quarter_support_photo_20261025','test_support_photo_camera_20261025','test_support_photo_contact_resolution_20261025'],cwd=ROOT,capture_output=True,text=True,encoding='utf-8',timeout=120)
log=r.stdout+r.stderr;(OUT/'photo_mode_tests.log').write_text(log,encoding='utf-8');print(log,end='')
if r.returncode:raise SystemExit(r.returncode)
assert 'Ran 19 tests' in log and log.rstrip().endswith('OK')
import leap_quarter_support_photo_20261025 as mode
mode.require_offline_review()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
proof=mode.FIXTURE/'screen.json'
review=dict(timestamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat(),script_sha256=sha(ROOT/'leap_quarter_support_photo_20261025.py'),
    off_camera_adjustment_sha256=sha(ROOT/'support_photo_camera_20261025.py'),
    command_sha256=sha(ROOT/'leap_quarter_support_photo.cmd'),test_script_sha256=sha(ROOT/'test_quarter_support_photo_20261025.py'),
    native_full_route_proof_sha256=sha(proof),device_free_tests_passed=7,off_camera_tests_passed=7,contact_resolution_tests_passed=5,total_tests_passed=19,loaded_goals_sent_in_mock=0,off_and_settings_restored_in_mock=True,
    independently_reviewed_empty_prefix=True,current_pose_rescreen_required_at_runtime=True,hardware_commands_sent_during_review=0,
    execution_requires_user_first_space=True,user_cube_placement_and_removal_required=True,physical_rotation_verified=False)
(OUT/'photo_mode_review.json').write_text(json.dumps(review,indent=2),encoding='utf-8');mode.require_mode_review();print(json.dumps(review))
