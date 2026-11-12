"""Run the explicitly selected offline checks and save their source bindings."""
import os
os.environ['JAX_PLATFORMS']='cpu'
os.environ.setdefault('MUJOCO_GL','egl')
import unittest,json,hashlib,io,time
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip'
names=['test_upper_layer_separation_certificate_20261105','test_canonical_grip_certificate_latch_20261105','test_canonical_grip_task_review_20261105','test_canonical_grip_render_display_20261105','test_offline_two_hand_coordinator_20261105','test_offline_fractional_goal_accumulator_20261105','test_cube_task_frame_20261105','test_native_pyramidal_point_forces_20261106']
names.append('test_checkpoint_quarter_review_20261106')
names.append('test_offline_goal_tracking_bounds_20261106')
names.append('test_temporal_two_hand_coordinator_20261106')
suite=unittest.defaultTestLoader.loadTestsFromNames(names);stream=io.StringIO();began=time.perf_counter();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
report=dict(status='offline_software_checks',tests_run=result.testsRun,successful=result.wasSuccessful(),failures=[dict(test=str(t),detail=msg) for t,msg in result.failures],errors=[dict(test=str(t),detail=msg) for t,msg in result.errors],test_source_sha256={n+'.py':hashlib.sha256((R/(n+'.py')).read_bytes()).hexdigest() for n in names},elapsed_seconds=time.perf_counter()-began,devices_opened=False,physical_controller_or_robot_not_tested=True)
(O/'software_checks.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');(O/'software_checks.log').write_text(stream.getvalue(),encoding='utf-8');print(json.dumps(report),flush=True)
if not result.wasSuccessful():raise SystemExit(1)
