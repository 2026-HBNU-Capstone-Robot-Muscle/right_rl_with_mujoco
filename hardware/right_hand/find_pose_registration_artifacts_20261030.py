from pathlib import Path
R=Path(__file__).resolve().parent
for p in sorted(R.glob('*.py')):
 if any(s in p.name for s in ('camera_fit','fit_camera','pose_views','reconstruct','extrinsic','registration','pose_overlay','render_readonly','hand_pose_fit','body_camera')):print(p.name)
