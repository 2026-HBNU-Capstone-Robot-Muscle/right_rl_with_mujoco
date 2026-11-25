"""Export complete saved trials plus a clearly labelled forward-only excerpt."""
import json
from pathlib import Path
import export_gapped_inverse_video_20261029 as e
R=Path(__file__).resolve().parent
for F in sorted(R.glob('yaw_roundtrip_trial_*'))[-2:]:
 O=F/'offline_review';e.F=F;e.OUT=O
 frames=json.loads((O/'leg_frames.json').read_bytes())['frames'];exports=[]
 if not (O/'full_run_realtime.mp4').exists():exports.append(e.encode(frames,'full_run_realtime'))
 if F.name.endswith('011416_269699'):
  start=next(f['camera_read_monotonic'] for f in frames if f['leg_label']=='placed')
  end=max(f['camera_read_monotonic'] for f in frames if f['leg_label']=='forward_hold')
  if not (O/'forward_only_excerpt_realtime.mp4').exists():
   clip=e.encode([f for f in frames if start<=f['camera_read_monotonic']<=end],'forward_only_excerpt_realtime')
   clip.update(scope='forward_leg_excerpt_ends_before_programmed_inverse_not_complete_run',does_not_certify_full_cube_solve=True);exports.append(clip)
 (O/'video_exports.json').write_text(json.dumps(dict(source=F.name,exports=exports,original_images_unchanged=True,motor_writes=0),indent=2)+'\n',encoding='utf-8')
 print(json.dumps(dict(folder=F.name,exports=exports)),flush=True)
