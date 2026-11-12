"""Timestamp-faithful export of the saved supported Cube experiment."""
import json
from pathlib import Path
import export_gapped_inverse_video_20261029 as exporter
ROOT=Path(__file__).resolve().parent;F=ROOT/'supported_inverse_trial_20261029_231345_341471';O=F/'offline_review'
exporter.F=F;exporter.OUT=O
frames=json.loads((F/'camera/frames.json').read_bytes())['frames']
exports=[exporter.encode(frames,'full_run_realtime')]
start=next(x['camera_read_monotonic'] for x in frames if x['phase']=='loaded_countdown')
end=max(x['camera_read_monotonic'] for x in frames if x['phase']=='stage_10_remove_hold')
exports.append(exporter.encode([x for x in frames if start<=x['camera_read_monotonic']<=end],'grasp_turn_open_realtime'))
(O/'video_exports.json').write_text(json.dumps(dict(source=F.name,exports=exports,motor_writes=0),indent=2)+'\n',encoding='utf-8')
print(json.dumps(exports),flush=True)
