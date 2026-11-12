"""Export recorded motion frames to a video file; no device I/O."""
import json
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_small_rotation_trial_20261022_151241_962114'
rows=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
rows=[r for r in rows if r['phase'] in ('stage_4_small_rotation','stage_2_fixed_hold')]
span=rows[-1]['camera_read_monotonic']-rows[0]['camera_read_monotonic']
fps=(len(rows)-1)/span
path=FOLDER/'recorded_small_rotation.mp4'
writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1280,720))
if not writer.isOpened():raise RuntimeError('Could not open file encoder')
try:
    for row in rows:
        frame=cv2.imdecode(np.frombuffer((FOLDER/'camera'/row['image']).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        if frame is None or frame.shape!=(720,1280,3):raise RuntimeError('Invalid stored image')
        writer.write(frame)
finally:writer.release()
print(json.dumps(dict(file=str(path),frames=len(rows),source_camera_span_s=span,video_fps=fps,
                     hardware_commands_sent=0),indent=2))
