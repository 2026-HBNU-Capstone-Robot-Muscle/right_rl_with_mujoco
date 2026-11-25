"""Export raw recorded rehearsal frames; no live camera, overlays or devices."""
import json
import os
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parent
folder=ROOT/'supported_yaw_empty_20261023_030552_675209'
r=json.loads((folder/'camera/frames.json').read_bytes())
frames=[f for f in r['frames'] if f['phase'] in ('stage_4_supported_yaw_EMPTY','stage_2_fixed_hold')]
def read_image(path):return cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
first=read_image(folder/'camera'/frames[0]['image'])
assert first is not None
period=(frames[-1]['camera_read_monotonic']-frames[0]['camera_read_monotonic'])/max(1,len(frames)-1)
os.chdir(folder)
writer=cv2.VideoWriter('recorded_empty_suffix.mp4',cv2.VideoWriter_fourcc(*'mp4v'),1/period,(first.shape[1],first.shape[0]))
assert writer.isOpened()
for frame in frames:
    image=read_image(folder/'camera'/frame['image'])
    assert image is not None and image.shape==first.shape
    writer.write(image)
writer.release()
print('FRAMES',len(frames),'SECONDS',round(len(frames)*period,3))
for index in (0,len(frames)//3,2*len(frames)//3,len(frames)-1):print(frames[index]['image'])
