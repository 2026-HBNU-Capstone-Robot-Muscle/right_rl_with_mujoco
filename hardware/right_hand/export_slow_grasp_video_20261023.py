"""Export stored frames at their recorded pace, without any device access."""
import hashlib
import json
import os
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'slow_grasp_demo_20261023_182105_573825'


def export(rows,name):
    stamps=np.array([r['camera_read_monotonic'] for r in rows]);fps=15.
    timeline=np.arange(stamps[0],stamps[-1]+1/fps/2,1/fps)
    writer=cv.VideoWriter(name,cv.VideoWriter_fourcc(*'mp4v'),fps,(1280,720))
    if not writer.isOpened():raise RuntimeError('Cannot open stored-video encoder')
    last_index=None;frame=None;max_age=0.
    try:
        for stamp in timeline:
            index=max(0,int(np.searchsorted(stamps,stamp,side='right')-1))
            max_age=max(max_age,stamp-stamps[index])
            if index!=last_index:
                path=FOLDER/'camera'/rows[index]['image']
                frame=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)
                if frame is None or frame.shape!=(720,1280,3):raise RuntimeError('Invalid stored camera frame')
                last_index=index
            writer.write(frame)
    finally:writer.release()
    reader=cv.VideoCapture(name)
    try:
        count=int(reader.get(cv.CAP_PROP_FRAME_COUNT));ok,decoded=reader.read()
        assert reader.isOpened() and ok and count==len(timeline) and decoded.shape==(720,1280,3)
    finally:reader.release()
    return dict(file=name,frames=count,fps=fps,source_span_s=float(stamps[-1]-stamps[0]),
        video_duration_s=count/fps,maximum_repeated_frame_age_s=float(max_age),
        bytes=(FOLDER/name).stat().st_size,sha256=hashlib.sha256((FOLDER/name).read_bytes()).hexdigest())


def main():
    r=json.loads((FOLDER/'result.json').read_bytes());assert r['status']=='slow_grasp_demo_complete'
    frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    start=min(f['camera_read_monotonic'] for f in frames if f['phase']=='stage_0_empty_recover')
    end=max(f['camera_read_monotonic'] for f in frames if f['phase']=='grasp_observation')
    loaded_start=min(f['camera_read_monotonic'] for f in frames if f['phase']=='stage_3_loaded_approach')
    os.chdir(FOLDER)
    outputs=[export([f for f in frames if start<=f['camera_read_monotonic']<=end],'slow_grasp_full.mp4'),
             export([f for f in frames if loaded_start-.5<=f['camera_read_monotonic']<=end],'slow_grasp_close.mp4')]
    record=dict(status='recorded_slow_grasp_videos_exported',source=FOLDER.name,
        timing='Original driver-read pace, previous raw image repeated only until the next recorded frame.',
        source_images_modified=False,live_sac=False,cube_rotation_claimed=False,real_devices_opened=False,outputs=outputs)
    (FOLDER/'video_export_review.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
