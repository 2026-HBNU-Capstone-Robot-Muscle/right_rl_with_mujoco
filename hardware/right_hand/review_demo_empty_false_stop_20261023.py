"""Replay saved EMPTY camera frames to inspect the pre-motion false alarm."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from cube_marker_readiness import make_detector,inspect_frame

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'slow_grasp_demo_20261023_180831_999579'


def main():
    result=json.loads((FOLDER/'result.json').read_bytes())
    print('LIVE_EMPTY_SCREEN',json.dumps(result.get('empty_start_marker_screen'),indent=2))
    detector=make_detector();detections=[];overlays=[]
    for row in json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']:
        path=FOLDER/'camera'/row['image']
        frame=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)
        quality=inspect_frame(frame,detector)
        cube=[i for i in quality['ids'] if i in range(6)]
        if not cube:continue
        corners={str(int(i)):np.asarray(c).reshape(4,2).tolist()
                 for i,c in zip(detector.last_ids.ravel(),detector.last_corners)}
        record=dict(row,quality=quality,corners=corners,detector_method=detector.stats['method'])
        detections.append(record);display=frame.copy();detector.draw(display)
        cv.putText(display,row['image'],(20,30),cv.FONT_HERSHEY_SIMPLEX,.8,(0,255,255),2)
        name='replayed_cube_'+row['image']
        ok,data=cv.imencode('.jpg',display)
        if ok:(FOLDER/name).write_bytes(data.tobytes())
        print(json.dumps(record,ensure_ascii=False),flush=True)
    review=dict(source=FOLDER.name,live_empty_screen=result.get('empty_start_marker_screen'),
        replay_detections=detections,frames_reviewed=len(json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']),
        motors_enabled=False,torque_off_verified=result['all_torque_off_verified'])
    (FOLDER/'empty_start_false_alarm_review.json').write_text(json.dumps(review,indent=2),encoding='utf-8')
    print('CUBE_DETECTION_FRAMES',len(detections),flush=True)


if __name__=='__main__':main()
