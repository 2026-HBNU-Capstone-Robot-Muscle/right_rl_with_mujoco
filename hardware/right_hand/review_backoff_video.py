"""Read-only video/telemetry inspection; no hardware access."""
from pathlib import Path
import json
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'backoff_video_review_20261016'
OUT.mkdir(exist_ok=True)
video = Path(r'C:\Users\김준서\Documents\카카오톡 받은 파일\KakaoTalk_20261016_144944731.mp4')
cap = cv2.VideoCapture(str(video))
fps = cap.get(cv2.CAP_PROP_FPS)
n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
duration = n / fps
print(json.dumps(dict(fps=fps, frames=n, duration=duration, width=cap.get(3), height=cap.get(4))))
times = np.arange(0, duration, 1.0)
for page in range((len(times)+19)//20):
    canvas = np.full((5*290, 4*384, 3), 235, np.uint8)
    for j,t in enumerate(times[page*20:(page+1)*20]):
        cap.set(cv2.CAP_PROP_POS_MSEC, float(t)*1000)
        ok, frame = cap.read()
        if not ok: continue
        h,w=frame.shape[:2]; scale=min(384/w,260/h)
        thumb=cv2.resize(frame,(int(w*scale),int(h*scale)))
        y=(j//4)*290; x=(j%4)*384
        canvas[y:y+thumb.shape[0],x:x+thumb.shape[1]]=thumb
        cv2.putText(canvas,f'{t:.1f}s',(x+8,y+281),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,0),2)
    cv2.imencode('.jpg',canvas)[1].tofile(str(OUT/f'sheet_{page}.jpg'))
for t in (10,12,13,14,15,16,17,18,20):
    cap.set(cv2.CAP_PROP_POS_MSEC,t*1000)
    ok,frame=cap.read()
    if ok: cv2.imencode('.jpg',frame)[1].tofile(str(OUT/f'frame_{t}.jpg'))
cap.release()
for p in sorted(ROOT.glob('leap_id12_backoff_empty_20261016_*.json')):
    d=json.loads(p.read_text(encoding='utf-8'))
    result={k:d.get(k) for k in ('timestamp','status','stop_reason','all_torque_off_verified','settings_restored')}
    for key in ('approach_hold','return_hold'):
        a=d.get(key)
        if a: result[key]={k:a.get(k) for k in ('duration_s','actual_end_median','current_end_median_ma','tracking_end_median_rad')}
    print(p.name,json.dumps(result))
