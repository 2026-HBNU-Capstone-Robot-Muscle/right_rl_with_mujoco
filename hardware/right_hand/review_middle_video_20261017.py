"""Extract timestamped evidence from user-supplied video; no device access."""
from pathlib import Path
import hashlib,json
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'middle_pose_review_20261017'/'phone_video';OUT.mkdir(parents=True,exist_ok=True)
src=Path(r'C:\Users\김준서\Documents\카카오톡 받은 파일\KakaoTalk_20261017_010259283.mp4')
cap=cv2.VideoCapture(str(src))
if not cap.isOpened():raise RuntimeError('Cannot open video')
fps=cap.get(cv2.CAP_PROP_FPS);n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
if not fps>0 or n<1:raise RuntimeError('Invalid video metadata')
report=dict(source=str(src),sha256=hashlib.sha256(src.read_bytes()).hexdigest(),fps=fps,
            frames=n,duration_s=n/fps,width=cap.get(3),height=cap.get(4),samples=[])
times=np.linspace(0,max(0,(n-1)/fps),min(20,n))
canvas=np.full((5*350,4*400,3),235,np.uint8)
for i,t in enumerate(times):
    cap.set(cv2.CAP_PROP_POS_MSEC,float(t)*1000)
    ok,frame=cap.read()
    if not ok:continue
    path=OUT/f'frame_{i:02d}_{t:.2f}s.jpg'
    cv2.imencode('.jpg',frame)[1].tofile(str(path))
    h,w=frame.shape[:2];scale=min(400/w,320/h)
    thumb=cv2.resize(frame,(round(w*scale),round(h*scale)))
    x=(i%4)*400;y=(i//4)*350
    canvas[y:y+thumb.shape[0],x:x+thumb.shape[1]]=thumb
    cv2.putText(canvas,f'{i:02d}: {t:.2f}s',(x+8,y+340),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,0),2)
    report['samples'].append(dict(index=i,time_s=float(t),path=path.name))
cap.release()
cv2.imencode('.jpg',canvas)[1].tofile(str(OUT/'contact_sheet.jpg'))
(OUT/'metadata.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
