"""Select saved frames and encoder plateau evidence, without opening devices."""
import json
from pathlib import Path
import numpy as np
import cv2
ROOT=Path(__file__).resolve().parent
folder=ROOT/'book_cube_grasp_trial_20261024_172726_286561'
r=json.loads((folder/'result.json').read_bytes());frames=json.loads((folder/'camera/frames.json').read_bytes())['frames']
samples=r['samples'];selected=[]
for label,phase,which in [('open','open_before_cube','last'),('placed','place_cube','last'),('grasp start','stage_3_loaded_approach','first'),('grasp late','stage_3_loaded_approach','last')]:
    rows=[s for s in samples if s['phase']==phase];sample=rows[0 if which=='first' else -1]
    frame=min(frames,key=lambda f:abs(f['camera_read_monotonic']-r['enabled_at']-sample['t']))
    selected.append(dict(label=label,**frame))
tiles=[]
for f in selected:
    im=cv2.imdecode(np.frombuffer((folder/'camera'/f['image']).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    tile=np.zeros((395,640,3),np.uint8);tile[35:]=cv2.resize(im,(640,360))
    cv2.putText(tile,f['label'],(10,25),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1,cv2.LINE_AA);tiles.append(tile)
out=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_actual_frames.jpg'
cv2.imencode('.jpg',np.vstack((np.hstack(tiles[:2]),np.hstack(tiles[2:]))))[1].tofile(str(out))
loaded=[s for s in samples if s['phase']=='stage_3_loaded_approach'];summary=[]
for i in (0,100,200,250,280,288,300,len(loaded)-1):
    s=loaded[i];summary.append(dict(index=i+1,t=s['t'],goal=s['goal'][11],actual=s['actual'][11],current_ma=s['current_ma'][11],
        thumb_goal=s['goal'][:4],thumb_actual=s['actual'][:4]))
print(json.dumps(dict(selected_frames=selected,contact_sheet=str(out),ID12_approach=summary),ensure_ascii=False))
