"""Find a directly decoded palm reference in existing frames; no devices."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_grasp_trial_20261023_044607_477240'

def main():
    frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),hand_id=40,required_cube_count=1)
    rows=[]
    selected=[f for i,f in enumerate(frames) if i%6==0 and i<1200]
    for index,f in enumerate(selected):
        im=cv.imdecode(np.frombuffer((FOLDER/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
        gray=cv.cvtColor(im,cv.COLOR_BGR2GRAY)
        cs,ids,_=detector.detectMarkers(gray)
        ids=[] if ids is None else list(map(int,ids.ravel()))
        if 40 in ids:
            rows.append(dict(**f,ids=ids,corners40=np.asarray(cs[ids.index(40)]).reshape(4,2).tolist()))
        if index%40==0:print('SCAN',index,'/',len(selected),'direct palm references',len(rows),flush=True)
    output=dict(status='saved_frame_direct_palm_reference_scan',source=FOLDER.name,scanned=len(selected),rows=rows)
    (FOLDER/'direct_palm_reference_scan.json').write_text(json.dumps(output,indent=2),encoding='utf-8')
    print('FOUND',len(rows),'FIRST_LAST',[(r['image'],r['phase']) for r in rows[:3]+rows[-3:]],flush=True)

if __name__=='__main__':main()
