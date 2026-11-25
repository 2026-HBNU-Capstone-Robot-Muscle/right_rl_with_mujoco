"""Find existing usable ID5 photos before requesting another physical capture."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent

def main():
    detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[5],hand_id=41,required_cube_count=1)
    rows=[]
    for path in sorted(ROOT.glob('sequence_vision_*/placement_at_space.png')):
        frame=cv.imdecode(np.fromfile(path,np.uint8),cv.IMREAD_GRAYSCALE)
        if frame is None:continue
        corners,ids,_=detector.detectMarkers(frame)
        if ids is None or 5 not in ids:continue
        pts=np.array(corners[list(ids.ravel()).index(5)]).reshape(4,2)
        edges=np.linalg.norm(pts-np.roll(pts,1,axis=0),axis=1)
        rows.append(dict(image=str(path.relative_to(ROOT)),min_edge_px=float(edges.min()),max_edge_px=float(edges.max()),corners_px=pts.tolist()))
    rows.sort(key=lambda r:r['min_edge_px'],reverse=True)
    (ROOT/'sac_marker5_archive_inventory_result.json').write_text(json.dumps(dict(status='offline_image_inventory',motor_commands_sent=0,images=rows),indent=2)+'\n')
    print(json.dumps([{k:v for k,v in r.items() if k!='corners_px'} for r in rows[:8]],indent=2))

if __name__=='__main__':main()
