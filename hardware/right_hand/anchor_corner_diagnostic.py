"""Read-only comparison of ArUco refinements on archived multiview images."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'anchor_multiview_20261014_030307_761969'

def main():
    report=json.loads((FOLDER/'result.json').read_text(encoding='utf-8'))
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    detectors={}
    for name,method in [('none',cv.aruco.CORNER_REFINE_NONE),('subpix',cv.aruco.CORNER_REFINE_SUBPIX),('contour',cv.aruco.CORNER_REFINE_CONTOUR),('apriltag',cv.aruco.CORNER_REFINE_APRILTAG)]:
        p=cv.aruco.DetectorParameters();p.cornerRefinementMethod=method;p.adaptiveThreshWinSizeMax=53
        detectors[name]=cv.aruco.ArucoDetector(dictionary,p)
    rows=[]
    for frame in report['frames']:
        raw=cv.imdecode(np.fromfile(FOLDER/frame['image'],dtype=np.uint8),cv.IMREAD_COLOR)
        gray=cv.cvtColor(raw,cv.COLOR_BGR2GRAY)
        row={'image':frame['image'],'methods':{}}
        for name,det in detectors.items():
            corners,ids,_=det.detectMarkers(gray)
            row['methods'][name]={str(int(i)):np.asarray(c).reshape(4,2).tolist() for c,i in zip(corners,[] if ids is None else ids.ravel())}
        for ident,marker in frame['markers'].items():
            cv.polylines(raw,[np.rint(marker['corners_px']).astype(np.int32)],True,(0,0,255),1)
        for ident,pts in row['methods']['subpix'].items():
            if ident in ('40','41'):cv.polylines(raw,[np.rint(pts).astype(np.int32)],True,(0,255,0),1)
        if frame['image'] in ('view_3_4.png','view_3_5.png'):
            (ROOT/('anchor_corners_'+frame['image'])).write_bytes(cv.imencode('.png',raw)[1].tobytes())
        rows.append(row)
        print(frame['image'],{name:round(float(np.max(np.linalg.norm(np.asarray(markers['41'])-frame['markers']['41']['corners_px'],axis=1))),2) if '41' in markers else None for name,markers in row['methods'].items()})
    (ROOT/'anchor_corner_diagnostic_result.json').write_text(json.dumps(rows,indent=2)+'\n')

if __name__=='__main__':main()
