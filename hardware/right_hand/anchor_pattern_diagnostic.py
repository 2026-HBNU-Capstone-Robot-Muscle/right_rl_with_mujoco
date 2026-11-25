"""Offline experiment: locate printed code features independently of outer contours."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from leap_anchor_multiview import pair_observation

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'anchor_multiview_20261014_030307_761969'

def refine(gray,points,ident):
    n=180
    dst=np.array([[0,0],[n,0],[n,n],[0,n]],np.float32)
    H=cv.getPerspectiveTransform(dst,np.asarray(points,np.float32))
    warped=cv.warpPerspective(gray,H,(n,n),flags=cv.INTER_LINEAR|cv.WARP_INVERSE_MAP)
    template=cv.aruco.generateImageMarker(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),int(ident),n)
    mask=np.zeros((n,n),np.uint8);mask[12:-12,12:-12]=255
    cc,W=cv.findTransformECC(template,warped,np.eye(3,dtype=np.float32),cv.MOTION_HOMOGRAPHY,
                           (cv.TERM_CRITERIA_COUNT|cv.TERM_CRITERIA_EPS,150,1e-6),mask,5)
    refined=cv.perspectiveTransform(dst[None],H@W)[0]
    return refined,float(cc)

def main():
    report=json.loads((FOLDER/'result.json').read_text(encoding='utf-8'))
    K=np.asarray(report['intrinsic']);dist=np.asarray(report['distortion']);rows=[]
    for frame in report['frames']:
        raw=cv.imdecode(np.fromfile(FOLDER/frame['image'],dtype=np.uint8),cv.IMREAD_COLOR)
        gray=cv.cvtColor(raw,cv.COLOR_BGR2GRAY);corners=[];scores={}
        for ident in ('40','41'):
            points,score=refine(gray,frame['markers'][ident]['corners_px'],ident)
            corners.append(points[None]);scores[ident]=score
            cv.polylines(raw,[np.rint(points).astype(np.int32)],True,(255,0,255),1)
        markers=pair_observation(corners,np.array([40,41]),K,dist)
        rows.append(dict(frame,markers=markers,pattern_correlations=scores))
        if frame['image']=='view_3_5.png':(ROOT/'anchor_pattern_view_3_5.png').write_bytes(cv.imencode('.png',raw)[1].tobytes())
        print(frame['image'],scores,round(markers['41']['reprojection_px'],3))
    report['frames']=rows;report['source_report']=str(FOLDER/'result.json')
    report['status']='offline_pattern_fit_experiment_unvalidated'
    (ROOT/'anchor_pattern_diagnostic_input.json').write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
