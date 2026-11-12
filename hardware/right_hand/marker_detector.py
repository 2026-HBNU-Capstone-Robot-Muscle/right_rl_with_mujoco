"""Same-frame ArUco detection with bounded contrast/dense-threshold fallbacks.

Only decoded board IDs and the hand ID are returned. No inferred, remembered
or projected markers are counted toward the two-marker policy gate.
"""
import time
import cv2 as cv
import numpy as np


class MarkerDetector:
    def __init__(self,dictionary,cube_ids,hand_id=40,required_cube_count=2):
        self.cube_ids=set(int(i) for i in np.asarray(cube_ids).ravel())
        self.hand_id=hand_id
        self.required_cube_count=required_cube_count
        params=cv.aruco.DetectorParameters()
        params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_CONTOUR
        params.adaptiveThreshWinSizeMax=53
        self.detector=cv.aruco.ArucoDetector(dictionary,params)
        dense=cv.aruco.DetectorParameters()
        dense.cornerRefinementMethod=cv.aruco.CORNER_REFINE_CONTOUR
        dense.adaptiveThreshWinSizeMax=53
        dense.adaptiveThreshWinSizeStep=4
        dense.adaptiveThreshConstant=9
        self.dense_detector=cv.aruco.ArucoDetector(dictionary,dense)
        self.raw_dense_detector=cv.aruco.ArucoDetector(dictionary,dense)
        strong=cv.aruco.DetectorParameters()
        strong.cornerRefinementMethod=cv.aruco.CORNER_REFINE_CONTOUR
        strong.adaptiveThreshWinSizeMax=53
        strong.adaptiveThreshWinSizeStep=4
        strong.adaptiveThreshConstant=15
        self.strong_detector=cv.aruco.ArucoDetector(dictionary,strong)
        self.clahe=cv.createCLAHE(clipLimit=2.,tileGridSize=(8,8))
        self.last_corners=[]; self.last_ids=None; self.stats={}

    def filtered(self,result):
        corners,ids,rejected=result
        if ids is None: return [],None,rejected
        ids=np.asarray(ids).ravel()
        unique,counts=np.unique(ids,return_counts=True)
        allowed={int(i) for i,n in zip(unique,counts) if n==1 and
                 (int(i) in self.cube_ids or int(i)==self.hand_id)}
        keep=[k for k,i in enumerate(ids) if int(i) in allowed]
        return [corners[k] for k in keep],ids[keep].astype(np.int32).reshape(-1,1) if keep else None,rejected

    def score(self,result):
        ids=set() if result[1] is None else set(int(i) for i in result[1].ravel())
        cube=len(ids & self.cube_ids); hand=self.hand_id in ids
        return (hand and cube>=self.required_cube_count,hand,cube)

    def detectMarkers(self,gray):
        began=time.monotonic()
        result=self.filtered(self.detector.detectMarkers(gray))
        method='contour'
        # Try the original grayscale before CLAHE: contrast enhancement can lose
        # a small foreshortened marker. At most five independent same-frame passes.
        # A complete primary detection still keeps its original corners.
        if not self.score(result)[0]:
            fallback=self.filtered(self.raw_dense_detector.detectMarkers(gray))
            if self.score(fallback)>self.score(result): result,method=fallback,'raw_dense_threshold'
        if not self.score(result)[0]:
            contrast=self.clahe.apply(gray)
            fallback=self.filtered(self.detector.detectMarkers(contrast))
            if self.score(fallback)>self.score(result): result,method=fallback,'contrast'
        if not self.score(result)[0]:
            fallback=self.filtered(self.dense_detector.detectMarkers(contrast))
            if self.score(fallback)>self.score(result): result,method=fallback,'dense_threshold'
        if not self.score(result)[0]:
            fallback=self.filtered(self.strong_detector.detectMarkers(contrast))
            if self.score(fallback)>self.score(result): result,method=fallback,'strong_threshold'
        self.last_corners,self.last_ids,_=result
        sizes={}
        if self.last_ids is not None:
            for corners,ident in zip(self.last_corners,self.last_ids.ravel()):
                points=np.asarray(corners).reshape(4,2)
                sizes[str(int(ident))]=round(float(np.min(np.linalg.norm(points-np.roll(points,1,axis=0),axis=1))),1)
        self.stats=dict(method=method,corner_refinement='contour',ids=[] if self.last_ids is None else self.last_ids.ravel().tolist(),
                        min_edge_px_by_id=sizes,detect_ms=(time.monotonic()-began)*1000)
        return result

    def draw(self,frame):
        if self.last_ids is not None:
            cv.aruco.drawDetectedMarkers(frame,self.last_corners,self.last_ids)
        sizes=' '.join(f'ID{k}:{v:g}px' for k,v in self.stats.get('min_edge_px_by_id',{}).items())
        cv.putText(frame,'Detected '+(sizes or 'none'),(10,frame.shape[0]-15),
                   cv.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),2)
