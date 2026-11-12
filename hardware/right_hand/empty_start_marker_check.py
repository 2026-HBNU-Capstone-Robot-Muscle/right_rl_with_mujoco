"""Conservative pre-motion cube evidence. No pose inference or motor access."""
import numpy as np

MINIMUM_EDGE_PX=20.
CONFIRM_FRAMES=3
CONFIRM_SECONDS=.2
MAX_FRAME_GAP_SECONDS=.35


def make_empty_detector():
    import cv2 as cv
    params=cv.aruco.DetectorParameters()
    params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_CONTOUR
    params.adaptiveThreshWinSizeMax=53
    # An EMPTY-hand stop must not use the permissive multi-pass pose tracker.
    # Require an exact codeword and a nearly intact black border instead.
    params.errorCorrectionRate=0.
    params.maxErroneousBitsInBorderRate=.05
    return cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),params)


def inspect_empty_frame(frame,detector):
    import cv2 as cv
    corners,ids,_=detector.detectMarkers(cv.cvtColor(frame,cv.COLOR_BGR2GRAY))
    found=[]
    if ids is None:return found
    ids=ids.ravel()
    for polygon,ident in zip(corners,ids):
        ident=int(ident)
        if ident not in range(6) or np.count_nonzero(ids==ident)!=1:continue
        points=np.asarray(polygon,dtype=float).reshape(4,2)
        if not np.isfinite(points).all():continue
        edge=float(np.min(np.linalg.norm(points-np.roll(points,1,axis=0),axis=1)))
        if edge<MINIMUM_EDGE_PX:continue
        found.append(dict(id=ident,minimum_edge_px=edge,corners=points.tolist(),
                          center_px=points.mean(0).tolist()))
    return found


class EmptyStartEvidence:
    def __init__(self):
        self.last_stamp=None;self.tracks={};self.empty_streak=0

    def observe(self,stamp,candidates):
        if self.last_stamp is not None and stamp<=self.last_stamp:return None
        if self.last_stamp is not None and stamp-self.last_stamp>MAX_FRAME_GAP_SECONDS:
            self.tracks={};self.empty_streak=0
        self.last_stamp=stamp;next_tracks={};confirmed=[]
        for candidate in candidates:
            ident=candidate['id'];old=self.tracks.get(ident)
            same=(old is not None and np.linalg.norm(np.array(old['center_px'])-candidate['center_px'])
                  <=max(6.,candidate['minimum_edge_px']*.5))
            track=dict(candidate,first_stamp=old['first_stamp'] if same else stamp,
                       consecutive_frames=old['consecutive_frames']+1 if same else 1)
            next_tracks[ident]=track
            if track['consecutive_frames']>=CONFIRM_FRAMES and stamp-track['first_stamp']>=CONFIRM_SECONDS-1e-9:
                confirmed.append(ident)
        self.tracks=next_tracks
        self.empty_streak=0 if candidates else self.empty_streak+1
        return dict(camera_read_monotonic=stamp,decoded_cube_ids=sorted(next_tracks),
            confirmed_cube_ids=sorted(confirmed),candidates=list(next_tracks.values()),
            consecutive_frames_without_decoded_cube=self.empty_streak,
            confirmation_frames=CONFIRM_FRAMES,confirmation_seconds=CONFIRM_SECONDS,
            minimum_marker_edge_px=MINIMUM_EDGE_PX,absence_proves_empty_hand=False)
