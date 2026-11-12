"""Same-frame decoded marker visibility for READ ONLY capture, not motor gates."""


def make_detector():
    import cv2 as cv
    from marker_detector import MarkerDetector
    return MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
        range(6),hand_id=40,required_cube_count=1)


def quality_from_ids(ids,edges):
    found=list(map(int,ids));cube=[i for i in found if i in range(6)]
    palm_ok=found.count(40)==1 and edges.get('40',0)>=24
    cube_ok=any(found.count(i)==1 and edges.get(str(i),0)>=24 for i in cube)
    if not palm_ok:
        instruction='PALM ID40 missing: shade its glare or adjust CAMERA slightly'
    elif not cube_ok:
        instruction='Cube marker missing/small: show the GREEN top marker ID5'
    else:instruction='PALM + CUBE visible: keep hand, cube and camera still'
    return dict(ids=found,minimum_edge_px=dict(edges),readable_pair=bool(palm_ok and cube_ok),
        instruction=instruction)


def inspect_frame(frame,detector):
    import cv2 as cv
    _,ids,_=detector.detectMarkers(cv.cvtColor(frame,cv.COLOR_BGR2GRAY))
    found=[] if ids is None else ids.ravel().tolist()
    return quality_from_ids(found,detector.stats['min_edge_px_by_id'])


def preview_with_markers(frame,detector,streak,*,prompt_for_readiness=True):
    import cv2 as cv
    quality=inspect_frame(frame,detector)
    streak=streak+1 if quality['readable_pair'] else 0
    display=frame.copy();detector.draw(display)
    if prompt_for_readiness:
        message=('READY: palm + cube visible; SPACE in TERMINAL' if streak>=3
                 else quality['instruction'])
    else:
        message='Decoded IDs: '+str(quality['ids'])+' | recording; follow TERMINAL prompts'
    cv.putText(display,message,(10,62),cv.FONT_HERSHEY_SIMPLEX,.6,
               (0,255,0) if streak>=3 else (0,180,255),2)
    return display,streak,quality
