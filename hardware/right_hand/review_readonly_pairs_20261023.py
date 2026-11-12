"""Inspect saved pairs and try same-frame marker decoding, no devices."""
import hashlib
import json
from pathlib import Path
import time

import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_pose_readonly_20261023_165647_488518'


def main():
    blob=(FOLDER/'result.json').read_bytes();report=json.loads(blob)
    pairs=report['pairs'];assert len(pairs)==12
    summary=dict(status=report['status'],source_sha256=hashlib.sha256(blob).hexdigest(),
        pair_count=len(pairs),stable_endpoints=sum(r['stationary_endpoints'] for r in pairs),
        max_pair_drift_ticks=max(r['maximum_drift_ticks'] for r in pairs),
        min_voltage_v=min(min(r[k]['snapshot']['voltage_v']) for r in pairs for k in ('before','after')),
        all_torque_off=all(not any(r[k]['snapshot']['torque']) for r in pairs for k in ('before','after')),
        camera_restored=report['camera_restoration_verified'],
        original_marker_ids=[r['markers']['ids'] for r in pairs],
        position_min_ticks=np.min([r[k]['snapshot']['position_ticks'] for r in pairs for k in ('before','after')],axis=0).tolist(),
        position_max_ticks=np.max([r[k]['snapshot']['position_ticks'] for r in pairs for k in ('before','after')],axis=0).tolist(),
        frames_bracket_ms=[round(r['frame_bracket_ms'],1) for r in pairs],
        real_devices_opened=False,servo_writes_sent=0)
    print(json.dumps(summary,indent=2),flush=True)
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    recovered=[];attempts=0;began=time.monotonic()
    for row in pairs:
        image=cv.imdecode(np.frombuffer((FOLDER/row['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
        # ROI is the visible palm region only; no recoloring, marker-bit repair,
        # source-frame substitution, remembered corners or forced ID supplied.
        x0,y0,x1,y1=350,350,600,570
        crop=image[y0:y1,x0:x1]
        variants=[('gray',cv.cvtColor(crop,cv.COLOR_BGR2GRAY))]
        variants.extend((f'channel_{n}',crop[:,:,n]) for n in range(3))
        gray=variants[0][1]
        for clip in (1.,3.,6.):
            variants.append((f'clahe_{clip}',cv.createCLAHE(clipLimit=clip,tileGridSize=(4,4)).apply(gray)))
        found=[]
        for method,im in variants:
            for border in (.35,.6):
                params=cv.aruco.DetectorParameters()
                params.adaptiveThreshWinSizeMin=3;params.adaptiveThreshWinSizeMax=61
                params.adaptiveThreshWinSizeStep=4
                params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_CONTOUR
                params.maxErroneousBitsInBorderRate=border
                # Keep dictionary correction at its default. Do not force ID40.
                cs,ids,_=cv.aruco.ArucoDetector(dictionary,params).detectMarkers(im)
                attempts+=1
                if ids is None:continue
                for corners,ident in zip(cs,ids.ravel()):
                    full=np.asarray(corners).reshape(4,2)+[x0,y0]
                    found.append(dict(ident=int(ident),method=method,border_error_rate=border,corners=full.tolist()))
        recovered.append(dict(image=row['image'],detections=found))
        print(row['image'],'candidate IDs',sorted({r['ident'] for r in found}),flush=True)
    summary.update(same_frame_roi_decoding=recovered,attempts=attempts,
        elapsed_seconds=time.monotonic()-began,
        palm40_recovered_frames=sum(any(x['ident']==40 for x in r['detections']) for r in recovered),
        geometry_calibration_verified=False,hardware_execution_allowed=False)
    (FOLDER/'paired_capture_review.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')


if __name__=='__main__':main()
