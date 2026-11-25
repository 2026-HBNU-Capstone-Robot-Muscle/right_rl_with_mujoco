"""Recheck saved exposure frames and relative marker poses; never open devices."""
import hashlib
import itertools
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector
from review_prepared_cube_20261022 import marker_solutions

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'camera_exposure_sweep_20261023_161815_427701'

def main():
    blob=(FOLDER/'result.json').read_bytes();source=json.loads(blob)
    assert source['status']=='camera_exposure_diagnosis_complete'
    assert source['restoration']['restored_original_readbacks']
    detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),hand_id=40,required_cube_count=1)
    with np.load(ROOT/'camera1_720.npz') as d:K,dist=d['K'],d['dist']
    summaries=[];pose_rows=[]
    for profile in source['profiles']:
        if not profile.get('frames'):continue
        counts={'40':0,'5':0,'paired':0};details=[]
        for f in profile['frames']:
            image=cv.imdecode(np.frombuffer((FOLDER/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
            cs,ids,_=detector.detectMarkers(cv.cvtColor(image,cv.COLOR_BGR2GRAY))
            ids=[] if ids is None else list(map(int,ids.ravel()))
            for ident in (40,5):counts[str(ident)]+=ident in ids
            pair=40 in ids and 5 in ids;counts['paired']+=pair
            details.append(dict(image=f['image'],ids=ids,
                corners={str(i):np.asarray(c).reshape(4,2).tolist() for i,c in zip(ids,cs)}))
            if pair:
                palms=marker_solutions(cs,ids,40,.0282,K,dist)
                tops=marker_solutions(cs,ids,5,.0107,K,dist)
                hypotheses=[]
                for (a,(Rp,tp,ep)),(b,(Rt,tt,et)) in itertools.product(enumerate(palms),enumerate(tops)):
                    center=tt-Rt[:,2]*.0275
                    hypotheses.append(dict(palm_branch=a,cube_branch=b,reprojection_sum_px=ep+et,
                        cube_center_marker40_m=(Rp.T@(center-tp)).tolist(),cube_top_rotation_marker40=(Rp.T@Rt).tolist(),
                        top_normal_angle_to_palm_normal_deg=float(np.degrees(np.arccos(np.clip(Rp[:,2]@Rt[:,2],-1,1))))))
                pose_rows.append(dict(image=f['image'],camera_read_monotonic=f['camera_read_monotonic'],
                    hypotheses=hypotheses,paired_encoder_readings_available=False))
        summaries.append(dict(name=profile['name'],requested=profile['settings'].get('requested_exposure'),
            exposure_readback=profile['settings_after_capture']['exposure'],frames=len(details),counts=counts,details=details))
    chosen=next(p for p in summaries if p['name']==source['best_profile'])
    assert chosen['exposure_readback']==-7 and chosen['counts']['paired']>=3
    report=dict(status='exposure_minus7_readability_confirmed',source=FOLDER.name,
        source_sha256=hashlib.sha256(blob).hexdigest(),profiles=summaries,relative_pose_rows=pose_rows,
        selected_requested_exposure=-7.,auto_request=.25,auto_readback=-1.,
        auto_mode_meaning_verified=False,original_exposure_restored=-5.,
        runtime_motor_camera_settings_changed=False,registration_verified=False,
        previous_grasp_encoders_reused=False,
        next_missing_evidence='Fresh encoder readings bracketed around the same -7 exposure photos; current sweep has no joint measurements.',
        hardware_commands_sent=0,real_camera_or_serial_opened=False)
    (FOLDER/'exposure_review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('SOURCE_SHA',report['source_sha256'])
    for p in summaries:print(p['name'],'exposure',p['exposure_readback'],'counts',p['counts'])
    print('Relative poses: diagnostic only; no paired current encoders')
    for row in pose_rows:
        best=min(row['hypotheses'],key=lambda h:h['reprojection_sum_px'])
        print(row['image'],'center mm',np.round(np.array(best['cube_center_marker40_m'])*1000,2).tolist(),
            'normal angle',round(best['top_normal_angle_to_palm_normal_deg'],2),
            'reprojection',round(best['reprojection_sum_px'],2))

if __name__=='__main__':main()
