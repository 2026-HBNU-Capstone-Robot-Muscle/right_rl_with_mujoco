"""Independent OpenCV synthetic fixtures + diagnostic archived real frames."""
import json
from pathlib import Path
import numpy as np
import cv2 as cv
from leap_anchor_multiview import pair_observation,square,SIDES
from sac_registration_from_anchor import detect
ROOT=Path(__file__).resolve().parent

def main():
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    report=dict(intrinsic=K.tolist(),distortion=dist.ravel().tolist(),
                marker_sides_m={str(k):v for k,v in SIDES.items()},frames=[])
    rng=np.random.default_rng(42)
    relrv=np.array([.9,.1,.2]);relR=cv.Rodrigues(relrv)[0];relt=np.array([.04,.025,.01])
    for view,rv in enumerate(([.2,.05,0],[.2,.4,0],[-.15,-.3,.1])):
        R=cv.Rodrigues(np.asarray(rv,float))[0];t=np.array([-.03,-.01,.4])
        for sample in range(3):
            cs=[]
            for ident in (40,41):
                points=square(SIDES[ident]);points=points@relR.T+relt if ident==40 else points
                pixels=cv.projectPoints(points,np.asarray(rv,float),t,K,dist)[0].reshape(4,2)
                cs.append(pixels+rng.normal(0,.08,(4,2)))
            pair=pair_observation(cs,np.array([[40],[41]]),K,dist)
            if pair is None:raise RuntimeError('Synthetic pair solve failed')
            report['frames'].append(dict(view=view,markers=pair))
    report['truth_relative_rotation']=relR.tolist();report['truth_relative_translation_m']=relt.tolist()
    points=square(.03);test_rv=np.array([.4,-.2,.1]);test_t=np.array([.03,-.02,.35])
    report['projection_fixture']=dict(points=points.tolist(),rvec=test_rv.tolist(),tvec=test_t.tolist(),
        expected=cv.projectPoints(points,test_rv,test_t,K,dist)[0].reshape(4,2).tolist())
    (ROOT/'sac_anchor_bundle_synthetic_fixture.json').write_text(json.dumps(report,indent=2)+'\n')
    real={k:v for k,v in report.items() if k in ('intrinsic','distortion','marker_sides_m')};real['frames']=[]
    folder=ROOT/'sequence_vision_20261013_213113_258536'
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    for path in sorted(folder.glob('*.png')):
        _,cs,ids,_=detect(path,dictionary)
        pair=pair_observation(cs,ids,K,dist)
        if pair:
            real['frames'].append(dict(view=len(real['frames']),image=path.name,markers=pair))
    real['source_folder']=str(folder)
    (ROOT/'sac_anchor_bundle_archive_input.json').write_text(json.dumps(real,indent=2)+'\n')
    print('Synthetic:',len(report['frames']),'views/frames. Archived paired frames:',len(real['frames']))

if __name__=='__main__':main()
