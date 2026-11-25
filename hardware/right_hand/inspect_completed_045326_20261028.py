"""Saved-frame marker diagnostics and encoder inventory; no devices."""
import json,shutil,importlib.util
from pathlib import Path
import cv2 as cv,numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent;folder=ROOT/'horizontal_quarter_trial_20261028_045326_636152'
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),required_cube_count=1)
out={}
for name in ('cube_before_grasp.jpg','cube_before_horizontal_turn.jpg','cube_horizontal_turn_hold.jpg','cube_after_open.jpg'):
    im=cv.imdecode(np.frombuffer((folder/name).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    corners,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
    found={} if ids is None else {str(int(i)):np.asarray(c).reshape(4,2).tolist() for i,c in zip(ids.ravel(),corners)}
    out[name]=found
    if '40' in found and '5' in found:
        hp=cv.getPerspectiveTransform(np.float32(found['40']),np.float32([[0,0],[1,0],[1,1],[0,1]]))
        q=cv.perspectiveTransform(np.float32(found['5'])[None,:,:],hp)[0]
        u=q[1]-q[0];v=q[3]-q[0];angle=np.degrees(np.arctan2(u[1]-v[0],u[0]+v[1]))
        out[name]['plane_diagnostic']=dict(angle_deg=float(angle),edge_ratio=float(np.linalg.norm(u)/np.linalg.norm(v)),orthogonality_cos=float(np.dot(u,v)/np.linalg.norm(u)/np.linalg.norm(v)))
calibrations=[]
places=[ROOT,ROOT.parent/'work/Leap_Test/vision/rubik_center',ROOT.parent/'work/Leap_Test/vision']
for place in places:
    for path in place.glob('*720*.npz'):
        z=np.load(path);calibrations.append(dict(path=str(path),keys=z.files))
out['calibrations']=calibrations;out['ffmpeg']=shutil.which('ffmpeg')
out['imageio_ffmpeg_available']=importlib.util.find_spec('imageio_ffmpeg') is not None
z=json.loads((folder/'result.json').read_bytes());out['photo_metadata']={k:v for k,v in z.items() if 'photo' in k}
frames=json.loads((folder/'camera/frames.json').read_bytes())['frames']
out['timeline']=dict(first=frames[0],last=frames[-1],frames=len(frames))
(folder/'initial_marker_diagnostic.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print(json.dumps(out))
