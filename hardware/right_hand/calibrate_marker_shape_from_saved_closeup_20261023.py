"""Independent old close-up shape diagnostic; no hardware calibration install."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from sac_closeup_grid_audit import analyze
ROOT=Path(__file__).resolve().parent
folder=ROOT/'cube_supported_rotation_trial_20261023_034640_750784'
capture=json.loads((ROOT/'leap_cube_closeup_result.json').read_bytes())
source=Path(capture['folder'])
with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
shapes=[];errors=[]
keep=[i for i in range(9) if i!=4]
grid=np.array([[(i%3-1)*55/3,(i//3-1)*55/3] for i in keep],np.float32)
for f in capture['frames']:
    im=cv.imdecode(np.frombuffer((source/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    marker=np.asarray(f['quality']['corners_px'],float).reshape(4,2)
    result,pts=analyze(im,marker,K,dist)
    und=cv.undistortPoints(pts[:,None].astype(float),K,dist,P=K).reshape(-1,2)
    H,_=cv.findHomography(grid,und,0)
    undmarker=cv.undistortPoints(marker[:,None],K,dist,P=K).reshape(-1,2)
    onplane=cv.perspectiveTransform(undmarker[None],np.linalg.inv(H))[0]
    onplane[:,1]*=-1 # right/up plane; preserves the marker corner ID order
    shapes.append(onplane);errors.append(result['homography_mean_error_px'])
median=np.median(shapes,axis=0)
out=dict(status='independent_saved_closeup_marker_shape_diagnostic',source=str(source),
    marker_id=5,cube_side_mm=55,marker_corners_in_face_plane_mm=median.tolist(),
    measured_repeated_corner_spread_mm=np.ptp(shapes,axis=0).tolist(),
    center_offset_mm=median.mean(axis=0).tolist(),
    edge_lengths_mm=np.linalg.norm(median-np.roll(median,1,axis=0),axis=1).tolist(),
    maximum_grid_homography_error_px=max(errors),physical_calibration_verified=False,
    hardware_execution_allowed=False,hardware_commands_sent=0,
    limitations=['55/3mm tile-center pitch is approximate.',
                 'Sticker centroids, homography distortion, image blur and lens calibration produce systematic uncertainty.',
                 'Repeatability is not absolute metrology; do not overwrite the user-supplied physical marker size.'])
(folder/'independent_marker_shape_diagnostic.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2))
