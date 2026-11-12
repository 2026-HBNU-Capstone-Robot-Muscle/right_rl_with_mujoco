"""Extract pure image geometry without analysis/latest-pointer side effects."""
import ast,json,hashlib
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent
OUT=R/'right_hand_rotation_20261030/placement_alignment_capture';source=R/'placement_alignment_capture_20261030_092639_869097/offline_review/two_face_placement_comparison.json'
def functions(filename,names):
 s=(R/filename).read_text(encoding='utf-8');nodes=ast.parse(s).body;found={n.name:ast.get_source_segment(s,n) for n in nodes if isinstance(n,ast.FunctionDef) and n.name in names};assert set(found)==set(names);return '\n\n'.join(found[n] for n in names)+'\n'
header='''"""Image geometry only. No motor, camera or window construction at import."""
import json
from pathlib import Path
import cv2 as cv,numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent
cal=np.load(ROOT/'camera1_720.npz');K,D=cal['K'],cal['dist']
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),[3,5,41],hand_id=40,required_cube_count=1)
RG=np.array([[0.,0.,-1.],[0.,1.,0.],[1.,0.,0.]])
'''
pure=header+functions('measure_inverse_side_marker_20261029.py',['poses'])+functions('compare_alignment_placement_20261030.py',['decoded','grid_poses'])+functions('fit_two_face_placement_20261030.py',['fused'])
(R/'placement_reference_geometry_20261030.py').write_text(pure,encoding='utf-8')
analysis=json.loads(source.read_bytes());group=analysis['groups']['single_forward_trial_20261030_052636_068402'];valid=[r['body_relative']['40'] for r in group['rows'] if '40' in r['body_relative']];rot=np.mean([v['face_rotation_body'] for v in valid],axis=0);u,s,v=np.linalg.svd(rot);rot=u@v;assert np.linalg.det(rot)>.999
model=dict(status='image_reference_only_not_robot_registration',reference_result='single_forward_trial_20261030_052636_068402/result.json',reference_photo='single_forward_trial_20261030_052636_068402/cube_before_grasp.jpg',body_marker_id=40,body_marker_black_side_m=.0282,cube_side_m=.055,center_body_m=(np.median([v['center_mm'] for v in valid],axis=0)*.001).tolist(),orange_rotation_body=rot.tolist(),source_frames=len(valid),green_frame_in_orange_axes=analysis['green_frame_in_orange_axes'],source_diagnostic=source.relative_to(R).as_posix(),source_diagnostic_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),absolute_robot_registration_verified=False,display_only=True,measurement_accuracy_mm_not_certified=True,precise90_rotation_not_verified=True,missing_marker_not_counted_as_detected=True)
(OUT/'image_reference.json').write_text(json.dumps(model,indent=2)+'\n',encoding='utf-8');print(json.dumps(model))
