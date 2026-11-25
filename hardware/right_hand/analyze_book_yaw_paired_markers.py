"""Same-frame marker/grid diagnostics from saved baseline and after-turn images."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector
from analyze_readonly_direct_pose_20261023 import analyze_grid
from review_prepared_cube_20261022 import marker_solutions
from hand_registration_geometry import candidate

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'book_small_yaw_trial_20261024_215736_594865'
r=json.loads((FOLDER/'result.json').read_bytes())
frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames'];selected=[]
for phase in ('stage_4_book_grasp_hold','stage_6_book_yaw_hold'):
    group=[f for f in frames if f['phase']==phase]
    selected.extend(group[i] for i in np.linspace(0,len(group)-1,6).astype(int))
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),hand_id=40,required_cube_count=1)
with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
reg=candidate();Rwm=np.array(reg['hand_to_world_candidate']);gm=np.array(reg['grasp_site_in_marker_m_candidate'])
rows=[];hypotheses=[]
for f in selected:
    im=cv.imdecode(np.frombuffer((FOLDER/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    cs,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY));ids=[] if ids is None else ids.ravel().tolist()
    row=dict(image=f['image'],phase=f['phase'],decoded_ids=ids,
        marker_corners={str(i):np.array(c).reshape(4,2).tolist() for i,c in zip(ids,cs)})
    if 5 in ids:
        try:row['grid_diagnostic']=analyze_grid(im,row['marker_corners']['5'],K,dist)[0]
        except ValueError as exc:row['grid_error']=str(exc)
    palms=marker_solutions(cs,ids,40,.0282,K,dist)
    cubes=marker_solutions(cs,ids,5,.0107,K,dist)
    for ip,(Rp,tp,ep) in enumerate(palms):
        Rcw=Rp@Rwm.T;tcw=tp+Rp@gm-Rcw@np.array([.11,0,.03])
        for ic,(Rc,tc,ec) in enumerate(cubes):
            center=tc-Rc[:,2]*.0275
            hypotheses.append(dict(image=f['image'],phase=f['phase'],palm_branch=ip,cube_branch=ic,
                marker_errors_px=[ep,ec],cube_center_world_m=(Rcw.T@(center-tcw)).tolist(),
                cube_rotation_world=(Rcw.T@Rc).tolist(),world_to_camera_rotation=Rcw.tolist(),
                world_to_camera_translation=tcw.tolist()))
    rows.append(row)
out=dict(source=FOLDER.name,rows=rows,hypotheses=hypotheses,
    marker_to_model_registration_verified=False,hardware_commands_sent=0,
    solved_green_grid_used_only_for_offline_diagnostic=True)
(FOLDER/'paired_marker_diagnostic.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(dict(decoded_ids=[s['decoded_ids'] for s in rows],paired_hypotheses=len(hypotheses),
    grid_errors=[s.get('grid_error') for s in rows],grid_fits=[s.get('grid_diagnostic',{}).get('grid_pnp_mean_error_px') for s in rows])))
