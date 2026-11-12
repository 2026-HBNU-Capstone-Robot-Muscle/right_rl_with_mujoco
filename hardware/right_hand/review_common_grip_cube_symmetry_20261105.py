"""Static cube-label symmetry review; these are NOT24 reorientation runs.

Separate pose copies permute the labels of an equal-sided rigid cube while
keeping its occupied volume and the hand pose the same. No loaded motion,
physical orientation estimate, friction identification or force proof follows.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for
from cube_orientation_routes_20261028 import validate,NORMALS,NAMES
from cube_task_frame_20261105 import working_rotation
from upper_layer_separation_certificate_20261105 import SweepCertificate

def main():
    folder=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3';source=folder/'end_state.npz';m=mujoco.MjModel.from_binary_path(str(folder/'model.mjb'));z=np.load(source);initial=restore(m,z);cube=m.geom('cube').id;body=m.body('cube').id;B0=initial.xmat[body].reshape(3,3).copy();C=initial.geom_xpos[cube].copy();hand,_,check=inspect_for(m)
    assert int(m.geom_type[cube])==6 and np.max(np.ptp(m.geom_size[cube]))<1e-12 and np.ptp(m.body_inertia[body])<1e-12 and np.allclose(m.geom_quat[cube],[1,0,0,0])
    corners=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*m.geom_size[cube];reference=corners@B0.T+C;cases=[]
    for count,(logical,(S,moves)) in enumerate(sorted(validate().items())):
        B=B0@S;n=S.T@np.array([0,-1,0]);face=int(np.flatnonzero(np.all(NORMALS==n,axis=1))[0]);d=restore(m,z);q=Rotation.from_matrix(B).as_quat();d.qpos[-4:]=np.r_[q[3],q[:3]];d.qvel[:]=0;mujoco.mj_forward(m,d);W=working_rotation(d.geom_xmat[cube].reshape(3,3),face);certificate=SweepCertificate(m,hand,.3);gap,bad=certificate.query(d,d.geom_xpos[cube],W);vertices=corners@d.geom_xmat[cube].reshape(3,3).T+d.geom_xpos[cube];shape_error=float(max(min(np.linalg.norm(reference-v,axis=1)) for v in vertices))*1000;su,sp=check(d);cube_pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        cases.append(dict(index=count,selected_top_face_id=face,selected_top_face_name=NAMES[face],label_rotation=S.tolist(),occupied_cube_vertex_error_mm=shape_error,static_layer_gap_mm=gap,certificate_failed_geoms=bad,static_self_mm=max(sp.values(),default=0.),static_support_mm=max(su.values(),default=0.),static_cube_penetration_mm=cube_pen,passed=shape_error<1e-7 and not bad and max(sp.values(),default=0.)<=1e-6 and max(su.values(),default=0.)<=.15 and cube_pen<=1))
    assert len(cases)==24
    report=dict(status='static_geometric_cube_label_symmetry_only',source=str(source.relative_to(R)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),cases=cases,passed=sum(c['passed'] for c in cases),same_hand_pose_and_cube_occupied_volume=True,top_face_bookkeeping_varies_without_green_up_assumption=True,selected_face_counts={name:sum(c['selected_top_face_name']==name for c in cases) for name in NAMES},these_are_not_continuous_reorientation_trials=True,no_dynamic_or_reaction_load_proof=True,physical_top_faces_not_observed_or_verified=True,hardware_execution_allowed=False,devices_opened=False)
    (ROOT/'cube_symmetry_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='cases'}|{'gap_range_mm':[min(c['static_layer_gap_mm'] for c in cases),max(c['static_layer_gap_mm'] for c in cases)]}),flush=True)
if __name__=='__main__':main()
