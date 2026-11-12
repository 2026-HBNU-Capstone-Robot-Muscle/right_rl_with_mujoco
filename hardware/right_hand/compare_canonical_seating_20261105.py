"""Initial-support comparison for a common terminal grip. Offline only.

These model offsets are hypotheses, not instructions to move the real cube.
They do not demonstrate fingertip reach, layer clearance, or a loaded path.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore,inspect_for
from replay_body_hold_from_placement_20261104 import initial
from cube_contact_material_policy_20261029 import apply_material
R=Path(__file__).resolve().parent;CP=R/'right_hand_rotation_20261105/preintegration/reference_end';O=R/'right_hand_rotation_20261105/canonical_body_grip'

def unit(v):
    n=np.linalg.norm(v)
    if n<1e-8:raise RuntimeError('Ambiguous anatomical direction')
    return v/n

def main():
    O.mkdir(parents=True,exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));ref=np.load(CP/'end_state.npz');d=restore(m,ref);up=np.array(ref['up']);palm=d.xpos[m.body('palm').id].copy();roots=np.array([d.xanchor[j] for j in (0,4,8)]);plane=lambda v:v-up*(v@up)
    forward=unit(plane(roots.mean(0)-palm));side=plane(roots[0]-roots[1]);side=unit(side-forward*(side@forward));C=np.array(ref['initial_cube_center']);B=np.array(ref['initial_cube_rotation']);q=Rotation.from_matrix(B).as_quat();quat=np.r_[q[3],q[:3]];middle=(roots[0]+roots[1])/2
    candidates=[('current_reference',C),('palmward_10mm',C-forward*.01),('indexward_10mm',C+side*.01),('index_middle_midline',C-side*((C-middle)@side))]
    opened=np.load(R/'right_hand_rotation_20261104/forward_hold/candidate.npz')['start'];gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();hand,support,check=inspect_for(m);rows=[]
    for label,center in candidates:
        cases=[];began=time.perf_counter()
        for scale in (.5,1.,1.5):
          for friction in (.2,.3,.5):
            m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;apply_material(m,friction,hand,m.geom('cube').id)
            try:
                z,probe=initial(m,opened,up,np.r_[center,quat]);data=restore(m,z);cube=m.geom('cube').id;CB=data.xmat[m.body('cube').id].reshape(3,3);delta=z['qpos'][-7:-4]-center
                supported=any(cube in (int(c.geom1),int(c.geom2)) and int(m.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)]) in hand and c.dist<.0004 for c in data.contact)
                tilt=float(np.degrees(np.arccos(np.clip(CB[:,1]@B[:,1],-1,1))));lateral=float(np.linalg.norm(plane(delta))*1000)
                case=dict(gain_scale=scale,friction=friction,initial_placement=probe,hand_supported=bool(supported),lateral_shift_mm=lateral,tilt_from_seed_deg=tilt,initial_support_candidate_passed=bool(probe['passed'] and supported and lateral<=10 and tilt<=15))
            except Exception as exc:case=dict(gain_scale=scale,friction=friction,error=str(exc),initial_support_candidate_passed=False)
            cases.append(case)
        row=dict(label=label,seed_center_world_m=center.tolist(),offset_from_reference_in_model_mm=((center-C)*1000).tolist(),support_passed=sum(v['initial_support_candidate_passed'] for v in cases),cases=cases,elapsed_seconds=time.perf_counter()-began);rows.append(row)
        report=dict(status='initial_support_comparison_only',same_cube_orientation_seed=True,hand_start_unchanged=True,body_clearance_and_grasp_not_verified=True,physical_model_registration_verified=False,hand_force_and_limits_unchanged=True,cube_is_free_during_settling=True,new_operator_motion_admitted=False,hardware_execution_allowed=False,devices_opened=False,candidates=rows,anatomical_directions_world=dict(up=up.tolist(),toward_finger_roots=forward.tolist(),toward_index=side.tolist()))
        (O/'seating_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k!='cases'}),flush=True)
    specification=dict(status='design_goal_not_a_validated_grasp',purpose='One reproducible cube handoff location before the other hand turns the upper layer',fixed_at_handoff=['cube center in palm frame','cube top-layer height','cube edge alignment and tilt','right-hand lower-body grasp'],allowed_during_reorientation='bounded temporary movement and contact changes; return to the common terminal grip',cube_face_color_does_not_define_terminal_center=True,current_reference_is_first_candidate_not_final_selection=True,initial_placement_is_not_full_grasp_or_layer_clearance_proof=True,requires=['whole-hand upper-layer sweep clearance','free-cube transition with original health/native/contact bounds','repeatable physical terminal pose','integrated upper-layer-load verification'],metric_target_not_selected=True,hardware_execution_allowed=False)
    (O/'goal_specification.json').write_text(json.dumps(specification,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
