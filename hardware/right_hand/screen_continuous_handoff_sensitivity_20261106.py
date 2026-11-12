"""Fresh complete-path perturbation probes for the new nominal handoff.

Original90 target and station are retained under placement errors. These
are model hypotheses, not calibrated physical tolerances or a hardware policy.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from canonical_grip_dynamics_20261105 import run
from research_rotation_body_hold_20261104 import restore
from checkpoint_quarter_review_20261106 import quarter_reasons
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--group',choices=('placement','friction','gain'),required=True);a=ap.parse_args();parent=ROOT/'continuous_handoff';review=json.loads((parent/'review.json').read_bytes());assert review['quarter_and_handoff_passed'];model=R/review['source_model'];initial=R/review['source_initial_state'];candidate=parent/'candidate.npz';path=np.load(candidate);up=np.load(initial)['up'];target_center=review['planned_nominal_common_center_world_m'];target=upright_yaw_target(np.load(initial)['initial_cube_rotation'],up,90);root=ROOT/'continuous_sensitivity'/a.group;root.mkdir(parents=True,exist_ok=True);cases=[]
    if a.group=='placement':parameters=[dict(forward_mm=dx,side_mm=dy,yaw_deg=angle) for dx,dy,angle in ((-1,0,0),(1,0,0),(0,-1,0),(0,1,0),(0,0,-2),(0,0,2))]
    elif a.group=='friction':parameters=[dict(friction=x) for x in (.2,.25,.4,.5)]
    else:parameters=[dict(gain_scale=x) for x in (.8,.9,1.1,1.2)]
    for index,p in enumerate(parameters):
        began=time.perf_counter();m=mujoco.MjModel.from_binary_path(str(model));z=dict(np.load(initial));original_reference=z['initial_cube_rotation'].copy();original_center=z['initial_cube_center'].copy();preserved={k:getattr(m,k).copy() for k in ('geom_size','geom_contype','geom_conaffinity','jnt_range','actuator_forcelimited','actuator_forcerange')}
        if a.group=='placement':
            z['qpos']=z['qpos'].copy();z['qpos'][-7:-4]+=np.array([p['forward_mm'],p['side_mm'],0.])/1000
            turn=Rotation.from_rotvec(up*np.radians(p['yaw_deg']));rot=turn*Rotation.from_quat(z['qpos'][-4:][[1,2,3,0]]);xyzw=rot.as_quat();z['qpos'][-4:]=xyzw[[3,0,1,2]];z['qacc_warmstart']=np.zeros_like(z['qacc_warmstart'])
        elif a.group=='friction':m.geom_friction[:,0]=p['friction']
        else:
            scale=p['gain_scale'];m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1:3]*=scale
        assert np.array_equal(z['initial_cube_rotation'],original_reference) and np.array_equal(z['initial_cube_center'],original_center)
        folder=root/str(index);folder.mkdir(exist_ok=True);np.savez_compressed(folder/'initial_state.npz',**z);v=run(m,z,path['goals'],1,folder/'end_state.npz',None,True,5,('if_tip','th_tip'));d=restore(m,np.load(folder/'end_state.npz'));error=orientation_error_deg(d.xmat[m.body('cube').id].reshape(3,3),target);why=quarter_reasons(v,target_center,True,error)
        for k,value in preserved.items():assert np.array_equal(getattr(m,k),value),k
        case=dict(parameters=p,dynamic=v,orientation_error_deg=error,quarter_and_handoff_passed=not why,reasons=why,elapsed_seconds=time.perf_counter()-began);cases.append(case);report=dict(status='new_continuous_handoff_sensitivity',group=a.group,candidate=candidate.relative_to(R).as_posix(),candidate_sha256=sha(candidate),source_model=model.relative_to(R).as_posix(),source_model_sha256=sha(model),source_initial_state=initial.relative_to(R).as_posix(),source_initial_state_sha256=sha(initial),planned_cases=len(parameters),cases=cases,original90_reference_and_station_preserved=True,full_initial_path_replayed_no_resets=True,original_native_ranges_force_caps_and_geometry_unchanged=True,nominal_baseline_reused_not_another_trial=True,actual_model_parameter_ranges_not_calibrated=True,devices_opened=False,hardware_execution_allowed=False);(root/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(group=a.group,index=index,parameters=p,passed=not why,safe=v['safety_passed'],completed=v['completed_goals'],yaw=v['final_yaw_from_placement_deg'],reasons=why,seconds=case['elapsed_seconds'])),flush=True)
if __name__=='__main__':main()
