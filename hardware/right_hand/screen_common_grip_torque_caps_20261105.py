"""Full-path sensitivity under stricter, explicitly hypothetical torque caps.

The saved right-hand model has no enabled actuator-force limit. Real450mA
cannot be equated directly to a fixed joint torque without calibration.
These caps are research hypotheses, not a conversion or a motor-setting edit.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
from unittest.mock import patch
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from canonical_grip_dynamics_20261105 import run
from canonical_grip_task_review_20261105 import task_reasons
from replay_body_hold_from_placement_20261104 import initial

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--cap',type=float,choices=(.10,.15,.20,.23),required=True);a=ap.parse_args()
    folder=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3';model=folder/'model.mjb';m=mujoco.MjModel.from_binary_path(str(model))
    candidate=ROOT/'tip_squeeze/id1/plus12/candidate.npz';path=np.load(candidate);source_flags=m.actuator_forcelimited.copy();source_ranges=m.actuator_forcerange.copy()
    preserved={k:getattr(m,k).copy() for k in ('actuator_gainprm','actuator_biasprm','jnt_range','actuator_ctrlrange','geom_size','geom_friction','geom_contype','geom_conaffinity','body_mass','body_inertia')}
    low=np.where(source_flags,source_ranges[:,0],-np.inf);high=np.where(source_flags,source_ranges[:,1],np.inf)
    m.actuator_forcelimited[:]=1;m.actuator_forcerange[:,0]=np.maximum(low,-a.cap);m.actuator_forcerange[:,1]=np.minimum(high,a.cap)
    if np.any(m.actuator_forcerange[:,0]>=m.actuator_forcerange[:,1]):raise ValueError('empty stricter force interval')
    mujoco.mj_setConst(m,mujoco.MjData(m))
    out=ROOT/f'torque_cap_sensitivity/cap{a.cap:g}';out.mkdir(parents=True,exist_ok=True);mujoco.mj_saveModel(m,str(out/'model.mjb'))
    prior=json.loads((R/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());seed=np.array(next(c for c in prior['loaded_physics'] if (c['book_front_mm'],c['gain_scale'],c['friction'])==(-10.,1.,.3))['settled_cube_qpos']);up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    maxima=np.zeros(m.nu);steps=0;original_step=mujoco.mj_step;began=time.perf_counter()
    def observe_step(model,data,*args,**kwargs):
        nonlocal maxima,steps
        original_step(model,data,*args,**kwargs);steps+=1;maxima=np.maximum(maxima,abs(data.actuator_force))
        if np.any(data.actuator_force<model.actuator_forcerange[:,0]-1e-10) or np.any(data.actuator_force>model.actuator_forcerange[:,1]+1e-10):raise RuntimeError('model actuator force exceeded its active cap')
    with patch.object(mujoco,'mj_step',observe_step):
        z,placement=initial(m,path['start'],up,seed);np.savez_compressed(out/'initial_state.npz',**z)
        dynamic=run(m,z,path['goals'],1417,out/'end_state.npz',None,True) if placement['passed'] else None
    for k,v in preserved.items():assert np.array_equal(getattr(m,k),v),k
    reasons=task_reasons(dynamic)
    if not placement['passed']:reasons.append('initial_placement')
    report=dict(status='stricter_offline_torque_cap_full_path_sensitivity',hypothetical_cap_each_joint_Nm=a.cap,source_model=str(model.relative_to(R)),source_model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),original_actuator_force_limit_enabled=source_flags.tolist(),new_actuator_force_limit_enabled=m.actuator_forcelimited.tolist(),placement=placement,dynamic=dynamic,task_passed=not reasons,reasons=reasons,observed_actuator_force_max_Nm_by_sim_order=maxima.tolist(),observed_maximum_actuator_force_Nm=float(max(maxima)),observed_mj_steps=steps,force_observer_only_calls_original_mj_step=True,original_geometry_gains_native_ranges_and_dynamic_guards_preserved=True,real450mA_to_torque_relation_not_calibrated=True,not_an_operator_current_setting_recommendation=True,source_reference_url='https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#goal-current102',current_sensing_reference_note='Manufacturer describes XC330 input-source current sensing; not a direct phase-current measurement.',devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(cap=a.cap,placement=placement['passed'],safety=dynamic['safety_passed'] if dynamic else None,task=not reasons,reasons=reasons,maximum_model_torque=report['observed_maximum_actuator_force_Nm'],elapsed=report['elapsed_seconds'])),flush=True)
if __name__=='__main__':main()
