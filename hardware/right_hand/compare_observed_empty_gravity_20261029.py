"""Check gravity-direction consistency using measured, stationary EMPTY poses.

This is an identification diagnostic, not a new calibration. Cable friction,
servo gain conversion and model masses remain uncertain; no execution approval.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np,mujoco
from hand_motor_mapping import SIM_TO_MOTOR_IDS,ticks_to_sim,TICK
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/empty_gravity_diagnostic'
def main():
    OUT.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint/checkpoint_model.mjb'))
    legacy=m.opt.gravity.copy();gain=m.actuator_gainprm[:,0].copy();ids=np.asarray(SIM_TO_MOTOR_IDS)
    records=[]
    for folder in ('reverse_yaw_empty_20261029_164720_101745','supported_inverse_empty_20261029_225256_285948'):
        r=json.loads((ROOT/folder/'result.json').read_bytes());seen=set()
        for phase in ('stage_4_place_hold','stage_4_inverse_empty_gap_hold','stage_6_grasp_hold','stage_8_horizontal_hold','stage_10_remove_hold','stage_2_fixed_hold'):
            samples=[s for s in r['samples'] if s['phase']==phase][-12:]
            if len(samples)<10:continue
            goals=np.asarray([s['goal'] for s in samples]);actual=np.asarray([s['actual'] for s in samples])
            if np.max(np.ptp(goals,axis=0))>0 or np.max(np.ptp(actual,axis=0))>3:continue
            key=tuple(goals[-1])
            if key in seen:continue
            seen.add(key);q=np.median(actual,axis=0);error=ticks_to_sim(goals[-1])-ticks_to_sim(q)
            d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:]=[0,0,10,1,0,0,0];d.qvel[:]=0
            columns=[]
            for axis in np.eye(3):
                m.opt.gravity[:]=axis;mujoco.mj_forward(m,d);columns.append(d.qfrc_bias[:16].copy())
            G=np.asarray(columns).T;response=gain*error
            records.append(dict(folder=folder,phase=phase,actual_ticks=q.tolist(),goal_ticks=goals[-1].tolist(),
                current_median_mA=np.median([s['current_ma'] for s in samples],axis=0).tolist(),
                measured_error_sim_rad=error.tolist(),nominal_PD_torque=response.tolist(),gravity_basis_Nm_per_m_s2=G.tolist()))
    results={}
    for name,indices in (('all16',np.arange(16)),('roots',np.flatnonzero(np.isin(ids,[4,8,12,16])))):
        matrices=[];targets=[]
        for r in records:
            A=np.asarray(r['gravity_basis_Nm_per_m_s2'])[indices];b=np.asarray(r['nominal_PD_torque'])[indices]
            matrices.append(A);targets.append(b)
        A=np.vstack(matrices);b=np.concatenate(targets);solution,_,rank,svals=np.linalg.lstsq(A,b,rcond=None)
        def residual(g):
            predicted=A@g;scale=max(0.,float(predicted@b/(predicted@predicted))) if predicted@predicted>1e-15 else 0.
            err=predicted*scale-b;signmask=abs(b)>.004
            return dict(positive_shared_scale=scale,RMS_Nm=float(np.sqrt(np.mean(err**2))),
                sign_agreement=float(np.mean(np.sign(predicted[signmask])==np.sign(b[signmask]))) if np.any(signmask) else None)
        checks={'legacy':residual(legacy),'global_downZ':residual(np.array([0.,0.,-9.81])),
            'opposite20degX':residual(np.array([9.81*np.sin(np.radians(20)),0.,-9.81*np.cos(np.radians(20))]))}
        norm=np.linalg.norm(solution);results[name]=dict(rank=int(rank),effective_gravity_vector=solution.tolist(),
            estimated_direction=solution.tolist() if norm==0 else (solution/norm).tolist(),
            fitted_RMS_Nm=float(np.sqrt(np.mean((A@solution-b)**2))),candidate_checks=checks)
    report=dict(status='EMPTY_static_gravity_direction_diagnostic_only',records=records,identification=results,legacy_gravity=legacy.tolist(),
        actuator_nominal_gains=gain.tolist(),sim_order_motor_ids=ids.tolist(),model_mass_gain_and_cable_friction_not_identified=True,
        model_or_controller_calibration_changed=False,hardware_execution_allowed=False,devices_opened=False)
    (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='records'}),flush=True)
if __name__=='__main__':main()
