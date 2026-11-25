"""Bounded thumb-only IK followed by free-cube replay; research, no admission."""
from pathlib import Path
import sys,json,time,hashlib
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'.runtime/sim-win'))
import numpy as np
import mujoco
from scipy.optimize import least_squares
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from dynamic_common_grip_search_20261105 import ramp
from canonical_grip_dynamics_20261105 import run

SOURCE=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
CP=ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction'
OUT=ROOT/'right_hand_rotation_20261109/thumb_lowering_after_probe'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True)
    model=mujoco.MjModel.from_binary_path('source.mjb',assets={'source.mjb':SOURCE.read_bytes()})
    z=np.load(CP/'checkpoint_1042.npz',allow_pickle=False);old=np.load(CP/'checkpoint_927.npz',allow_pickle=False)
    d=restore(model,z);p=restore(model,z);before=restore(model,old);cube=model.geom('cube').id;tip=model.geom('th_tip').id
    contact=min((c for c in before.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}),key=lambda c:c.dist)
    local_point=before.geom_xmat[tip].reshape(3,3).T@(contact.pos-before.geom_xpos[tip])
    cube_local=before.geom_xmat[cube].reshape(3,3).T@(contact.pos-before.geom_xpos[cube])
    reference_normal=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1)
    local_normal=before.geom_xmat[tip].reshape(3,3).T@reference_normal
    cube_normal=before.geom_xmat[cube].reshape(3,3).T@reference_normal
    hand,_,check=inspect_for(model);cache=precompute(model,hand)
    thumb_cache=[row for row in cache if model.body(int(model.geom_bodyid[row[0]])).name.startswith('th_')]
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();base=d.qpos.copy();pd=d.ctrl-d.qpos[:16];qbase=d.qpos[:16].copy()
    active=np.arange(12,16);lo,hi,_,_=native(model)
    from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
    ids=np.array(SIM_TO_MOTOR_IDS)-1;low=(np.ceil(lo[ids])+8-2048)*TICK-pd;high=(np.floor(hi[ids])-8-2048)*TICK-pd
    initial_goal=motor(d.ctrl);cases=[]
    for height in (6.,0.,-6.,-12.):
        target_local=cube_local.copy();target_local[1]=-height/1000.;target=C+B@target_local;target_normal=B@cube_normal
        def measure(x):
            p.qpos[:]=base;p.qpos[active]=x;p.ctrl[:]=p.qpos[:16]+pd;p.qvel[:]=0;mujoco.mj_forward(model,p)
            G=p.geom_xmat[tip].reshape(3,3);point=p.geom_xpos[tip]+G@local_point
            clear=gaps(model,p,C,B,thumb_cache,True);su,sp=check(p)
            penetration=max((max(0.,-float(c.dist)*1000) for c in p.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.r_[(point-target)*1000*5,(G@local_normal-target_normal)*2,(x-qbase[active])*.3,
                         [max(0.,.5-v)*8 for _,v in clear],max(sp.values(),default=0.)*1500,max(su.values(),default=0.)*1500,max(0.,penetration-.6)*700]
        seeds=[qbase[active].copy()]
        alternate=qbase[active].copy();alternate[2]-=.2;alternate[3]-=.2;seeds.append(alternate)
        fits=[]
        for seed in seeds:
            fit=least_squares(measure,np.clip(seed,low[active]+1e-9,high[active]-1e-9),bounds=(low[active],high[active]),max_nfev=140,ftol=1e-7,xtol=1e-7,gtol=1e-7,diff_step=1e-5)
            residual=measure(fit.x);fits.append((float(np.linalg.norm(residual)),fit.x.copy(),fit.nfev))
        cost,x,evaluations=min(fits,key=lambda row:row[0]);residual=measure(x);goal=motor(p.ctrl)
        point_error=float(np.linalg.norm(residual[:3])/5);all_gaps=sorted(gaps(model,p,C,B,cache,True),key=lambda row:row[1]);thumb_gap=min(value for _,value in gaps(model,p,C,B,thumb_cache,True));su,sp=check(p)
        geometry_ok=bool(point_error<.8 and thumb_gap>.3 and not sp and max(su.values(),default=0.)<=.15 and np.all(goal>=lo) and np.all(goal<=hi))
        folder=OUT/f'contact_height_{height:g}';folder.mkdir(exist_ok=True)
        rows=ramp(initial_goal,goal,134)
        np.savez_compressed(folder/'research_path.npz',start=initial_goal,goals=rows)
        dynamic=run(model,z,rows,1,folder/'end.npz',folder/'trajectory.npz',False)
        endpoint=restore(model,np.load(folder/'end.npz',allow_pickle=False));end_gaps=sorted(gaps(model,endpoint,endpoint.geom_xpos[cube],endpoint.geom_xmat[cube].reshape(3,3),cache,True),key=lambda row:row[1])
        row={'target_thumb_contact_height_mm':height,'only_thumb_goals_changed':bool(np.array_equal(goal[4:],initial_goal[4:])),
             'static':{'thumb_geometry_passed':geometry_ok,'contact_error_mm':point_error,'thumb_sweep_gap_mm':thumb_gap,'all_upper_gaps_mm':all_gaps,'cost':cost,'optimizer_evaluations':evaluations},
             'goal':goal.tolist(),'dynamic':dynamic,'free_endpoint_worst_upper_gaps_mm':end_gaps[:8],
             'complete_lower_grip_and_upper_sweep_verified':False}
        cases.append(row);(folder/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
        report={'status':'thumb_only_lower_contact_after_middle_probe_research','source_model_sha256':sha(SOURCE),'checkpoint_sha256':sha(CP/'checkpoint_1042.npz'),
                'source_checkpoint_exact_original_tolerance_not_met':True,'source_checkpoint_max_qpos_difference':5.919483221027838e-05,
                'model_force_limits_Nm':model.actuator_forcerange.tolist(),'cases':cases,'devices_opened':False,'hardware_execution_allowed':False,'physical_success':False,'elapsed_seconds':time.perf_counter()-began}
        (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({'height':height,'static_thumb_ok':geometry_ok,'error_mm':point_error,'thumb_gap_mm':thumb_gap,'dynamic_safety':dynamic['safety_passed'],'completed':dynamic['completed_goals'],'total':dynamic['total_goals'],'hold':dynamic['settled_tail'],'final_gap':end_gaps[0]}),flush=True)


if __name__=='__main__':main()
