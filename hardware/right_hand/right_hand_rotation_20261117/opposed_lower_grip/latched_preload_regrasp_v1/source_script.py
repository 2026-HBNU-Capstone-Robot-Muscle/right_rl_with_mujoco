"""Latch a real two-sided contact, preload it, then clear index distal links.

All checkpoints are obtained by free simulation of recorded bounded goals.
IK uses separate data after preload. The final independent replay starts
before preload, so no cube or hand is reset between the motion stages.
"""
from pathlib import Path
import json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.optimize import least_squares
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import TICK,ticks_to_sim
from canonical_grip_dynamics_20261105 import run
from opposed_grip_force_balance_20261116 import allocate
from audit_opposed_working_grip_20261117 import MODEL,replay,sha
from feedback_opposed_regrasp_20261117 import contacts
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate

BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
PARENT=BASE/'feedback_regrasp_normal0.6_gain1_nearpatch_basesupport_slip2_targetsurface'
REQUIRED=('mf_tip','th_tip')


def ramp(start,target):
    count=max(1,int(np.ceil(np.max(abs(target-start))/2)))
    return [np.rint(start+(target-start)*j/count).astype(int) for j in range(1,count+1)]


def main():
    out=BASE/'latched_preload_regrasp_v1';out.mkdir(exist_ok=False)
    (out/'source_script.py').write_bytes(Path(__file__).read_bytes());began=time.perf_counter()
    prior=json.loads((PARENT/'review.json').read_bytes());source=ROOT/prior['source_path']
    if sha(source)!=prior['source_sha256'] or sha(MODEL)!=prior['model_sha256']:raise RuntimeError('Source changed')
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    original=dict(np.load(source,allow_pickle=False));prefix=np.load(PARENT/'research_goals.npz',allow_pickle=False)['goals'][:21]
    prefix_result=run(m,original,prefix,1,out/'touch_state.npz',None,False,5,REQUIRED)
    if not prefix_result['safety_passed']:raise RuntimeError('Known contact prefix failed')
    z=dict(np.load(out/'touch_state.npz',allow_pickle=False));d=restore(m,z)
    hand,_,check=inspect_for(m);lo,hi,nlo,nhi=native(m);cube=m.geom('cube').id
    force,lower=contacts(m,d,cube)
    if not all(lower[n]>.2 for n in REQUIRED):raise RuntimeError('Recorded opposed contact not reproduced')
    proposed,allocation=allocate(m,d,cube,hand,nlo,nhi,lo,hi,tip_minimum=.6,required_tips=REQUIRED)
    report=dict(status='contact_latched_preload_then_index_distal_clearance',source_path=source.relative_to(ROOT).as_posix(),
        source_sha256=sha(source),model_sha256=sha(MODEL),parent_goals_sha256=sha(PARENT/'research_goals.npz'),
        script_sha256=sha(Path(__file__)),prefix_goals=21,prefix_dynamic=prefix_result,latched_lower_N=lower,
        allocation=allocation,preload_dynamic=None,index_geometry=None,audit=None,regrasp_and_hold_passed=False,
        cube_reset_after_start=False,devices_opened=False,hardware_execution_allowed=False)
    if proposed is not None:
        start=motor(d.ctrl);target=motor(proposed);preload_rows=ramp(start,target)
        preload_rows.extend(target.copy() for _ in range(60))
        preload=run(m,z,preload_rows,1,out/'preload_end.npz',None,False,5,REQUIRED)
        report['preload_dynamic']=preload
        print(json.dumps(dict(stage='preload',allocation=allocation,result={k:v for k,v in preload.items() if k not in ('trace','final_rotation_world')})),flush=True)
        if preload['safety_passed']:
            p=restore(m,np.load(out/'preload_end.npz',allow_pickle=False));C=p.geom_xpos[cube].copy();B=p.geom_xmat[cube].reshape(3,3).copy()
            base=p.qpos.copy();pd=p.ctrl-p.qpos[:16];geom=precompute(m,hand);g=m.geom('if_tip').id
            nearest=np.zeros(6);mujoco.mj_geomDistance(m,p,cube,g,.1,nearest)
            G=p.geom_xmat[g].reshape(3,3);local=G.T@(nearest[3:]-p.geom_xpos[g]);clocal=(nearest[3:]-C)@B
            clocal[1]=max(clocal[1]+.004,.007);clocal[2]=-.0325;point_target=C+B@clocal
            active=np.array([2,3]);seed=base[active]
            lower_q=np.maximum(nlo[active]+8*TICK,nlo[active]+8*TICK-pd[active])
            upper_q=np.minimum(nhi[active]-8*TICK,nhi[active]-8*TICK-pd[active])
            def measure(x,exact=False):
                p.qpos[:]=base;p.qpos[active]=x;p.qvel[:]=0;p.ctrl[:]=p.qpos[:16]+pd;mujoco.mj_forward(m,p)
                point=p.geom_xpos[g]+p.geom_xmat[g].reshape(3,3)@local
                su,sp=check(p);clear=gaps(m,p,C,B,geom,exact)
                return point,clear,max(sp.values(),default=0.),max(su.values(),default=0.)
            def residual(x):
                point,clear,selfmm,supportmm=measure(x)
                error=(point-point_target)@B*1000
                return np.r_[error[[1,2]],np.maximum(1.-np.asarray([v for _,v in clear]),0)*20,
                             (x-seed)*.1,selfmm*500000,max(0.,supportmm-.05)*1000]
            fit=least_squares(residual,np.clip(seed,lower_q+1e-9,upper_q-1e-9),bounds=(lower_q,upper_q),max_nfev=150)
            point,clear,selfmm,supportmm=measure(fit.x,True);cert=SweepCertificate(m,hand,.3);value,bad=cert.query(p,C,B)
            geometry_ok=not bad and selfmm<=1e-6 and supportmm<=.15
            report['index_geometry']=dict(passed=geometry_ok,certified_gap_mm=value,self_mm=selfmm,
                support_mm=supportmm,point_error_mm=float(np.linalg.norm(point-point_target)*1000),
                changed_motor_ids=[7,5],base_motor_targets_preserved=True)
            if geometry_ok:
                goal_sim=ticks_to_sim(target);goal_sim[active]=fit.x+pd[active];clear_target=motor(goal_sim)
                suffix=ramp(target,clear_target);hold_start=len(preload_rows)+len(suffix)
                rows=preload_rows+suffix+[clear_target.copy() for _ in range(400)]
                np.savez_compressed(out/'research_goals.npz',start=start,goals=rows,hold_start=hold_start)
                np.savez_compressed(out/'planned_clearance.npz',qpos=p.qpos,ctrl=goal_sim)
                audit=replay(m,z,rows,hold_start,out,C,B,required_roles=REQUIRED)
                report.update(audit=audit,regrasp_and_hold_passed=audit['passed'],hold_start=hold_start,goals=len(rows))
    report['elapsed_seconds']=time.perf_counter()-began
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('prefix_dynamic','preload_dynamic','audit')},ensure_ascii=True),flush=True)


if __name__=='__main__':main()
