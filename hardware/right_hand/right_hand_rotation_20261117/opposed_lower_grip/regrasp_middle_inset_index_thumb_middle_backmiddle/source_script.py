"""Solve a lower-side regrasp at the reached cube pose, then replay it free.

IK uses a separate planning data object. The dynamic continuation starts at
the actual saved turn endpoint and never sets cube/hand state again. Offline
perfect model research only; no robot transport or hardware-ready output.
"""
from pathlib import Path
import argparse, json, sys, time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.optimize import least_squares
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS,ticks_to_sim
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate
from opposed_grip_force_balance_20261116 import allocate
from audit_opposed_working_grip_20261117 import MODEL,ROLES,replay,sha

BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
PARENT=BASE/'next_turn5p0_palmclear_margin8_fingerclear_allpalms'
REFERENCE=BASE/'level_triple_normal1p2_settle1_settled_frame_active_hold400_heightlock0p0/complete_hold_audit/audit_end_state.npz'


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--target-mode',choices=('reference','middle_inset'),default='reference')
    ap.add_argument('--order',choices=('together','thumb_middle_index','index_thumb_middle'),default='together')
    ap.add_argument('--back',choices=('both','middle'),default='both')
    a=ap.parse_args()
    source=PARENT/'end_state.npz';out=BASE/(f'regrasp_{a.target_mode}_{a.order}'+('_backmiddle' if a.back=='middle' else ''))
    out.mkdir(exist_ok=False);(out/'source_script.py').write_bytes(Path(__file__).read_bytes())
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    z=dict(np.load(source,allow_pickle=False));d=restore(m,z);ref=restore(m,np.load(REFERENCE,allow_pickle=False))
    hand,_,check=inspect_for(m);geometry=precompute(m,hand);cube=m.geom('cube').id
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
    Cr=ref.geom_xpos[cube].copy();Br=ref.geom_xmat[cube].reshape(3,3).copy()
    base=d.qpos.copy();active=np.r_[np.arange(8),np.arange(12,16)]
    lo,hi,nlo,nhi=native(m);pd=d.ctrl-d.qpos[:16]
    lower=np.maximum(nlo[active]+8*TICK,nlo[active]+8*TICK-pd[active])
    upper=np.minimum(nhi[active]-8*TICK,nhi[active]-8*TICK-pd[active])
    specs=[]
    required=ROLES if a.back=='both' else ('mf_tip','th_tip')
    for name in ROLES:
        g=m.geom(name).id;contacts=[]
        for ci,c in enumerate(ref.contact):
            if {int(c.geom1),int(c.geom2)}!={cube,g}:continue
            force=np.zeros(6);mujoco.mj_contactForce(m,ref,ci,force);contacts.append((float(force[0]),c))
        c=max(contacts,key=lambda p:p[0])[1];G=ref.geom_xmat[g].reshape(3,3)
        normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
        local=G.T@(c.pos-ref.geom_xpos[g]);clocal=Br.T@(c.pos-Cr)
        if a.target_mode=='middle_inset' and name=='mf_tip':clocal[0]=.020
        # Lower the narrow-clearance index contact slightly, not the cube pose.
        if name=='if_tip':clocal[1]+=.001
        clocal[2]=(.02748 if name=='th_tip' else -.02748)
        if a.back=='middle' and name=='if_tip':
            clocal[1]+=.003;clocal[2]=-.0325
        specs.append((name,g,local,G.T@normal,C+B@clocal,B@(Br.T@normal),clocal))
    def measure(x,exact=False):
        d.qpos[:]=base;d.qpos[active]=x;d.qvel[:]=0;d.ctrl[:]=d.qpos[:16]+pd
        mujoco.mj_forward(m,d);errors=[];normals=[];details=[]
        for name,g,local,ln,target,normal,clocal in specs:
            G=d.geom_xmat[g].reshape(3,3);point=d.geom_xpos[g]+G@local
            errors.extend((point-target)*1000);normals.extend((G@ln-normal)*10)
            details.append(dict(name=name,point_error_mm=float(np.linalg.norm(point-target)*1000)))
        su,sp=check(d);clear=gaps(m,d,C,B,geometry,exact)
        pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return np.asarray(errors),np.asarray(normals),details,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen
    seed=np.clip(base[active],lower+1e-9,upper-1e-9)
    def residual(x):
        err,normal,_,clear,selfmm,supportmm,pen=measure(x)
        return np.r_[err*3,normal,np.maximum(.8-np.asarray([v for _,v in clear]),0)*10,
                     (x-seed)*.1,selfmm*500000,max(0.,supportmm-.05)*1000,max(0.,pen-.2)*1000]
    began=time.perf_counter();fit=least_squares(residual,seed,bounds=(lower,upper),max_nfev=200,
        ftol=1e-9,xtol=1e-9,gtol=1e-9)
    err,normal,details,clear,selfmm,supportmm,pen=measure(fit.x,True)
    certificate=SweepCertificate(m,hand,.3);certgap,bad=certificate.query(d,C,B)
    present=set();contacts=[]
    for c in d.contact:
        aa,bb=int(c.geom1),int(c.geom2)
        if cube not in (aa,bb):continue
        other=bb if aa==cube else aa;name=m.geom(other).name
        contacts.append(dict(name=name,gap_mm=float(c.dist*1000)))
        if c.dist<.0004 and (c.pos-C)@(-B[:,1])<55/6000:present.add(name)
    geometry_ok=bool(set(required)<=present and not bad and selfmm<=1e-6 and supportmm<=.15 and pen<=1)
    proposed=allocation=audit=None;rows=[];hold_start=None
    if geometry_ok:
        proposed,allocation=allocate(m,d,cube,hand,nlo,nhi,lo,hi,required_tips=required)
    if proposed is not None:
        start=motor(z['ctrl']);target=motor(proposed);previous=start.copy()
        groups=([np.arange(16)] if a.order=='together' else
                [np.arange(4),np.arange(8,12),np.arange(4,8)] if a.order=='thumb_middle_index' else
                [np.arange(4,8),np.arange(4),np.arange(8,12)])
        for group in groups:
            endpoint=previous.copy();endpoint[group]=target[group]
            count=max(1,int(np.ceil(max(abs(endpoint-previous))/2)))
            rows.extend(np.rint(previous+(endpoint-previous)*j/count).astype(int) for j in range(1,count+1))
            previous=endpoint
        hold_start=len(rows);rows.extend(target.copy() for _ in range(400))
        np.savez_compressed(out/'research_goals.npz',start=start,goals=rows,hold_start=hold_start)
        np.savez_compressed(out/'planned_geometry.npz',qpos=d.qpos,ctrl=proposed)
        audit=replay(m,z,rows,hold_start,out,C,B,required_roles=required)
    report=dict(status='postturn_fixed_cube_planning_free_regrasp',parameters=vars(a),source_path=source.relative_to(ROOT).as_posix(),
        source_sha256=sha(source),reference_sha256=sha(REFERENCE),model_sha256=sha(MODEL),script_sha256=sha(Path(__file__)),
        geometry_passed=geometry_ok,point_errors=details,normal_residual_norm=float(np.linalg.norm(normal)),
        required_lower_contacts=list(required),
        self_mm=selfmm,support_mm=supportmm,cube_mm=pen,exact_support_plane_gap_mm=certgap,
        radial_screen_min_mm=min(v for _,v in clear),actual_engine_contacts=contacts,
        allocation=allocation,hold_start=hold_start,goals=len(rows),audit=audit,
        regrasp_and_hold_passed=bool(audit and audit['passed']),static_cube_fixed_only_in_separate_planning_data=True,
        dynamic_cube_reset_after_start=False,devices_opened=False,hardware_execution_allowed=False,
        full_quarter_turn_cycle_verified=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('actual_engine_contacts','audit')},ensure_ascii=True),flush=True)
    if audit:print(json.dumps({k:v for k,v in audit.items() if k not in ('timeline','certificate')},ensure_ascii=True),flush=True)


if __name__=='__main__':main()
