"""Contact-gated, pose-following regrasp of the actual saved free cube.

Motion tasks for missing fingertips are geometric approach requests, never
contact forces injected into MuJoCo. Force allocation uses actual engine
contacts only. Perfect model feedback is not a real SAC deployment.
"""
from pathlib import Path
import argparse,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from research_quasistatic_wrench_path_20261105 import allocate_native,hybrid_increment
from probe_opposed_grip_turn_20261117 import FingerClearPlanner
from audit_opposed_working_grip_20261117 import MODEL,ROLES,replay,sha
from hand_motor_mapping import ticks_to_sim
from upper_layer_separation_certificate_20261105 import SweepCertificate
from upper_layer_cylinder_clearance_20261105 import precompute

BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
SOURCE=BASE/'next_turn5p0_palmclear_margin8_fingerclear_allpalms/end_state.npz'
REFERENCE=BASE/'level_triple_normal1p2_settle1_settled_frame_active_hold400_heightlock0p0/complete_hold_audit/audit_end_state.npz'
REQUIRED=('mf_tip','th_tip')


class ApproachPlanner(FingerClearPlanner):
    def increment_bounds(self):
        lower,upper=super().increment_bounds()
        if getattr(self,'freeze_index',False):lower[4:8]=0.;upper[4:8]=0.
        if getattr(self,'freeze_index_base',False):lower[[5,7]]=0.;upper[[5,7]]=0.
        if getattr(self,'freeze_middle',False):lower[8:12]=0.;upper[8:12]=0.
        return lower,upper


def contacts(m,d,cube):
    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
    forces=dict.fromkeys(ROLES,0.);lower=dict.fromkeys(ROLES,0.)
    for ci,c in enumerate(d.contact):
        aa,bb=int(c.geom1),int(c.geom2)
        if cube not in (aa,bb):continue
        other=bb if aa==cube else aa;name=m.geom(other).name
        if name not in forces or c.dist>=.0004:continue
        ff=np.zeros(6);mujoco.mj_contactForce(m,d,ci,ff);forces[name]+=float(ff[0])
        normal=(c.frame[:3]*(1 if aa==cube else -1))@B
        side=normal[2]>.5 if name=='th_tip' else normal[2]<-.5
        if side and (c.pos-C)@(-B[:,1])<55/6000:lower[name]+=float(ff[0])
    return forces,lower


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--normal',type=float,choices=(.6,1.),default=.6)
    ap.add_argument('--gain',type=float,choices=(.5,1.),default=1.)
    ap.add_argument('--surface-point',action='store_true')
    ap.add_argument('--free-index-tip',action='store_true')
    ap.add_argument('--near-patch',action='store_true')
    a=ap.parse_args()
    tag=f'feedback_regrasp_normal{a.normal:g}_gain{a.gain:g}'
    if a.surface_point:tag+='_surface'
    if a.free_index_tip:tag+='_indexbase'
    if a.near_patch:tag+='_nearpatch'
    out=BASE/tag;out.mkdir(exist_ok=False);(out/'source_script.py').write_bytes(Path(__file__).read_bytes())
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    z=dict(np.load(SOURCE,allow_pickle=False));d=restore(m,z);ref=restore(m,np.load(REFERENCE,allow_pickle=False))
    hand,supports,check=inspect_for(m);lo,hi,nlo,nhi=native(m);cube=m.geom('cube').id;body=m.body('cube').id
    vertices={g:v for g,v,e in precompute(m,hand)}
    C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
    Cr=ref.geom_xpos[cube].copy();Br=ref.geom_xmat[cube].reshape(3,3).copy()
    start=motor(d.ctrl);planner=ApproachPlanner(start,np.minimum(lo+8,start),np.maximum(hi-8,start),m,d)
    planner.pairs=[(m.geom(g).name,'th_tip') for g in range(m.ngeom) if (m.geom(g).name or '').startswith('palm_collision_')]
    planner.pairs.append(('if_md_collision_5','mf_px_collision'))
    frames={};points={};targets={}
    for name in ROLES:
        g=m.geom(name).id;c=min((c for c in ref.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist)
        G=ref.geom_xmat[g].reshape(3,3);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
        points[name]=G.T@(c.pos-ref.geom_xpos[g]);clocal=Br.T@(c.pos-Cr)
        if name=='mf_tip':clocal[0]=.020
        if name=='if_tip':clocal[1]+=.004;clocal[2]=-.0325
        else:clocal[2]=.0275 if name=='th_tip' else -.0275
        if a.near_patch and name in REQUIRED:clocal[0]=.023;clocal[1]=.010
        targets[name]=clocal;frames[name]=(G.T@normal,Br.T@normal)
    properties={key:getattr(m,key).copy() for key in ('actuator_gainprm','actuator_biasprm','actuator_forcerange','jnt_range','geom_size','geom_contype','geom_conaffinity')}
    phase='secure_thumb' if a.near_patch else 'secure_opposed';stable=held=0;hold_start=None;stop=None;rows=[];phases=[];trace=[];poses=[];loss=0.
    cert=SweepCertificate(m,hand,.3);cert_min=float('inf');cert_steps=0;began=time.perf_counter()
    refs={'mf_tip':a.normal,'th_tip':a.normal}
    for index in range(1000):
        C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
        v=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,body,v,0)
        F=-m.body_mass[body]*m.opt.gravity+30*(C0-C)-1.5*v[3:]
        T=.04*Rotation.from_matrix(B0@B.T).as_rotvec()-.002*v[:3]
        proposed,allocation=allocate_native(m,d,cube,C,F,T,refs.copy(),hand,True,(nlo,nhi),REQUIRED,normal_reference_weight=.2)
        if proposed is None:stop=dict(reason='force_allocation',details=allocation);break
        forces,lower_now=contacts(m,d,cube)
        tasks=[t for t in allocation['contact_tasks'] if m.geom(t['geom_id']).name not in ROLES]
        moving=REQUIRED if phase=='secure_opposed' and not a.free_index_tip else ROLES
        if phase=='secure_thumb':moving=('th_tip',)
        for name in moving:
            g=m.geom(name).id;point=d.geom_xpos[g]+d.geom_xmat[g].reshape(3,3)@points[name]
            normal=B@frames[name][1]
            if a.surface_point:
                world=vertices[g]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
                projection=(world-C)@normal
                weight=np.exp(-(projection-projection.min())/.00002)
                point=np.average(world,axis=0,weights=weight)
            gap=None
            if a.near_patch:
                closest=np.zeros(6);gap=float(mujoco.mj_geomDistance(m,d,cube,g,.1,closest));point=closest[3:].copy()
            error=C+B@targets[name]-point
            if name in REQUIRED and (lower_now[name]>.02 if a.near_patch else abs(float(error@normal))<.0003):
                error-=normal*float(error@normal)
            length=np.linalg.norm(error)
            if length>.002:error*=.002/length
            wanted=refs.get(name,0.)
            if a.near_patch and name in REQUIRED:
                clocal=(point-C)@B
                if gap>.0004 or abs(clocal[0])>.026:wanted=lower_now[name]
            tasks.append(dict(geom_id=g,point=point.tolist(),outward_normal=normal.tolist(),
                actual_normal_N=lower_now[name] if a.near_patch else forces[name],predicted_normal_N=wanted,
                recover_tip_displacement=(error*a.gain).tolist(),geometric_approach_task=True))
        allocation['contact_tasks']=tasks
        planner.freeze_index=phase=='secure_opposed' and not a.free_index_tip
        planner.freeze_index_base=phase=='secure_opposed' and a.free_index_tip
        if a.near_patch:
            planner.freeze_index=phase=='secure_thumb';planner.freeze_index_base=phase!='secure_thumb'
            planner.freeze_middle=phase=='secure_thumb'
        try:
            goal=hybrid_increment(m,d,allocation,planner,C,C0,B,B0,maximum_tracking_rad=.12,
                contact_normal_frames=None if a.surface_point or a.near_patch else {name:frames[name] for name in moving},
                tip_memory_names=REQUIRED,recovery_normal_targets=refs)
        except ValueError as exc:
            if 'projection_infeasible' not in str(exc):raise
            stop=dict(reason=str(exc),goal=index+1);break
        rows.append(goal.copy());phases.append(phase);d.ctrl[:]=ticks_to_sim(goal)
        for sub in range(75):
            mujoco.mj_step(m,d);su,sp=check(d)
            values=dict(self_mm=max(sp.values(),default=0.),support_mm=max(su.values(),default=0.),
                cube_mm=0.,tracking_rad=float(max(abs(d.qpos[:16]-d.ctrl))),
                range_rad=float(max(0.,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
            inhand=external=False
            for c in d.contact:
                aa,bb=int(c.geom1),int(c.geom2)
                if cube not in (aa,bb):continue
                other=bb if aa==cube else aa
                if int(m.geom_bodyid[other]) in hand:
                    inhand|=c.dist<.0004;values['cube_mm']=max(values['cube_mm'],max(0.,-float(c.dist)*1000))
                else:external|=other in supports and c.dist<.0004
            loss=0. if inhand else loss+.001
            failed=[key for key,bound in [('self_mm',1e-6),('support_mm',.15),('cube_mm',1.),('tracking_rad',.18),('range_rad',1e-6)] if values[key]>bound]
            if external:failed.append('external_cube_support')
            if loss>.25:failed.append('hand_contact_loss')
            if np.linalg.norm(d.qpos[-7:-4]-C0)>.08:failed.append('translation_bound')
            if not np.isfinite(np.r_[d.qpos,d.qvel]).all():failed.append('nonfinite_state')
            if phase=='hold':
                clearance,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3));cert_steps+=1;cert_min=min(cert_min,clearance)
                if bad:failed.append('upper_sweep_clearance')
            if failed:stop=dict(reason='original_guard_or_hold_space',goal=index+1,substep=sub+1,failed=failed,values=values);break
        C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();forces,lower=contacts(m,d,cube)
        angle=float(np.degrees(np.linalg.norm(Rotation.from_matrix(B@B0.T).as_rotvec())));shift=float(np.linalg.norm(C-C0)*1000)
        loaded=all(lower[name]>.2 for name in REQUIRED)
        if phase=='secure_thumb':
            stable=stable+1 if lower['th_tip']>.2 else 0
            if stable>=10:phase='secure_opposed';stable=0
        elif phase=='secure_opposed':
            stable=stable+1 if loaded else 0
            if stable>=10:phase='clear_index';stable=0
        elif phase=='clear_index':
            clearance,bad=cert.query(d,C,B)
            ready=loaded and not bad and clearance>.8 and angle<3 and shift<2
            stable=stable+1 if ready else 0
            if stable>=10:phase='hold';hold_start=len(rows);stable=0
        else:held+=1
        if index%20==0 or stop or held>=400:
            row=dict(goal=index+1,phase=phase,angle_error_deg=angle,center_shift_mm=shift,normal_N=forces,opposed_lower_N=lower)
            trace.append(row);print(json.dumps(row),flush=True)
        if index%2==0 or stop:poses.append(d.qpos.copy())
        if stop or held>=400:break
        if index>=599 and hold_start is None:stop=dict(reason='bounded_regrasp_not_converged',phase=phase);break
    for key,value in properties.items():
        if not np.array_equal(getattr(m,key),value):raise RuntimeError('Model properties changed')
    np.savez_compressed(out/'research_goals.npz',start=start,goals=rows,phases=phases)
    np.savez_compressed(out/'trajectory.npz',qpos=poses)
    np.savez_compressed(out/'end_state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,
        time=d.time,up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    audit=None
    if stop is None and held==400:audit=replay(m,z,rows,hold_start,out,C0,B0,required_roles=REQUIRED)
    report=dict(status='contact_gated_pose_following_regrasp',parameters=vars(a),source_sha256=sha(SOURCE),
        source_path=SOURCE.relative_to(ROOT).as_posix(),reference_sha256=sha(REFERENCE),model_sha256=sha(MODEL),
        script_sha256=sha(Path(__file__)),stop=stop,phase=phase,goals=len(rows),hold_start=hold_start,
        held_goals=held,audit=audit,regrasp_and_hold_passed=bool(audit and audit['passed']),trace=trace,
        held_space_certificate=dict(steps=cert_steps,minimum_gap_mm=cert_min if cert_steps else None),
        perfect_simulated_feedback=True,geometric_tasks_are_not_injected_contact_forces=True,
        cube_reset_after_start=False,model_properties_unchanged=True,devices_opened=False,
        new_SAC_training=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('trace','audit')},ensure_ascii=True),flush=True)


if __name__=='__main__':main()
