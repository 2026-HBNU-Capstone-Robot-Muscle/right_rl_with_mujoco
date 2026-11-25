"""Hold the achieved valid grip with contact preload and material anchors.

No further index withdrawal or prescribed middle-finger slide is requested.
The independent check replays the whole original stream plus this suffix.
"""
from pathlib import Path
import argparse,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from audit_opposed_working_grip_20261117 import MODEL,replay,sha
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from research_quasistatic_wrench_path_20261105 import allocate_native
from probe_opposed_grip_turn_20261117 import FingerClearPlanner
from opposed_force_slide_control_20261117 import increment
from feedback_opposed_regrasp_20261117 import contacts
from hand_motor_mapping import ticks_to_sim
from upper_layer_separation_certificate_20261105 import SweepCertificate

BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
PARENT=BASE/'core_support_chain_10s'
REQUIRED=('mf_tip','th_tip')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--normal',type=float,choices=(.6,.7,1.),default=1.)
    args=ap.parse_args()
    prior=json.loads((PARENT/'review.json').read_bytes())
    if not prior['working_grip_passed']:raise RuntimeError('Existing complete working grip required')
    source=PARENT/'audit_end_state.npz';out=PARENT/('active_hold30'+('_normal'+str(args.normal).replace('.','p') if args.normal!=1 else ''));out.mkdir(exist_ok=False)
    (out/'source_script.py').write_bytes(Path(__file__).read_bytes())
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    z=dict(np.load(source,allow_pickle=False));d=restore(m,z)
    hand,support,check=inspect_for(m);lo,hi,nlo,nhi=native(m);cube=m.geom('cube').id;body=m.body('cube').id
    C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();anchors={}
    for name in REQUIRED:
        g=m.geom(name).id;c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist)
        anchors[name]=(d.geom_xmat[g].reshape(3,3).T@(c.pos-d.geom_xpos[g]),B0.T@(c.pos-C0),
                       B0.T@(c.frame[:3]*(1 if int(c.geom1)==cube else -1)))
    start=motor(d.ctrl);planner=FingerClearPlanner(start,np.minimum(lo+2,start),np.maximum(hi-2,start),m,d)
    planner.pairs=[(m.geom(g).name,'th_tip') for g in range(m.ngeom) if (m.geom(g).name or '').startswith('palm_collision_')]
    planner.pairs.append(('if_md_collision_5','mf_px_collision'));refs={name:args.normal for name in REQUIRED}
    cert=SweepCertificate(m,hand,.3);rows=[];trace=[];stop=None;loss=0.;began=time.perf_counter()
    for index in range(266):
        C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();v=np.zeros(6)
        mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,body,v,0)
        F=-m.body_mass[body]*m.opt.gravity+30*(C0-C)-1.5*v[3:]
        T=.04*Rotation.from_matrix(B0@B.T).as_rotvec()-.002*v[:3]
        proposed,allocation=allocate_native(m,d,cube,C,F,T,refs.copy(),hand,True,(nlo,nhi),REQUIRED,normal_reference_weight=.2)
        if proposed is None:stop=dict(reason='allocation',details=allocation);break
        forces,lower=contacts(m,d,cube);tasks=[]
        for name,(local,clocal,cn) in anchors.items():
            g=m.geom(name).id;point=d.geom_xpos[g]+d.geom_xmat[g].reshape(3,3)@local
            error=C+B@clocal-point;length=np.linalg.norm(error)
            if length>.002:error*=.002/length
            tasks.append(dict(geom_id=g,point=point.tolist(),outward_normal=(B@cn).tolist(),recover_tip_displacement=error.tolist()))
        try:goal=increment(m,d,allocation,tasks,planner,C,C0,B,B0,refs,lower)
        except ValueError as exc:
            if 'projection_infeasible' not in str(exc):raise
            stop=dict(reason=str(exc),goal=index+1);break
        rows.append(goal.copy());d.ctrl[:]=ticks_to_sim(goal)
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
                else:external|=other in support and c.dist<.0004
            loss=0. if inhand else loss+.001
            failed=[key for key,bound in [('self_mm',1e-6),('support_mm',.15),('cube_mm',1.),('tracking_rad',.18),('range_rad',1e-6)] if values[key]>bound]
            if external:failed.append('external_cube_support')
            if loss>.25:failed.append('hand_contact_loss')
            if np.linalg.norm(d.qpos[-7:-4]-C0)>.08:failed.append('translation_bound')
            if not np.isfinite(np.r_[d.qpos,d.qvel]).all():failed.append('nonfinite_state')
            gap,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3))
            if bad:failed.append('upper_sweep_clearance')
            if failed:stop=dict(reason='unchanged_guard',goal=index+1,substep=sub+1,failed=failed,values=values);break
        if index%40==0 or stop:
            forces,lower=contacts(m,d,cube);trace.append(dict(goal=index+1,lower_N=lower,space_gap_mm=gap))
            print(json.dumps(trace[-1]),flush=True)
        if stop:break
    np.savez_compressed(out/'suffix.npz',start=start,goals=rows)
    audit=None
    if stop is None and len(rows)==266:
        old=np.load(PARENT/'research_goals.npz',allow_pickle=False);full=np.vstack([old['goals'],rows])
        initial=ROOT/prior['source_path']
        target=json.loads((BASE/'feedback_regrasp_normal1_gain1_nearpatch_basesupport_slip16_targetsurface_budget300_forceslide_margin2_paired_early141_latched_seated_freeifx/review.json').read_bytes())
        audit=replay(m,dict(np.load(initial,allow_pickle=False)),full,prior['hold_start'],out,
            target['hold_target_center_world_m'],target['hold_target_rotation_world'],required_roles=REQUIRED)
        np.savez_compressed(out/'research_goals.npz',start=old['start'],goals=full,hold_start=prior['hold_start'])
    result=dict(status='active_achieved_grip_hold_with_uninterrupted_replay',passed=bool(audit and audit['passed']),
        contact_normal_target_N=args.normal,
        audit=audit,stop=stop,suffix_goals=len(rows),source_sha256=sha(source),model_sha256=sha(MODEL),
        parent_review_sha256=sha(PARENT/'review.json'),script_sha256=sha(Path(__file__)),trace=trace,
        cube_or_hand_reset_between_segments=False,full_quarter_turn_cycle_verified=False,
        new_SAC_training=False,physical_grasp_verified=False,devices_opened=False,hardware_execution_allowed=False,
        elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('audit','trace')},ensure_ascii=True),flush=True)


if __name__=='__main__':main()
