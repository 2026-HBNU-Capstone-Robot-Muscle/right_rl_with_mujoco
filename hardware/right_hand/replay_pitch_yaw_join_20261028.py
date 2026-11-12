"""Finite-actuator/free-cube transfer diagnostic, from saved seed hypotheses.

No motor access. Cube pose is initialized once, never prescribed during replay.
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import argparse
import hashlib
import json
import time
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS, TICK
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from screen_pitch_yaw_join_20261028 import OUT as ROUTES, contact_reader
from calibrated_quarter_plan_20261028 import LOADED_PATH

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/two_axis_connection/loaded_join'
SEED=ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1/seed.json'

def run(args,env,front=-10.):
    route_path=ROUTES/(args.route+'.npz')
    route=np.load(route_path)
    start, goals=route['start'],route['goals']
    seed=json.loads(SEED.read_bytes())
    up=np.asarray(seed['up_world'])
    out=OUT/(args.route+('_seat_cycle' if args.seat_cycle else '')+('_with_yaw' if args.append_yaw else ''));out.mkdir(parents=True,exist_ok=True)
    m,manifest=build(env,out/'scene.xml',book_projection_mm=front)
    correct_model(m,start);set_cube_physics(m,.055,.1)
    m.opt.timestep=.002;m.opt.gravity[:]=-9.81*up
    positive=m.geom_solref[:,0]>0
    m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    inspect=contact_reader(m)
    cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube])
    hand={m.body('palm').id}
    for body in range(m.nbody):
        if int(m.body_parentid[body]) in hand:hand.add(body)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    lo,hi=limits();ids=np.asarray(SIM_TO_MOTOR_IDS)-1
    nlo,nhi=(lo[ids]-2048)*TICK,(hi[ids]-2048)*TICK
    gains,bias=m.actuator_gainprm.copy(),m.actuator_biasprm.copy()
    prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    yawseed=next(v for v in prior['loaded_physics'] if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10.,1.,.3))
    desired_center=np.asarray(yawseed['settled_cube_qpos'][:3])
    reports=[];began=time.perf_counter()
    for scale,friction in ([(1.,.3)] if args.nominal_only else [(s,f) for s in (.5,1.,1.5) for f in (.2,.3,.5)]):
        m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias
        m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale
        m.geom_friction[cube,0]=friction
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(start);d.ctrl[:]=d.qpos[:16]
        d.qpos[-7:]=seed['settled_cube_qpos']
        mujoco.mj_forward(m,d)
        initial_red_up=float(np.degrees(np.arccos(np.clip(d.xmat[cb].reshape(3,3)[:,0]@up,-1,1))))
        initial_cube=d.qpos[-7:].copy()
        initial_R=d.xmat[cb].reshape(3,3).copy()
        sp=sup=cp=tracking=range_excess=gap=maxgap=0.
        external=False;stopped=None;worst_pair=None;trace=[];sample=-1
        # Initial settle and final hold use exactly the same finite actuator model.
        rows=np.vstack([np.repeat(start[None,:],20,axis=0),goals,np.repeat(goals[-1][None,:],40,axis=0)])
        phases=['initial_settle']*20+['transfer']*len(goals)+['transfer_hold']*40
        if args.seat_cycle:
            yaw=np.load(LOADED_PATH);grasp=yaw['goals'][yaw['phases']=='grasp']
            release=np.vstack([grasp[-2::-1],yaw['start'][None,:]])
            pieces=[('seat_grasp',grasp),('seat_grasp_hold',np.repeat(grasp[-1][None,:],40,axis=0)),
                    ('seat_open',release),('seat_open_hold',np.repeat(yaw['start'][None,:],40,axis=0))]
            rows=np.vstack([rows,*[q for _,q in pieces]])
            for p,q in pieces:phases.extend([p]*len(q))
        if args.append_yaw:
            yaw=np.load(LOADED_PATH)
            np.testing.assert_array_equal(goals[-1],yaw['start'])
            rows=np.vstack([rows,yaw['goals']])
            phases.extend(str(p) for p in yaw['phases'])
        assert np.max(abs(np.diff(np.vstack([start,rows]),axis=0)))<=2
        checkpoints={}
        steps=0;contact=False
        for sample,q in enumerate(rows):
            d.ctrl[:]=ticks_to_sim(q)
            target=round((sample+1)*.075/.002)
            for step in range(target-steps):
                mujoco.mj_step(m,d)
                a,b,pair,_=inspect(d);sp=max(sp,a);sup=max(sup,b)
                if a>1e-6:worst_pair=pair
                tracking=max(tracking,float(max(abs(d.qpos[:16]-d.ctrl))))
                range_excess=max(range_excess,float(max(0.,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
                contact=False
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    if cube not in (a,b):continue
                    other=b if a==cube else a
                    if int(m.geom_bodyid[other]) in hand:
                        cp=max(cp,max(0.,-float(c.dist)*1000));contact|=c.dist<.0004
                    elif other in support and c.dist<.0004:external=True
                gap=0. if contact else gap+.002;maxgap=max(maxgap,gap)
                if not np.isfinite(d.qpos).all():stopped='nonfinite_state'
                elif sp>1e-6:stopped='hand_self_contact'
                elif sup>.15:stopped='hand_support_contact'
                elif cp>1:stopped='cube_overlap'
                elif tracking>.18:stopped='tracking'
                elif range_excess>1e-6:stopped='native_range'
                elif external:stopped='cube_external_contact'
                elif maxgap>.25:stopped='lost_hand_contact'
                if stopped:break
            steps=target
            if sample%2==0:trace.append(d.qpos.copy())
            phase=phases[sample]
            if sample==len(rows)-1 or phases[sample+1]!=phase:
                mujoco.mj_forward(m,d)
                B=d.xmat[cb].reshape(3,3)
                quat=Rotation.from_matrix(B@initial_R.T).as_quat()
                checkpoints[phase]=dict(cube_qpos=d.qpos[-7:].tolist(),
                    twist_about_world_up_deg=float(np.degrees(2*np.arctan2(quat[:3]@up,quat[3]))),
                    red_up_deg=float(np.degrees(np.arccos(np.clip(B[:,0]@up,-1,1)))))
            if stopped:break
        mujoco.mj_forward(m,d)
        R=d.xmat[cb].reshape(3,3)
        reorientation=float(np.degrees(Rotation.from_matrix(R@initial_R.T).magnitude()))
        seat_error=float(np.linalg.norm(d.qpos[-7:-4]-desired_center)*1000)
        red_up=float(np.degrees(np.arccos(np.clip(R[:,0]@up,-1,1))))
        checks=dict(motion_health_passed=stopped is None,final_hand_contact=bool(contact),
                    final_red_up_within15deg=red_up<15)
        if not args.append_yaw:checks['seat_within8mm_of_existing_yaw_start']=seat_error<8
        else:
            yaw_hold=checkpoints.get('yaw_hold',{})
            checks['substantial_horizontal_turn']=abs(yaw_hold.get('twist_about_world_up_deg',0))>=45
        passed=all(checks.values())
        name=f'trace_s{scale:g}_mu{friction:g}.npz'
        np.savez_compressed(out/name,qpos=np.asarray(trace),sample_period_s=.15,end_qpos=d.qpos.copy())
        row=dict(book_front_mm=front,scale=scale,friction=friction,passed=passed,stop=stopped,last_sample=sample,
                 requested_samples=len(rows),last_phase=phases[sample],acceptance_checks=checks,
                 phase_checkpoints=checkpoints,initial_red_up_deg=initial_red_up,red_up_deg=red_up,
                 cube_reorientation_deg=reorientation,seat_error_to_yaw_initial_mm=seat_error,
                 cube_translation_mm=float(np.linalg.norm(d.qpos[-7:-4]-initial_cube[:3])*1000),
                 max_self_mm=sp,max_support_mm=sup,max_cube_mm=cp,max_tracking_rad=tracking,
                 native_range_excess_rad=range_excess,max_no_hand_contact_seconds=maxgap,
                 external_cube_contact=external,worst_self_pair=worst_pair,trace=name,
                 end_cube_qpos=d.qpos[-7:].tolist())
        reports.append(row)
        print(json.dumps(dict(route=args.route,**{k:row[k] for k in (
            'scale','friction','passed','stop','last_phase','red_up_deg',
            'seat_error_to_yaw_initial_mm','max_self_mm','max_cube_mm')})),flush=True)
        report=dict(status='finite_actuator_free_cube_pitch_to_yaw_transfer_diagnostic',
                    cases=reports,all_passed=all(v['passed'] for v in reports),
                    route_source_sha256=hashlib.sha256(route_path.read_bytes()).hexdigest(),
                    initial_cube_seed_sha256=hashlib.sha256(SEED.read_bytes()).hexdigest(),
                    appended_yaw_source_sha256=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest() if args.append_yaw else None,
                    existing_grasp_open_seating_cycle_added=bool(args.seat_cycle),
                    initial_cube_seed_is_uncertified_model_hypothesis=True,
                    finite_actuators=True,hand_qpos_prescribed_during_replay=False,
                    cube_qpos_prescribed_during_replay=False,model=manifest,
                    real_devices_opened=False,hardware_execution_allowed=False,
                    elapsed_seconds=time.perf_counter()-began)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    return report

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--route',default='together')
    parser.add_argument('--nominal-only',action='store_true')
    parser.add_argument('--append-yaw',action='store_true')
    parser.add_argument('--seat-cycle',action='store_true')
    parser.add_argument('--batch',action='store_true')
    args=parser.parse_args()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    if not args.batch:return run(args,env)
    assert args.nominal_only and not args.append_yaw
    reports=[]
    for path in sorted(ROUTES.glob('*.npz')):
        r=run(SimpleNamespace(route=path.stem,nominal_only=True,append_yaw=False,seat_cycle=False),env)
        reports.append(dict(route=path.stem,**r['cases'][0]))
    report=dict(status='loaded_transfer_order_comparison',cases=reports,
                promising=sum(r['passed'] for r in reports),
                initial_cube_seed_is_uncertified_model_hypothesis=True,
                hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/'route_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('BATCH SUMMARY',len(reports),report['promising'],flush=True)

if __name__=='__main__':main()
