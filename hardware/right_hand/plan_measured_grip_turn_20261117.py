"""Bounded IK diagnostic from the measured grip's fitted model state, offline only."""
from pathlib import Path
import argparse,hashlib,json,time
import mujoco,numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim,sim_to_ticks,SIM_TO_MOTOR_IDS,TICK
from research_rotation_body_hold_20261104 import restore,native,inspect_for

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261117/user_measured_grip'
MODEL=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
INITIAL=BASE/'model_hold_probe/end.npz'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rolling-contacts',action='store_true')
    args=parser.parse_args()
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    d=restore(m,dict(np.load(INITIAL,allow_pickle=False)))
    cube=m.geom('cube').id;palm=m.body('palm').id
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
    axis=d.xmat[palm].reshape(3,3)[:,2].copy()
    hand,support,check=inspect_for(m)
    contacts=[]
    for wanted in ('if_tip','th_px_collision_1'):
        gid=m.geom(wanted).id;found=[]
        for ci,c in enumerate(d.contact):
            if {int(c.geom1),int(c.geom2)}!={cube,gid}:continue
            f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f)
            found.append((f[0],c))
        if not found:raise RuntimeError('Missing settled contact '+wanted)
        _,c=max(found,key=lambda row:row[0])
        body=int(m.geom_bodyid[gid]);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
        G=d.xmat[body].reshape(3,3)
        contacts.append(dict(name=wanted,body=body,body_point=G.T@(c.pos-d.xpos[body]),
                             cube_point=B.T@(c.pos-C),body_normal=G.T@normal,cube_normal=B.T@normal))
    q0=d.qpos[:16].copy();preload=d.ctrl-q0
    active=np.r_[np.arange(4),np.arange(12,16)]
    lo,hi,nlo,nhi=native(m)
    lower=nlo[active]+4*TICK-preload[active];upper=nhi[active]-4*TICK-preload[active]
    previous=q0[active].copy();records=[];goals=[];began=time.perf_counter();stopped=None
    for degrees in range(1,91):
        if time.perf_counter()-began>45:stopped='bounded45second budget';break
        target_B=Rotation.from_rotvec(axis*np.radians(degrees)).as_matrix()@B
        target_q=Rotation.from_matrix(target_B).as_quat()
        def evaluate(x):
            d.qpos[:16]=q0;d.qpos[active]=x
            d.qpos[-7:-4]=C;d.qpos[-4:]=np.r_[target_q[3],target_q[:3]]
            mujoco.mj_fwdPosition(m,d)
            positions=[];normals=[]
            for c in contacts:
                G=d.xmat[c['body']].reshape(3,3)
                positions.extend((d.xpos[c['body']]+G@c['body_point']-C-target_B@c['cube_point'])*1000)
                normals.extend((G@c['body_normal']-target_B@c['cube_normal'])*3)
            self_depth,support_depth=check(d)
            cube_pen=max((max(0,-c.dist*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0)
            return np.array(positions),np.array(normals),max(self_depth.values(),default=0),max(support_depth.values(),default=0),cube_pen
        def residual(x):
            positions,normals,selfmm,supportmm,cubemm=evaluate(x)
            return np.r_[positions*3,normals,(x-previous)*.15,
                         max(0,selfmm-.5)*30,max(0,supportmm-.2)*30,max(0,cubemm-.5)*30]
        seed=np.clip(previous,lower+1e-8,upper-1e-8)
        fit=least_squares(residual,seed,bounds=(lower,upper),max_nfev=90,
                          diff_step=1e-4,ftol=1e-7,xtol=1e-7,gtol=1e-7)
        positions,normals,selfmm,supportmm,cubemm=evaluate(fit.x)
        q=q0.copy();q[active]=fit.x;goal=sim_to_ticks(q+preload)
        error=float(np.max(np.linalg.norm(positions.reshape(-1,3),axis=1)))
        row=dict(requested_turn_degrees=degrees,contact_point_error_mm=error,
                 self_penetration_mm=float(selfmm),support_penetration_mm=float(supportmm),
                 cube_penetration_mm=float(cubemm),goal_ticks=goal.tolist())
        records.append(row)
        if error>1 or selfmm>.6 or supportmm>.3 or cubemm>.6:
            stopped='IK/contact/geometry diagnostic limit';break
        goals.append(goal);previous=fit.x.copy()
        if args.rolling_contacts:
            for contact in contacts:
                gid=m.geom(contact['name']).id
                matches=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,gid}]
                if not matches:
                    segment=np.zeros(6)
                    gap=float(mujoco.mj_geomDistance(m,d,gid,cube,.02,segment))
                    if gap>.0002:
                        stopped='rolling surface gap exceeds0.2mm';break
                    if gap>1e-9 and np.linalg.norm(segment[:3]-segment[3:])>1e-9:
                        G=d.xmat[contact['body']].reshape(3,3)
                        normal=(segment[:3]-segment[3:])/np.linalg.norm(segment[:3]-segment[3:])
                        contact.update(body_point=G.T@(segment[:3]-d.xpos[contact['body']]),
                            cube_point=target_B.T@(segment[3:]-C),body_normal=G.T@normal,cube_normal=target_B.T@normal)
                    continue
                c=min(matches,key=lambda c:c.dist)
                G=d.xmat[contact['body']].reshape(3,3)
                normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
                contact.update(body_point=G.T@(c.pos-d.xpos[contact['body']]),
                    cube_point=target_B.T@(c.pos-C),body_normal=G.T@normal,cube_normal=target_B.T@normal)
            if stopped:break
    out=BASE/('turn_connection_rolling_probe' if args.rolling_contacts else 'turn_connection_probe');out.mkdir(exist_ok=True)
    if goals:np.savez_compressed(out/'research_waypoints.npz',goals=np.array(goals,dtype=int))
    report=dict(kind='measured_reference_model_turn_connection_diagnostic',
        model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(INITIAL.read_bytes()).hexdigest(),
        accepted_model_waypoints=len(goals),attempted_degrees=records[-1]['requested_turn_degrees'] if records else 0,
        stopped=stopped,records=records,active_motor_ids=[SIM_TO_MOTOR_IDS[j] for j in active],
        contact_point_rollover=args.rolling_contacts,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        static_geometry_diagnostic_only=True,not_a_dense_two_tick_motion_path=True,
        physical_cube_pose_measured=False,executable_candidate=False,
        hardware_access=False,new_SAC_training=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='records'}))
    if records:print('last_waypoint',json.dumps(records[-1]))


if __name__=='__main__':main()
