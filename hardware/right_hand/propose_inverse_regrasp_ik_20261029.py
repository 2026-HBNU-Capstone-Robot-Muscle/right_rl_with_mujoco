"""Kinematic regrasp/lift/turn hypotheses from a finite replay checkpoint.

Kinematics only proposes targets. It never counts as finite-physics approval.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from new_pair_grasp_plan_20261027 import OPEN_PATH

ROOT=Path(__file__).resolve().parent;CHECK=ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint'
OUT=ROOT/'right_hand_rotation_20261029/inverse_regrasp_ik'
ACTIVE=np.array([0,1,2,3,8,9,10,11]);SIMIDS=np.asarray(SIM_TO_MOTOR_IDS)-1
def interp(a,b):
    n=max(1,int(np.ceil(np.max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)
def raw_from_q(q):
    raw=np.empty(16);raw[SIMIDS]=np.asarray(q)/TICK+2048;return raw
def restore(model,path):
    z=np.load(path);data=mujoco.MjData(model)
    mujoco.mj_setState(model,data,z['state'],mujoco.mjtState(int(z['state_spec'])))
    data.ctrl[:]=z['ctrl'];data.qacc_warmstart[:]=z['warmstart'];mujoco.mj_forward(model,data)
    return data
def main():
    OUT.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CHECK/'checkpoint_model.mjb'))
    base=restore(m,CHECK/'yaw_hold_checkpoint.npz');grasp=restore(m,CHECK/'grasp_hold_checkpoint.npz')
    g=json.loads((CHECK/'grasp_hold_contacts.json').read_bytes())
    baseline=np.rint(raw_from_q(base.ctrl)).astype(int);cube=base.qpos[-7:].copy()
    R0=base.xmat[m.body('cube').id].reshape(3,3).copy();up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    contacts={}
    for name,bodyname,sign in (('mf_tip','mf_ds',-1.),('th_tip','th_ds',1.)):
        row=next(c for c in g['cube_contacts'] if c['other']==name)
        body=m.body(bodyname).id;local=np.asarray(row['finger_body_local_contact'])
        normal_local=grasp.xmat[body].reshape(3,3).T@(sign*grasp.xmat[m.body('cube').id].reshape(3,3)[:,0])
        contacts[name]=(body,local,normal_local,sign)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);d=mujoco.MjData(m);d.qpos[-7:]=[0,0,10,1,0,0,0]
    lo,hi=limits();lower=(np.ceil(lo[ACTIVE]+.18/TICK)-2048)*TICK
    upper=(np.floor(hi[ACTIVE]-.18/TICK)-2048)*TICK
    original=np.load(ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/candidate.npz')
    frontmask=np.isin(original['phases'],['stage_5_loaded_grasp','stage_6_grasp_hold','stage_7_horizontal_yaw'])
    prefix=original['goals'][frontmask];prefix_phases=original['phases'][frontmask]
    opened=np.load(OPEN_PATH)['open_goal'].astype(int);reports=[]
    for station in (0.,.015,.024):
        name=f'contact_z{round(station*1000):02d}';folder=OUT/name;folder.mkdir(exist_ok=True)
        previous=baseline.copy();previous[13]-=128
        sequence=[('ring_clear',q) for q in interp(baseline,previous)]
        keyframes=[];failed=None
        targets=[('regrasp',0.,0.),('lift',0.,.006)]+[('turn',float(a),.006) for a in (-10,-20,-30,-40,-50,-60)]+[('lower',-60.,0.)]
        for phase,angle,height in targets:
            R=Rotation.from_rotvec(up*np.radians(angle)).as_matrix()@R0;center=cube[:3]+up*height
            desired={}
            for tip,(body,point,normal,sign) in contacts.items():
                compression=.0015 if tip=='mf_tip' else .003
                local=np.array([-.0275*sign,-.005,station]);local[0]+=compression*sign
                desired[tip]=(center+R@local,sign*R[:,0])
            def evaluate(x,detail=False):
                raw=previous.astype(float).copy();raw[ACTIVE]=x/TICK+2048
                d.qpos[:16]=ticks_to_sim(raw);mujoco.mj_forward(m,d)
                residual=[];errors=[];normal_errors=[]
                for tip,(body,point,normal,sign) in contacts.items():
                    mat=d.xmat[body].reshape(3,3);p=d.xpos[body]+mat@point
                    target,normal_target=desired[tip];delta=(p-target)*1000
                    residual.extend(delta);errors.append(float(np.linalg.norm(delta)))
                    nn=mat@normal;residual.extend((nn-normal_target)*3)
                    normal_errors.append(float(np.degrees(np.arccos(np.clip(nn@normal_target,-1,1)))))
                residual.extend((x-(previous[ACTIVE]-2048)*TICK)*.5)
                residual.append((x[7]-(2350-2048)*TICK)*1.)
                su,sp=inspect(d);selfmm=max(sp.values(),default=0.);supportmm=max(su.values(),default=0.)
                residual.extend([selfmm*20,supportmm*20])
                return (errors,normal_errors,selfmm,supportmm,raw) if detail else np.asarray(residual)
            seeds=[previous.copy()]
            if phase=='regrasp':
                for root_delta,pip_delta,dip_delta in ((200,-150,-70),(350,-250,-100)):
                    q=previous.copy();q[11]+=root_delta;q[10]+=pip_delta;q[8]+=dip_delta;seeds.append(q)
                q=np.rint(raw_from_q(grasp.ctrl)).astype(int);q[13]=previous[13];seeds.append(q)
            solutions=[]
            for seed in seeds:
                x=np.clip((seed[ACTIVE]-2048)*TICK,lower+1e-8,upper-1e-8)
                result=least_squares(evaluate,x,bounds=(lower,upper),max_nfev=150,ftol=1e-7,xtol=1e-7,gtol=1e-7)
                solutions.append((float(np.linalg.norm(result.fun)),result))
            score,result=min(solutions,key=lambda p:p[0]);err,nerr,se,su,raw=evaluate(result.x,True)
            q=np.rint(raw).astype(int)
            row=dict(phase=phase,angle_deg=angle,height_mm=height*1000,tip_position_errors_mm=err,
                tip_normal_errors_deg=nerr,max_self_mm=se,max_support_mm=su,target=q.tolist(),score=score)
            keyframes.append(row)
            if max(err)>1. or se>1e-6 or su>.15:
                failed='IK position or strict hand geometry';break
            sequence.extend((phase,p) for p in interp(previous,q));sequence.extend((phase+'_hold',q.copy()) for _ in range(15))
            previous=q
        row=dict(name=name,station_mm=station*1000,keyframes=keyframes,kinematic_proposal_passed=failed is None,
            failure=failed,devices_opened=False,hardware_execution_allowed=False,finite_physics_verified=False)
        if failed is None:
            held_open=opened.copy();held_open[13]-=128
            release=np.vstack([interp(previous,held_open),interp(held_open,opened)])
            added=np.asarray([q for _,q in sequence]);
            goals=np.vstack([prefix,added,np.repeat(previous[None,:],40,axis=0),release,np.repeat(opened[None,:],20,axis=0)])
            phases=np.asarray([*prefix_phases,*(['stage_7_horizontal_yaw']*len(added)),
                *(['stage_8_horizontal_hold']*40),*(['stage_9_loaded_open']*len(release)),*(['stage_10_remove_hold']*20)])
            z=folder/'candidate.npz';np.savez_compressed(z,start=original['start'],goals=goals,phases=phases,
                empty_preposition=original['empty_preposition'],empty_gap_open=original['empty_gap_open'])
            row.update(candidate=z.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(z.read_bytes()).hexdigest(),
                loaded_goals=len(goals),extension_goals=len(added),extension_only_phases=[p for p,_ in sequence],
                total_nominal_motion_s=(len(goals)+1327)*.075)
        reports.append(row);print(json.dumps({k:v for k,v in row.items() if k not in ('keyframes','extension_only_phases')}),flush=True)
        (folder/'proposal.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
        (OUT/'review.json').write_text(json.dumps(dict(cases=reports,devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
