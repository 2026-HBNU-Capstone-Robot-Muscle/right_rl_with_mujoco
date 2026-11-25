"""Coarse endpoint search with finite dynamics; never a clearance certificate.

Checks every25ms for speed during hypothesis search. A selected route must be
replayed at every1ms with the original acceptance bounds before it is usable.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib,time
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from propose_inverse_regrasp_ik_20261029 import restore,interp
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from new_pair_grasp_plan_20261027 import OPEN_PATH

ROOT=Path(__file__).resolve().parent
CHECK=ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint'
OUT=ROOT/'right_hand_rotation_20261029/inverse_contact_search_v2'
ACTIVE=np.array([0,1,2,3,8,9,10,11])

def main():
    OUT.mkdir(exist_ok=True);began=time.perf_counter();rng=np.random.default_rng(290054)
    m=mujoco.MjModel.from_binary_path(str(CHECK/'checkpoint_model.mjb'))
    original=np.load(ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/candidate.npz')
    ready=original['goals'][original['phases']=='stage_6_grasp_hold'][-1].copy()
    opened=np.load(OPEN_PATH)['open_goal'].astype(int);lo,hi=limits();sid=np.array(SIM_TO_MOTOR_IDS)-1
    nlo=(lo[sid]-2048)*TICK;nhi=(hi[sid]-2048)*TICK
    lower=np.ceil(lo[ACTIVE]+16);upper=np.floor(hi[ACTIVE]-16) # native limits unchanged; actual-state bounds still enforced
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);cube=m.geom('cube').id;cube_body=m.body('cube').id
    up=-m.opt.gravity/np.linalg.norm(m.opt.gravity);static=mujoco.MjData(m);static.qpos[-7:]=[0,0,10,1,0,0,0]
    records=[];best=None
    center=np.array([2720,2674,3068,2308,2791,2613,3022,2122.])
    spread=np.array([90,100,140,80,120,80,120,100.])
    def run(endpoint):
        d=restore(m,CHECK/'grasp_hold_checkpoint.npz');B=d.xmat[cube_body].reshape(3,3).copy();C=d.qpos[-7:-4].copy()
        cleared=ready.copy();cleared[13]-=128
        target=cleared.copy();target[ACTIVE]=endpoint
        rows=np.vstack([interp(ready,cleared),interp(cleared,target),np.repeat(target[None,:],40,axis=0)])
        for q in rows[::max(1,len(rows)//12)]:
            static.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,static);su,sp=inspect(static)
            if max(sp.values(),default=0.)>1e-6 or max(su.values(),default=0.)>.15:
                return dict(passed_sampled_screen=False,stop='static goal geometry',yaw_deg=None,score=1000.)
        mm=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);stop=None;gap=0.;longest=0.
        for i,q in enumerate(rows):
            d.ctrl[:]=ticks_to_sim(q)
            for _ in range(3):
                mujoco.mj_step(m,d,nstep=25);su,sp=inspect(d)
                mm['self_mm']=max(mm['self_mm'],max(sp.values(),default=0.));mm['support_mm']=max(mm['support_mm'],max(su.values(),default=0.))
                mm['tracking_rad']=max(mm['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))))
                mm['range_rad']=max(mm['range_rad'],float(max(np.maximum(nlo-d.qpos[:16],d.qpos[:16]-nhi))))
                external=False;contact=False
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    if cube not in (a,b):continue
                    other=b if a==cube else a
                    if int(m.geom_bodyid[other]) in hand:
                        mm['cube_mm']=max(mm['cube_mm'],max(0.,-float(c.dist)*1000));contact|=c.dist<.0004
                    elif other in support and c.dist<.0004:external=True
                gap=0. if contact else gap+.025;longest=max(longest,gap)
                if mm['self_mm']>1e-6 or mm['support_mm']>.15 or mm['cube_mm']>1 or mm['tracking_rad']>.18 or mm['range_rad']>1e-6 or gap>.25 or external:
                    stop='sampled original contact/range/tracking/retention bound';break
            if stop:break
        R=d.xmat[cube_body].reshape(3,3);rq=Rotation.from_matrix(R@B.T).as_quat()
        angle=float(np.degrees(2*np.arctan2(rq[:3]@up,rq[3])))
        tilt=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1.,0.]))@(B@np.array([0.,-1.,0.])), -1,1))))
        shift=float(np.linalg.norm(d.qpos[-7:-4]-C)*1000)
        score=abs(angle+90)+tilt*.5+max(0.,shift-20)*.5+(300 if stop else 0)
        return dict(passed_sampled_screen=stop is None,stop=stop,yaw_deg=angle,tilt_deg=tilt,center_shift_mm=shift,
            score=score,samples_completed=i+1,samples_planned=len(rows),maxima=mm,longest_sampled_no_hand_contact_s=longest,
            endpoint=target.tolist())
    for generation in range(4):
        proposals=[center]+[rng.normal(center,spread) for _ in range(19)]
        population=[]
        for x in proposals:
            endpoint=np.rint(np.clip(x,lower,upper)).astype(int);report=run(endpoint)
            report.update(generation=generation,index=len(records),active_goal=endpoint.tolist(),sampled_only_not_approval=True,devices_opened=False,hardware_execution_allowed=False)
            records.append(report);population.append(report)
            if best is None or report['score']<best['score']:
                best=report;print(json.dumps(dict(event='best',**report)),flush=True)
                if report['passed_sampled_screen']:
                    target=np.asarray(report['endpoint']);cleared=ready.copy();cleared[13]-=128
                    turn=np.vstack([interp(ready,cleared),interp(cleared,target)])
                    held=opened.copy();held[13]-=128;release=np.vstack([interp(target,held),interp(held,opened)])
                    frontmask=np.isin(original['phases'],['stage_5_loaded_grasp','stage_6_grasp_hold'])
                    goals=np.vstack([original['goals'][frontmask],turn,np.repeat(target[None,:],40,axis=0),release,np.repeat(opened[None,:],20,axis=0)])
                    phases=np.r_[original['phases'][frontmask],['stage_7_horizontal_yaw']*len(turn),['stage_8_horizontal_hold']*40,['stage_9_loaded_open']*len(release),['stage_10_remove_hold']*20]
                    file=OUT/f"candidate_{report['index']:03d}.npz";np.savez_compressed(file,start=original['start'],goals=goals,phases=phases,
                        empty_preposition=original['empty_preposition'],empty_gap_open=original['empty_gap_open'])
                    report.update(candidate=file.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(file.read_bytes()).hexdigest(),loaded_goals=len(goals))
            (OUT/'review.json').write_text(json.dumps(dict(cases=records,best=best,check_interval_s=.025,full_route_verified=False,
                hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began),indent=2)+'\n',encoding='utf-8')
        elite=sorted([r for r in population if r['passed_sampled_screen']],key=lambda r:r['score'])[:4]
        if elite:
            a=np.asarray([r['active_goal'] for r in elite]);center=a.mean(axis=0);spread=np.maximum(a.std(axis=0)*1.1,spread*.5)
        else:spread*=.75
        print(json.dumps(dict(event='generation',generation=generation,best=best['score'],elapsed_seconds=time.perf_counter()-began)),flush=True)
if __name__=='__main__':main()
