"""Contact search including an index support, in two finite model plants.

The search samples dynamics every25ms and cannot approve motor execution.
Full 1ms replay and complete geometry checks remain separate requirements.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,hashlib,time
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from propose_inverse_regrasp_ik_20261029 import restore,interp
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from new_pair_grasp_plan_20261027 import OPEN_PATH

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/supported_inverse_search'
class Plant:
    def __init__(self,folder):
        self.folder=folder;self.m=mujoco.MjModel.from_binary_path(str(folder/'checkpoint_model.mjb'))
        m=self.m;self.hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in self.hand:self.hand.add(b)
        self.support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
        self.inspect=cached_contacts(m,self.hand,self.support);self.cube=m.geom('cube').id;self.body=m.body('cube').id
        self.up=-m.opt.gravity/np.linalg.norm(m.opt.gravity);lo,hi=limits();idx=np.asarray(SIM_TO_MOTOR_IDS)-1
        self.lo=(lo[idx]-2048)*TICK;self.hi=(hi[idx]-2048)*TICK
        self.static=mujoco.MjData(m);self.static.qpos[-7:]=[0,0,10,1,0,0,0]
    def evaluate(self,rows):
        m=self.m;d=restore(m,self.folder/'grasp_hold_checkpoint.npz');B=d.xmat[self.body].reshape(3,3).copy();C=d.qpos[-7:-4].copy()
        for q in rows[::max(1,len(rows)//15)]:
            self.static.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,self.static);su,sp=self.inspect(self.static)
            if max(sp.values(),default=0.)>1e-6 or max(su.values(),default=0.)>.15:
                return dict(passed=False,stop='sampled static geometry',yaw_deg=None)
        maxima=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.)
        stop=None;gap=0.;longest=0.
        for index,q in enumerate(rows):
            d.ctrl[:]=ticks_to_sim(q)
            for _ in range(3):
                mujoco.mj_step(m,d,nstep=25);su,sp=self.inspect(d)
                maxima['self_mm']=max(maxima['self_mm'],max(sp.values(),default=0.));maxima['support_mm']=max(maxima['support_mm'],max(su.values(),default=0.))
                maxima['tracking_rad']=max(maxima['tracking_rad'],float(np.max(abs(d.qpos[:16]-d.ctrl))))
                maxima['range_rad']=max(maxima['range_rad'],float(np.max(np.maximum(self.lo-d.qpos[:16],d.qpos[:16]-self.hi))))
                contact=False;external=False
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    if self.cube not in (a,b):continue
                    other=b if a==self.cube else a
                    if int(m.geom_bodyid[other]) in self.hand:
                        maxima['cube_mm']=max(maxima['cube_mm'],max(0.,-float(c.dist)*1000));contact|=c.dist<.0004
                    elif other in self.support and c.dist<.0004:external=True
                gap=0. if contact else gap+.025;longest=max(longest,gap)
                if maxima['self_mm']>1e-6 or maxima['support_mm']>.15 or maxima['cube_mm']>1 or maxima['tracking_rad']>.18 or maxima['range_rad']>1e-6 or gap>.25 or external:
                    stop='sampled original contact/range/tracking/retention bound';break
            if stop:break
        R=d.xmat[self.body].reshape(3,3);quat=Rotation.from_matrix(R@B.T).as_quat()
        angle=float(np.degrees(2*np.arctan2(quat[:3]@self.up,quat[3])))
        tilt=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1.,0.]))@(B@np.array([0.,-1.,0.])), -1,1))))
        return dict(passed=stop is None,stop=stop,yaw_deg=angle,tilt_deg=tilt,center_shift_mm=float(np.linalg.norm(d.qpos[-7:-4]-C)*1000),
            samples_completed=index+1,samples_planned=len(rows),maxima=maxima,longest_sampled_no_hand_contact_s=longest)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--cem',action='store_true');args=parser.parse_args()
    folder=OUT/('cem' if args.cem else 'support_grid');folder.mkdir(parents=True,exist_ok=True)
    original=np.load(ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/candidate.npz')
    ready=original['goals'][original['phases']=='stage_6_grasp_hold'][-1].copy();opened=np.load(OPEN_PATH)['open_goal'].astype(int)
    plants=[Plant(ROOT/'right_hand_rotation_20261029/inverse_low_contact_checkpoint'),Plant(ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint')]
    active=np.array([0,1,2,3,7,8,9,10,11]);lo,hi=limits()
    start=time.perf_counter();reports=[];best=None
    def trial(endpoint,label):
        nonlocal best
        endpoint=np.rint(endpoint).astype(int);target=ready.copy();target[active]=endpoint;target[13]-=128
        support=ready.copy();support[7]=target[7];support[13]-=128
        rows=np.vstack([interp(ready,support),interp(support,target),np.repeat(target[None,:],40,axis=0)])
        if np.any(rows<lo) or np.any(rows>hi):raise RuntimeError('Native goal bounds violated')
        results=[]
        for plant in plants:
            result=plant.evaluate(rows);results.append(result)
            if not result['passed']:break
        passed=len(results)==2 and all(r['passed'] for r in results)
        quality=[abs(r['yaw_deg']+90)+r['tilt_deg']*.5+max(0.,r['center_shift_mm']-15)*.3 for r in results if r['yaw_deg'] is not None]
        score=max(quality,default=300)+(0 if passed else 500)
        row=dict(index=len(reports),label=label,endpoint=target.tolist(),active_goal=endpoint.tolist(),plants=results,passed_sampled_both=passed,
            score=score,sampled_only_not_approval=True,hardware_execution_allowed=False,devices_opened=False)
        if best is None or score<best['score']:
            best=row
            if passed:
                held_open=opened.copy();held_open[13]-=128
                release=np.vstack([interp(target,held_open),interp(held_open,opened)])
                frontmask=np.isin(original['phases'],['stage_5_loaded_grasp','stage_6_grasp_hold'])
                goals=np.vstack([original['goals'][frontmask],rows,release,np.repeat(opened[None,:],20,axis=0)])
                phases=np.r_[original['phases'][frontmask],['stage_7_horizontal_yaw']*(len(rows)-40),['stage_8_horizontal_hold']*40,['stage_9_loaded_open']*len(release),['stage_10_remove_hold']*20]
                path=folder/f"candidate_{row['index']:03d}.npz";np.savez_compressed(path,start=original['start'],goals=goals,phases=phases,
                    empty_preposition=original['empty_preposition'],empty_gap_open=original['empty_gap_open'])
                row.update(candidate=path.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),loaded_goals=len(goals))
            print(json.dumps(dict(event='best',**row)),flush=True)
        reports.append(row)
        (folder/'review.json').write_text(json.dumps(dict(cases=reports,best=best,check_interval_s=.025,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-start),indent=2)+'\n',encoding='utf-8')
        return row
    base=np.array([2815,2642,2970,2193,2048,2747,2607,2965,2091])
    if not args.cem:
        for support in (0,32,64,96,128):
            q=base.copy();q[4]=2048+support;trial(q,f'index_support_{support}')
        return
    rng=np.random.default_rng(290090);center=base.astype(float);center[4]+=64
    spread=np.array([90,90,130,70,42,100,60,100,100.]);lower=np.ceil(lo[active]+16);upper=np.floor(hi[active]-16)
    for generation in range(4):
        population=[]
        for x in [center]+[rng.normal(center,spread) for _ in range(19)]:
            x=np.clip(x,lower,upper);x[4]=np.clip(x[4],2000,2200)
            population.append(trial(x,f'generation_{generation}'))
        elite=sorted([r for r in population if r['passed_sampled_both']],key=lambda r:r['score'])[:4]
        if elite:
            values=np.array([r['active_goal'] for r in elite]);center=values.mean(axis=0);spread=np.maximum(values.std(axis=0)*1.1,spread*.5)
        else:spread*=.8
        print(json.dumps(dict(event='generation',generation=generation,best_score=best['score'],elapsed_seconds=time.perf_counter()-start)),flush=True)
if __name__=='__main__':main()
