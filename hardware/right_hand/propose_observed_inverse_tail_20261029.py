"""Contact-following IK hypotheses from the observed54 encoder/angle fit.

No coordinates are prescribed in subsequent dynamic reviews. This proposer
does not approve a motor path and does not change controller gains or limits.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from propose_inverse_regrasp_ik_20261029 import interp
from new_pair_grasp_plan_20261027 import OPEN_PATH
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/observed_inverse_tail'
ACTIVE=np.array([0,1,2,3,8,9,10,11]);IDX=np.array(SIM_TO_MOTOR_IDS)-1
def main():
    OUT.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint/checkpoint_model.mjb'))
    z=np.load(ROOT/'right_hand_rotation_20261029/observed_inverse_contact_fit/gapped_inverse_trial_20261029_184252_272166_conditional_state_v3.npz')
    actual=z['actual'];goal=z['goal'];offset=goal-actual;up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(actual);d.qpos[-7:]=z['cube_qpos'];mujoco.mj_forward(m,d)
    cube=m.geom('cube').id;cb=m.body('cube').id;C=d.xpos[cb].copy();R0=d.xmat[cb].reshape(3,3).copy();anchors={}
    for name,bname in (('mf_tip','mf_ds'),('th_tip','th_ds')):
        p=np.zeros(6);g=m.geom(name).id;mujoco.mj_geomDistance(m,d,cube,g,.2,p);b=m.body(bname).id;R=d.xmat[b].reshape(3,3)
        # Near-surface witness points only; full SAT and engine distances were
        # independently checked when creating this conditional fit.
        finger_local=R.T@(p[3:]-d.xpos[b]);cube_local=np.clip(R0.T@(p[:3]-C),-.0275,.0275)
        inward=-np.sign(cube_local[0])*R0[:,0];normal_local=R.T@inward
        anchors[name]=(b,finger_local,cube_local,normal_local)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);d.qpos[-7:]=[0,0,10,1,0,0,0];lo,hi=limits()
    lower=(np.maximum(lo[ACTIVE]+52,lo[ACTIVE]-offset[ACTIVE]+2)-2048)*TICK
    upper=(np.minimum(hi[ACTIVE]-52,hi[ACTIVE]-offset[ACTIVE]-2)-2048)*TICK
    original=np.load(ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/candidate.npz')
    prefixmask=np.isin(original['phases'],['stage_5_loaded_grasp','stage_6_grasp_hold','stage_7_horizontal_yaw'])
    prefix=original['goals'][prefixmask];opened=np.load(OPEN_PATH)['open_goal'].astype(int);reports=[]
    for height_mm in (0.,3.,6.):
      for nw in (0.,2.,5.):
        name=f'height{height_mm:g}_normal{nw:g}';folder=OUT/name;folder.mkdir(exist_ok=True)
        previous=actual.copy();previous[13]-=128;previous_command=goal.copy();previous_command[13]-=128
        sequence=list(interp(goal,previous_command));frames=[];failed=None
        for angle in (-5.,-10.,-15.,-20.,-25.,-30.,-36.3371815813):
            R=Rotation.from_rotvec(up*np.radians(angle)).as_matrix()@R0
            center=C+up*(height_mm/1000)*min(1.,abs(angle)/15)
            def evaluate(x,detail=False):
                raw=previous.copy();raw[ACTIVE]=x/TICK+2048;d.qpos[:16]=ticks_to_sim(raw);mujoco.mj_forward(m,d)
                errors=[];norm_errors=[];res=[]
                for tip,(b,p,cl,normal) in anchors.items():
                    B=d.xmat[b].reshape(3,3);point=d.xpos[b]+B@p;inward=-np.sign(cl[0])*R[:,0]
                    target=center+R@cl+inward*.0004
                    delta=(point-target)*1000;res.extend(delta);errors.append(float(np.linalg.norm(delta)))
                    current=B@normal;res.extend((current-inward)*nw)
                    norm_errors.append(float(np.degrees(np.arccos(np.clip(current@inward,-1,1)))))
                su,sp=inspect(d);selfmm=max(sp.values(),default=0.);supmm=max(su.values(),default=0.)
                res.extend((x-(previous[ACTIVE]-2048)*TICK)*.3);res.extend([selfmm*100,supmm*100])
                return dict(position_errors_mm=errors,normal_errors_deg=norm_errors,self_mm=selfmm,support_mm=supmm,actual=raw.tolist()) if detail else np.asarray(res)
            solutions=[]
            seeds=[previous.copy()]
            alternate=previous.copy();alternate[11]+=80;alternate[10]-=50;alternate[0]+=32;alternate[1]+=32;seeds.append(alternate)
            for seed in seeds:
                x=np.clip((seed[ACTIVE]-2048)*TICK,lower+1e-8,upper-1e-8)
                sol=least_squares(evaluate,x,bounds=(lower,upper),max_nfev=130,ftol=1e-7,xtol=1e-7,gtol=1e-7)
                solutions.append((float(np.linalg.norm(sol.fun)),sol))
            score,sol=min(solutions,key=lambda a:a[0]);info=evaluate(sol.x,True);raw=np.array(info['actual']);command=np.rint(raw+offset).astype(int)
            info.update(angle_deg=angle,height_mm=height_mm,normal_weight=nw,command=command.tolist(),score=score);frames.append(info)
            if max(info['position_errors_mm'])>1. or info['self_mm']>1e-6 or info['support_mm']>.15 or np.any(command<lo) or np.any(command>hi):failed='contact_target_or_original_geometry_or_native_bound';break
            sequence.extend(interp(previous_command,command));previous=raw;previous_command=command
        row=dict(name=name,frames=frames,kinematic_passed=failed is None,failure=failed,hardware_execution_allowed=False,devices_opened=False,conditional_initial_pose_not_registered=True)
        if failed is None:
            tail=np.array(sequence);held_open=opened.copy();held_open[13]-=128
            release=np.vstack([interp(previous_command,held_open),interp(held_open,opened)])
            goals=np.vstack([prefix,tail,np.repeat(previous_command[None,:],40,0),release,np.repeat(opened[None,:],20,0)])
            phases=np.r_[original['phases'][prefixmask],['stage_7_horizontal_yaw']*len(tail),['stage_8_horizontal_hold']*40,['stage_9_loaded_open']*len(release),['stage_10_remove_hold']*20]
            path=folder/'candidate.npz';np.savez_compressed(path,start=original['start'],goals=goals,phases=phases,empty_preposition=original['empty_preposition'],empty_gap_open=original['empty_gap_open'],tail=tail,tail_start=goal,tail_end=previous_command)
            row.update(candidate=path.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),tail_goals=len(tail),loaded_goals=len(goals))
        reports.append(row);(folder/'proposal.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k!='frames'}),flush=True)
        (OUT/'review.json').write_text(json.dumps(dict(cases=reports,physical_rotation_not_verified=True,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
