"""Native parallel free-cube trajectory search, never a hardware launcher.

Uses the archived SAC trajectory only as an initial shape. This optimizer is
CEM, not SAC retraining. Limits, geometry and masses remain unchanged.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,hashlib,time
from pathlib import Path
import numpy as np
import mujoco
import mujoco.rollout
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from collision_projection_20261025 import CollisionProjection
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
from hand_motor_mapping import ticks_to_sim,TICK

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/corrected_policy_reference'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--iterations',type=int,default=24)
    parser.add_argument('--population',type=int,default=64);args=parser.parse_args()
    p=OUT/'projected_policy_reference_v2.json';blob=p.read_bytes();plan=json.loads(blob)
    assert plan['full_static_path_clear']
    raw=np.asarray(plan['dense_raw_ticks']);base=ticks_to_sim(raw[::3])
    if not np.array_equal(base[-1],ticks_to_sim(raw[-1])):base=np.r_[base,ticks_to_sim(raw[-1:])]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'native_optimization_CAD.xml');set_cube_physics(m,.055,.1)
    assert np.array_equal(m.jnt_qposadr[m.actuator_trnid[:,0]],np.arange(16))
    d=mujoco.MjData(m);reset=np.array(json.loads((ROOT/'sac_cube55_resets.json').read_bytes())['candidates'][0]['qpos'])
    d.qpos[:]=reset;d.ctrl[:]=base[0];m.geom_friction[m.geom('cube').id,0]=.8;mujoco.mj_forward(m,d)
    for _ in range(round(1.5/m.opt.timestep)):mujoco.mj_step(m,d)
    kind=mujoco.mjtState.mjSTATE_FULLPHYSICS;initial=np.empty(mujoco.mj_stateSize(m,kind));mujoco.mj_getState(m,d,initial,kind)
    initial_pos=d.qpos[-7:-4].copy();target=target_quaternion(d.qpos[-4:].copy())
    warm=d.qacc_warmstart.copy();projection=CollisionProjection(m)
    active=np.array([0,3,4,7,12,14,15]);knots=5
    weights=np.array([np.interp(np.linspace(0,1,len(base)),np.linspace(0,1,knots),np.eye(knots)[k]) for k in range(knots)]).T
    lo=np.maximum(m.jnt_range[:16,0],m.actuator_ctrlrange[:,0]);hi=np.minimum(m.jnt_range[:16,1],m.actuator_ctrlrange[:,1])
    def controls(samples):
        commands=np.broadcast_to(base,(len(samples),*base.shape)).copy()
        commands[:,:,active]+=np.einsum('tk,bka->bta',weights,samples)
        commands=np.clip(commands,lo,hi);commands[:,0]=base[0]
        # Realizable, explicitly rate-limited proposed commands, not changes to
        # motor registers or to model joint/control ranges.
        for t in range(1,len(base)):commands[:,t]=commands[:,t-1]+np.clip(commands[:,t]-commands[:,t-1],-8*TICK,8*TICK)
        commands=np.rint((commands+np.pi)/TICK)*TICK-np.pi
        commands=np.concatenate([commands,np.repeat(commands[:,-1:],60,axis=1)],axis=1)
        return commands
    substeps=round(.05/m.opt.timestep);population=args.population;elite=max(6,population//6)
    threads=min(4,os.cpu_count() or 1);datas=[mujoco.MjData(m) for _ in range(threads)]
    rng=np.random.default_rng(20261025);mean=np.zeros((knots,len(active)));std=np.full_like(mean,.10);mean[0]=0;std[0]=0
    best=None;history=[];started=time.monotonic()
    def evaluate(pool,samples,geometry=True):
        commands=controls(samples);native=np.repeat(commands,substeps,axis=1)
        states,_=pool.rollout(m,datas,initial[None],native,initial_warmstart=warm[None])
        positions=states[:,:,1:1+m.nq];quat=positions[:,:,-4:];quat=quat/np.maximum(np.linalg.norm(quat,axis=2,keepdims=True),1e-12)
        angles=2*np.arccos(np.clip(abs(np.einsum('bti,i->bt',quat,target)),0,1))
        displacement=np.linalg.norm(positions[:,:,-7:-4]-initial_pos,axis=2)
        tracking=np.max(abs(positions[:,:,:16]-native),axis=(1,2));last=round(2/m.opt.timestep)
        cost=8*np.mean(angles[:,-last:],axis=1)+2*np.max(angles[:,-last:],axis=1)+.2*np.mean(angles,axis=1)
        cost+=200*np.maximum(displacement.max(axis=1)-.045,0)
        cost+=np.where(displacement.max(axis=1)>.12,100.,0.)
        cost+=25*np.maximum(tracking-.13,0)+np.where(tracking>.18,30.,0.)
        cost+=.2*np.mean(samples**2,axis=(1,2))
        valid=np.isfinite(states).all(axis=(1,2)) & (states[:,-1,0]>=initial[0]+len(native[0])*m.opt.timestep-.001)
        cost=np.where(valid,cost,1e6);overlap=np.zeros(len(samples))
        if geometry:
            for b,cmd in enumerate(commands):
                overlap[b]=max(projection.maximum_overlap(q) for q in cmd[:-60:24])*1000
            cost+=25*overlap+np.where(overlap>.15,30.,0.)
        return cost,tracking,angles[:,-last:].max(axis=1),displacement.max(axis=1),overlap,commands,states
    with mujoco.rollout.Rollout(nthread=threads) as pool:
        for iteration in range(args.iterations):
            samples=np.clip(mean+rng.standard_normal((population,*mean.shape))*std,-.35,.35);samples[:,0]=0
            samples[0]=mean
            if best is not None:samples[1]=best['parameters']
            scores,tracking,errors,shift,overlap,commands,states=evaluate(pool,samples)
            ranking=np.argsort(scores);ids=ranking[:elite];newmean=samples[ids].mean(axis=0);newstd=samples[ids].std(axis=0)
            mean=.25*mean+.75*newmean;std=np.maximum(.025,.25*std+.75*newstd);std[0]=0;mean[0]=0
            winner=int(ranking[0])
            if best is None or scores[winner]<best['score']:
                best=dict(score=float(scores[winner]),parameters=samples[winner].copy(),commands=commands[winner].copy(),
                    states=states[winner].copy(),maximum_tracking_rad=float(tracking[winner]),hold_maximum_target_error_deg=float(np.degrees(errors[winner])),
                    maximum_center_shift_mm=float(shift[winner]*1000),coarse_target_self_overlap_mm=float(overlap[winner]))
            log=dict(iteration=iteration,best_score=best['score'],hold_error_deg=best['hold_maximum_target_error_deg'],
                maximum_tracking_rad=best['maximum_tracking_rad'],coarse_target_self_overlap_mm=best['coarse_target_self_overlap_mm'],
                elapsed_seconds=round(time.monotonic()-started,1));history.append(log)
            print('FULL_CONTACT_CEM',json.dumps(log),flush=True)
            checkpoint={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in best.items() if k!='states'}
            (OUT/'native_contact_CEM_checkpoint.json').write_text(json.dumps(dict(best=checkpoint,history=history,
                optimization_complete=False,hardware_execution_allowed=False,hardware_commands_sent=0),indent=2),encoding='utf-8')
    dense_overlap=max(projection.maximum_overlap(q) for q in best['commands'])*1000
    result={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in best.items() if k!='states'}
    result['frames']=best['states'][::10,1:1+m.nq].tolist()
    report=dict(source_projection_sha256=hashlib.sha256(blob).hexdigest(),method='CEM around archived SAC reference, not SAC retraining',
        initial_state=initial.tolist(),target_quat_wxyz=target.tolist(),friction_hypothesis=.8,model=manifest,
        best=result,history=history,full_command_path_maximum_model_overlap_mm=dense_overlap,
        joint_ranges_changed=False,motor_parameters_identified=False,hardware_execution_allowed=False,
        hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False,optimization_complete=True)
    (OUT/'native_contact_CEM_result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('CEM_COMPLETE','error',result['hold_maximum_target_error_deg'],'static_mm',dense_overlap,flush=True)

if __name__=='__main__':main()
