"""Export SAC actor with parity fixtures and test an uninterrupted native path."""
import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import argparse
import json
import math
import time
import numpy as np
import jax
import mujoco
from brax.training.agents.sac import checkpoint
from native_cycle import OUT,ROOT,assets,Guard,restore,motor,ticks_to_sim,save_json,sha,BOUND
from probe_reference_policy_from_training_start_20261109 import observation_type
from saved_policy_observation_provider_20261109 import build_observation
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from inspect_endpoints import inspect
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate
from scipy.spatial.transform import Rotation

CASES={
    'nominal':{},'x_minus05':{'dx_mm':-.5},'x_plus05':{'dx_mm':.5},
    'y_minus05':{'dy_mm':-.5},'y_plus05':{'dy_mm':.5},
    'yaw_minus1':{'yaw_deg':-1.},'yaw_plus1':{'yaw_deg':1.},
    'friction025':{'friction':.25},'friction035':{'friction':.35},
    'gain090':{'gain':.9},'gain110':{'gain':1.1},
    'vision_x_minus1':{'vision_x_mm':-1.},'vision_x_plus1':{'vision_x_mm':1.},
    'encoder_id1_plus1':{'encoder_id1_ticks':1.},
}


class Actor:
    def __init__(self,path):
        with np.load(path,allow_pickle=False) as z:
            self.mean=z['mean'];self.std=z['std']
            self.layers=[(z[f'w{i}'],z[f'b{i}']) for i in range(3)]
    def action(self,obs):
        x=(np.asarray(obs,np.float32)-self.mean)/self.std
        for i,(w,b) in enumerate(self.layers):
            x=x@w+b
            if i<2:x=np.maximum(x,0)
        return np.tanh(x[:16])


def export(run,step=None,original=False):
    folder=OUT/run;settings=json.loads((folder/'settings.json').read_text())
    checkpoints=sorted((folder/'checkpoints').glob('[0-9]*'),key=lambda p:int(p.name))
    if original:
        selected=ROOT/'sac_forward_support_reference_v1_20k/checkpoints/000000020000'
    elif not checkpoints:
        raise RuntimeError('No saved training checkpoint yet; wait for the first completed epoch')
    else:
        selected=next(p for p in checkpoints if int(p.name)==step) if step is not None else checkpoints[-1]
    norm,weights=checkpoint.load(str(selected));actor=checkpoint.load_policy(str(selected),deterministic=True)
    output=folder/('export_original' if original else 'export_'+selected.name);output.mkdir(exist_ok=True)
    params={'mean':np.asarray(norm.mean,dtype=np.float32),'std':np.asarray(norm.std,dtype=np.float32)}
    for i in range(3):
        layer=weights['params'][f'hidden_{i}']
        params[f'w{i}']=np.asarray(layer['kernel']);params[f'b{i}']=np.asarray(layer['bias'])
    path=output/'actor.npz';np.savez_compressed(path,**params)
    policy=Actor(path)
    m,initial,ref,_,_=assets('active');target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)
    quat=np.empty(4);mujoco.mju_mat2Quat(quat,target.ravel())
    pool=dict(np.load(OUT/'baseline_archived_zero/state_pool.npz',allow_pickle=False));obs=[]
    for i,index in enumerate(pool['indices']):
        z={key:pool[key][i] for key in BOUND};z['time']=pool['time'][i]
        data=restore(m,z);builder=observation_type()()
        for _ in range(3):value=build_observation(builder,m,data,data.ctrl,quat)
        obs.append(np.r_[value,index/max(1,len(ref['goals'])-1),float(index>=len(ref['goals'])-134)])
    values=np.asarray(obs,np.float32)
    default_expected=np.asarray(jax.jit(jax.vmap(lambda x:actor(x,jax.random.PRNGKey(0))[0]))(values))
    # The original export also uses highest precision for mathematical parity;
    # GPU default reduced-precision inference is retained as a separate result.
    with jax.default_matmul_precision('highest'):
        expected=np.asarray(jax.jit(jax.vmap(lambda x:actor(x,jax.random.PRNGKey(0))[0]))(values))
    actual=np.stack([policy.action(x) for x in values]);error=float(np.max(np.abs(expected-actual)))
    np.savez_compressed(output/'parity.npz',observations=values,jax_highest_precision_actions=expected,
                        jax_default_precision_actions=default_expected,numpy_actions=actual)
    report={'format':'leap_current_cycle_sac_numpy_v1','checkpoint':selected.name,
            'actor_sha256':sha(path),'parity_samples':len(values),'max_action_difference':error,
            'parity_precision':'JAX highest matmul precision vs NumPy',
            'gpu_default_precision_max_action_difference':float(np.max(np.abs(default_expected-actual))),
            'parity_passed':error<1e-5,'residual_rad':settings['residual_rad'],
            'control_seconds':.075,'max_goal_ticks':2,'current_model_sha256':settings['current_model_sha256'],
            'reference_sha256':settings['reference_sha256'],'original_actor_continuation':True,
            'numpy_export_is_not_distillation':True,'hardware_execution_allowed':False}
    report['checkpoint_source']=str(selected.relative_to(ROOT))
    report['is_original_actor_before_new_training']=original
    save_json(output/'manifest.json',report)
    if not report['parity_passed']:raise AssertionError(report)
    return output,report


def evaluate(export_folder,manifest,model_kind='active',prefix=None,case='nominal',certify=False):
    m,z,reference,_,provenance=assets(model_kind);d=restore(m,z);guard=Guard(m)
    parameters=CASES[case]
    initial_cube_center=d.qpos[-7:-4].copy()
    d.qpos[-7:-4]+=np.array([parameters.get('dx_mm',0.),parameters.get('dy_mm',0.),0.])/1000
    if 'yaw_deg' in parameters:
        original=Rotation.from_quat(d.qpos[-4:][[1,2,3,0]])
        d.qpos[-4:]=(Rotation.from_rotvec(z['up']*np.radians(parameters['yaw_deg']))*original).as_quat()[[3,0,1,2]]
    if 'friction' in parameters:
        for gid in range(m.ngeom):
            if int(m.geom_bodyid[gid]) in guard.hand or gid==m.geom('cube').id:m.geom_friction[gid,0]=parameters['friction']
    if 'gain' in parameters:
        m.actuator_gainprm[:,0]*=parameters['gain'];m.actuator_biasprm[:,1:3]*=parameters['gain']
    if parameters:mujoco.mj_forward(m,d)
    builder=observation_type()();policy=Actor(export_folder/'actor.npz')
    B0=z['initial_cube_rotation'];target=upright_yaw_target(B0,z['up'],90)
    quat=np.empty(4);mujoco.mju_mat2Quat(quat,target.ravel())
    for _ in range(3):build_observation(builder,m,d,d.ctrl,quat)
    goals=reference['goals'];cache=precompute(m,guard.hand);cube=m.geom('cube').id
    label='evaluation_'+model_kind+('' if case=='nominal' else '_'+case)+('_certified' if certify else '')
    folder=export_folder/(label if prefix is None else prefix);folder.mkdir(exist_ok=True)
    observations=[];actions=[];raw=[];phase=[];stop=None;lower_count=hold_count=0
    tail_gap=float('inf');tail_B=tail_C=None;tail_drift=tail_shift=0.;began=time.perf_counter()
    cert=SweepCertificate(m,guard.hand,.3) if certify else None
    cert_gap=float('inf');cert_bad=0
    from research_rotation_body_hold_20261104 import angle
    for i in range(len(goals)+134):
        nominal=goals[min(i,len(goals)-1)]
        obs=build_observation(builder,m,d,d.ctrl,quat)
        if 'vision_x_mm' in parameters:
            obs[64:73]-=np.tile([parameters['vision_x_mm']/1000,0.,0.],3)
        if 'encoder_id1_ticks' in parameters:
            qbias=np.zeros(16);qbias[15]=parameters['encoder_id1_ticks']*2*np.pi/4096
            obs[:16]+=qbias;obs[16:64]+=np.tile(qbias,3)
        vector=np.r_[obs,min(i/max(1,len(goals)-1),1.),float(i>=len(goals)-134)]
        act=policy.action(vector);previous=motor(d.ctrl)
        proposed=motor(ticks_to_sim(nominal)+manifest['residual_rad']*act)
        goal=previous+np.clip(proposed-previous,-2,2)
        observations.append(vector);actions.append(act);raw.append(goal);phase.append(i)
        if np.any(goal<guard.lo) or np.any(goal>guard.hi):stop=['native_goal_range'];break
        before=d.ctrl.copy();d.ctrl[:]=ticks_to_sim(goal)
        builder.set_last_act(np.clip((d.ctrl-before)/.2,-1,1))
        for _ in range(75):
            mujoco.mj_step(m,d);stop=guard.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial_cube_center)>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=len(goals)+80:
                B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube]
                if tail_B is None:tail_B=B.copy();tail_C=C.copy()
                tail_drift=max(tail_drift,abs(angle(B,tail_B,z['up'])))
                tail_shift=max(tail_shift,float(np.linalg.norm(C-tail_C)*1000))
                force={}
                for ci,c in enumerate(d.contact):
                    a,b=int(c.geom1),int(c.geom2)
                    if cube not in (a,b):continue
                    other=b if a==cube else a;name=m.geom(other).name
                    if name in ('if_tip','th_tip') and -(c.pos-C)@B[:,1]*1000<55/6:
                        f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f)
                        force[name]=force.get(name,0.)+float(f[0])
                lower_count+=all(force.get(n,0.)>.2 for n in ('if_tip','th_tip'));hold_count+=1
                if cert is not None:
                    certified,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3))
                    cert_gap=min(cert_gap,certified);cert_bad+=bool(bad)
            if stop:break
        if i>=len(goals)+80:
            gap=min(v for _,v in gaps(m,d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3),cache,True));tail_gap=min(tail_gap,gap)
        if (i+1)%500==0 or stop:
            print(json.dumps({'checkpoint':manifest['checkpoint'],'model':model_kind,'goals':i+1,'stop':stop,'seconds':time.perf_counter()-began}),flush=True)
        if stop:break
    np.savez_compressed(folder/'closed_loop_trace.npz',observations=observations,actions=actions,goals=raw,reference_indices=phase)
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,
                        up=z['up'],initial_cube_rotation=B0,initial_cube_center=z['initial_cube_center'])
    end=inspect(m,d,z['up'],B0);why=[]
    nominal_end=dict(np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False))
    station_error=float(np.linalg.norm(np.array(end['center'])-nominal_end['qpos'][-7:-4])*1000)
    if stop:why.append('dynamic_guard')
    if end['target90_error_deg']>=5:why.append('quarter_orientation')
    if abs((end['yaw_deg']-90+180)%360-180)>=5:why.append('quarter_yaw')
    if end['up_tilt_deg']>=5:why.append('tilt')
    if station_error>=2:why.append('common_grip_station')
    if hold_count<4050:why.append('hold_not_completed')
    if lower_count/max(1,hold_count)<=.8:why.append('lower_tip_contacts')
    if tail_gap<=.3:why.append('upper_space')
    if tail_drift>=3 or tail_shift>=2:why.append('hold_drift')
    if cert is not None and (cert_bad or cert.metadata()['queries']!=4050 or cert_gap<=.3):why.append('every_1ms_upper_space_certificate')
    report={'status':'offline_closed_loop_current_cycle_evaluation','provenance':provenance,
            'checkpoint':manifest['checkpoint'],'goals_completed':i+1,'planned_goals':len(goals)+134,
            'case':case,'hypothetical_case_parameters':parameters,
            'target90_and_common_station_not_shifted_to_match_perturbation':True,
            'stop':stop,'worst':guard.worst,'endpoint':end,'settled_hold_seconds':hold_count*.001,
            'settled_lower_contact_fraction':lower_count/max(1,hold_count),'settled_upper_gap_sampled_75ms_mm':tail_gap if np.isfinite(tail_gap) else None,
            'settled_yaw_drift_deg':tail_drift,'settled_center_drift_mm':tail_shift,
            'common_station_error_mm':station_error,
            'settled_cube_goal_and_nominal_hold_reference_fixed_feedback_joint_goals_may_change':True,
            'rotation_and_hold_passed':not why,'reasons':why,'next_rotation_passed':False,
            'separation_certificate_1ms_not_yet_run':not certify,
            'separation_certificate':cert.metadata() if cert is not None else None,
            'certificate_bad_steps':cert_bad if cert is not None else None,
            'minimum_certificate_gap_mm':cert_gap if cert is not None and np.isfinite(cert_gap) else None,
            'no_cube_or_hand_reset_during_trial':True,
            'elapsed_seconds':time.perf_counter()-began,'devices_opened':0,'hardware_execution_allowed':False}
    save_json(folder/'review.json',report);print(json.dumps({key:report[key] for key in ('status','checkpoint','goals_completed','rotation_and_hold_passed','reasons','elapsed_seconds')}),flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--step',type=int)
    p.add_argument('--model',choices=('active','archived'),default='active');p.add_argument('--original',action='store_true')
    p.add_argument('--case',choices=tuple(CASES),default='nominal');p.add_argument('--certify',action='store_true')
    p.add_argument('--export-only',action='store_true',help='Export and compare inference only; do not imply a dynamics evaluation')
    p.add_argument('--use-export',help='Existing export directory name; skips GPU export work');a=p.parse_args()
    if a.use_export:
        folder=OUT/a.run/a.use_export;manifest=json.loads((folder/'manifest.json').read_text())
        if not manifest['parity_passed'] or sha(folder/'actor.npz')!=manifest['actor_sha256']:raise RuntimeError('Actor export validation failed')
    else:folder,manifest=export(a.run,a.step,a.original)
    if not a.export_only:evaluate(folder,manifest,a.model,case=a.case,certify=a.certify)
