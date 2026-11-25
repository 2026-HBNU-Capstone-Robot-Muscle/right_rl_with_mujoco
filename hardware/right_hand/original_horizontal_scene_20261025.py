"""Shared device-free contact rollout for the recorded horizontal reference."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from adapt_original_sequence_clearance_20261024 import build, OUT
from hand_motor_mapping import ticks_to_sim,sim_to_ticks
from sac_cube_model import set_cube_physics


class Scene:
    def __init__(self,*,model_kind='native_box',initial_raw=None,placement=None):
        prior=json.loads((OUT/'horizontal_cube_dynamics.json').read_bytes())
        rows=[r for r in prior['runs'] if r['valid_start'] and r['control_dt_s']==.05]
        rows.sort(key=lambda r:(not r['final_three_second_hold_retained'],r['final']['ninety_yaw_target_error_deg']))
        self.placement=rows[0]['placement']
        prefix=json.loads((OUT/'horizontal_reference_primitive.json').read_bytes())
        self.prefix_raw=np.array(prefix['goal_ticks']);self.prefix=ticks_to_sim(self.prefix_raw)
        if initial_raw is not None:
            self.prefix_raw=np.array(initial_raw,dtype=int).reshape(-1,16);self.prefix=ticks_to_sim(self.prefix_raw)
        if placement is not None:self.placement=dict(placement)
        if 'R' not in self.placement:
            quat=np.asarray(self.placement['quat_wxyz'],float)
            if quat.shape!=(4,) or not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1)>.001:
                raise ValueError('Invalid placement quaternion')
            self.placement['R']=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix().tolist()
        rotation=np.asarray(self.placement['R'],float)
        if rotation.shape!=(3,3) or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-5) or np.linalg.det(rotation)<.99:
            raise ValueError('Invalid placement rotation')
        if model_kind=='CAD_tip':
            from mujoco_playground import registry
            from fingertip_cad_model_20261025 import build as build_cad
            cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
            self.m,self.manifest=build_cad(env,OUT/'CAD_scene_rollout.xml')
        else:self.m,self.manifest=build()
        set_cube_physics(self.m,side_m=.055,mass_kg=.1)
        self.d=mujoco.MjData(self.m);self.cube=self.m.geom('cube').id;self.body=int(self.m.geom_bodyid[self.cube])
        self.q0=self.prefix[-1];self.raw0=self.prefix_raw[-1]
        self.up=-self.m.opt.gravity/np.linalg.norm(self.m.opt.gravity);self.dt=self.m.opt.timestep
        self.hand={self.m.body('palm').id}
        for b in range(self.m.nbody):
            if int(self.m.body_parentid[b]) in self.hand:self.hand.add(b)
        self.support={self.m.geom(n).id for n in ('support_book','support_glass','support_wood')}
        self.kind=mujoco.mjtState.mjSTATE_INTEGRATION
        self.state_size=mujoco.mj_stateSize(self.m,self.kind);self.caches={}
        self.reference_static_check=self.static_check(self.prefix_raw)
        if not self.reference_static_check['clear']:raise ValueError('Reference fails the current compiled geometry model')
    def pin(self,q,velocity=None):
        self.d.qpos[:16]=q;self.d.qvel[:16]=0 if velocity is None else velocity
        self.d.ctrl[:]=q;mujoco.mj_step(self.m,self.d)
    def baseline(self,friction):
        m,d=self.m,self.d;mujoco.mj_resetData(m,d);m.geom_friction[self.cube,0]=friction
        d.qpos[:16]=self.prefix[0];d.ctrl[:]=self.prefix[0]
        d.qpos[-7:]=np.r_[self.placement['center_m'],Rotation.from_matrix(self.placement['R']).as_quat()[[3,0,1,2]]]
        mujoco.mj_forward(m,d)
        for _ in range(round(1.5/self.dt)):self.pin(self.prefix[0])
        R0=d.xmat[self.body].reshape(3,3).copy();p0=d.qpos[-7:-4].copy()
        times=np.arange(len(self.prefix))*.05;elapsed=0.
        while elapsed<=times[-1]+2.:
            q=np.array([np.interp(elapsed,times,self.prefix[:,j]) for j in range(16)])
            nxt=np.array([np.interp(elapsed+self.dt,times,self.prefix[:,j]) for j in range(16)])
            self.pin(q,(nxt-q)/self.dt);elapsed+=self.dt
        self.caches[friction]=dict(state=self.get_state(),R0=R0,p0=p0,
            target=Rotation.from_rotvec(self.up*np.pi/2).as_matrix()@R0)
    def get_state(self):
        state=np.empty(self.state_size);mujoco.mj_getState(self.m,self.d,state,self.kind);return state
    def restore(self,state):
        mujoco.mj_setState(self.m,self.d,state,self.kind);mujoco.mj_forward(self.m,self.d)
    def reset_baseline(self,friction):
        if friction not in self.caches:self.baseline(friction)
        self.m.geom_friction[self.cube,0]=friction;self.restore(self.caches[friction]['state'])
    def contacts(self):
        touching=[]
        for c in self.d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if self.cube not in (a,b) or c.dist>.0003:continue
            other=b if a==self.cube else a;name=self.m.body(int(self.m.geom_bodyid[other])).name
            if name=='palm' or name.startswith(('if_','mf_','rf_','th_')):touching.append(name)
        return sorted(set(touching))
    def status(self,friction):
        cache=self.caches[friction];R=self.d.xmat[self.body].reshape(3,3).copy()
        initial=cache['R0'];a=initial[:,0]-self.up*(self.up@initial[:,0]);b=R[:,0]-self.up*(self.up@R[:,0])
        contacts=self.contacts();shift=float(np.linalg.norm(self.d.qpos[-7:-4]-cache['p0'])*1000)
        return dict(yaw_heading_deg=float(np.degrees(np.arctan2(self.up@np.cross(a,b),a@b))),
            ninety_target_error_deg=float(np.degrees(Rotation.from_matrix(R@cache['target'].T).magnitude())),
            tilt_change_deg=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1,0]))@(initial@np.array([0.,-1,0])),-1,1)))),
            displacement_mm=shift,hand_contacts=contacts,retained=bool(contacts and shift<100),
            distinct_finger_contact_count=len({n.split('_')[0] for n in contacts if n!='palm'}),
            cube_center_m=self.d.qpos[-7:-4].tolist(),cube_quat_wxyz=self.d.qpos[-4:].tolist())
    def static_check(self,raw):
        bad=[];maximum=0.;m,d=self.m,self.d
        for index,q in enumerate(ticks_to_sim(raw)):
            d.qpos[:16]=q;d.qpos[-7:]=[10,10,10,1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d)
            for c in d.contact:
                if c.dist>=-1e-7:continue
                a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in self.hand,int(m.geom_bodyid[b]) in self.hand
                if ha and hb or a in self.support and hb or b in self.support and ha:
                    depth=-float(c.dist)*1000;maximum=max(maximum,depth)
                    if len(bad)<12:bad.append(dict(index=index,pair=[m.geom(a).name,m.geom(b).name],model_overlap_mm=depth))
        return dict(clear=not bool(bad),maximum_model_overlap_mm=maximum,examples=bad)
    def rollout(self,raw,friction=.8,hold_s=3.,record=False,start_state=None,prescribed_hand=True,gain_scale=1.):
        self.reset_baseline(friction)
        if start_state is not None:self.restore(start_state)
        previous=self.d.qpos[:16].copy();frames=[]
        gains=self.m.actuator_gainprm.copy();bias=self.m.actuator_biasprm.copy()
        self.m.actuator_gainprm[:,0]*=gain_scale;self.m.actuator_biasprm[:,1:3]*=gain_scale
        max_tracking=0.;max_self=0.;max_support=0.
        def advance(q,velocity=None):
            nonlocal max_tracking,max_self,max_support
            if prescribed_hand:self.pin(q,velocity)
            else:self.d.ctrl[:]=q;mujoco.mj_step(self.m,self.d)
            max_tracking=max(max_tracking,float(np.max(abs(self.d.qpos[:16]-q))))
            for c in self.d.contact:
                if c.dist>=0:continue
                a,b=int(c.geom1),int(c.geom2);ha,hb=int(self.m.geom_bodyid[a]) in self.hand,int(self.m.geom_bodyid[b]) in self.hand
                if ha and hb:max_self=max(max_self,-float(c.dist)*1000)
                elif a in self.support and hb or b in self.support and ha:max_support=max(max_support,-float(c.dist)*1000)
        for sample_index,q in enumerate(ticks_to_sim(raw)):
            cycles=round(.05/self.dt);v=(q-previous)/.05
            for k in range(1,cycles+1):advance(previous+(q-previous)*k/cycles,v)
            previous=q
            if record and sample_index%2==0:frames.append(self.d.qpos.tolist())
            if np.linalg.norm(self.d.qpos[-7:-4]-self.caches[friction]['p0'])>.2:break
        holds=[]
        for k in range(round(hold_s/self.dt)):
            advance(previous)
            if (k+1)%round(.25/self.dt)==0:holds.append(self.status(friction))
            if record and k%10==0:frames.append(self.d.qpos.tolist())
        final=self.status(friction)
        self.m.actuator_gainprm[:]=gains;self.m.actuator_biasprm[:]=bias
        return dict(final=final,hold_retained=all(s['retained'] for s in holds),hold_samples=holds,
            maximum_joint_tracking_rad=max_tracking,maximum_hand_self_model_overlap_mm=max_self,
            maximum_hand_support_model_overlap_mm=max_support,
            native_actuator_force_limits_enabled=self.m.actuator_forcelimited.tolist(),
            state=self.get_state(),frames=frames)


def interpolate(raw0,raw1,indices=None):
    raw0=np.asarray(raw0,dtype=int);raw1=np.asarray(raw1,dtype=int)
    if indices is not None:
        goal=raw0.copy();goal[np.asarray(indices)]=raw1[np.asarray(indices)]
    else:goal=raw1
    steps=max(1,int(np.ceil(np.max(abs(goal-raw0))/2)))
    return np.array([np.rint(raw0+(goal-raw0)*k/steps).astype(int) for k in range(1,steps+1)])
