"""Free finite-stiffness replay from a conditional observed54 contact fit.

The initialization is a hypothesis. No hand or cube coordinates are imposed
after initialization. All original physics bounds are retained.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from cube_contact_material_policy_20261029 import apply_material
ROOT=Path(__file__).resolve().parent;IN=ROOT/'right_hand_rotation_20261029/observed_inverse_contact_fit'
OUT=ROOT/'right_hand_rotation_20261029/observed_inverse_settling'
def main():
    OUT.mkdir(exist_ok=True);state=IN/'gapped_inverse_trial_20261029_184252_272166_conditional_state_v3.npz';z=np.load(state)
    m=mujoco.MjModel.from_binary_path(str(ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint/checkpoint_model.mjb'))
    m.opt.timestep=.001;cube=m.geom('cube').id;body=m.body('cube').id;up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();lo,hi=limits();idx=np.array(SIM_TO_MOTOR_IDS)-1
    lo=(lo[idx]-2048)*TICK;hi=(hi[idx]-2048)*TICK;rows=[]
    for gain in (1.,.5,1.5):
      for mu in (.3,.2,.5):
        m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=gain;m.actuator_biasprm[:,1]*=gain
        apply_material(m,mu,hand,cube);d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(z['actual']);d.ctrl[:]=ticks_to_sim(z['goal']);d.qpos[-7:]=z['cube_qpos'];mujoco.mj_forward(m,d)
        R0=d.xmat[body].reshape(3,3).copy();c0=d.qpos[-7:-4].copy();bad=None;nohand=longest=0.;metrics=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,native_excess_rad=0.);external=False;history=[]
        for step in range(4001):
            if step:mujoco.mj_step(m,d)
            su,sp=inspect(d);metrics['self_mm']=max(metrics['self_mm'],max(sp.values(),default=0.));metrics['support_mm']=max(metrics['support_mm'],max(su.values(),default=0.))
            metrics['tracking_rad']=max(metrics['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));metrics['native_excess_rad']=max(metrics['native_excess_rad'],float(max(np.maximum(lo-d.qpos[:16],d.qpos[:16]-hi))),0.)
            inhand=False;contacts=[]
            for i,c in enumerate(d.contact):
                a,b=int(c.geom1),int(c.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    metrics['cube_mm']=max(metrics['cube_mm'],max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004
                    force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force);ob=int(m.geom_bodyid[other]);R=d.xmat[ob].reshape(3,3)
                    contacts.append(dict(geom=m.geom(other).name,body=m.body(ob).name,distance_mm=float(c.dist)*1000,normal_force_N=float(force[0]),
                        world_point=c.pos.tolist(),finger_body_local_point=(R.T@(c.pos-d.xpos[ob])).tolist(),
                        cube_local_point=(d.xmat[body].reshape(3,3).T@(c.pos-d.xpos[body])).tolist()))
                elif other in support and c.dist<.0004:external=True
            nohand=0 if inhand else nohand+.001;longest=max(longest,nohand)
            if step%500==0:
                quat=Rotation.from_matrix(d.xmat[body].reshape(3,3)@R0.T).as_quat();yaw=float(np.degrees(2*np.arctan2(quat[:3]@up,quat[3])))
                history.append(dict(seconds=step*.001,yaw_change_deg=yaw,observed_plus_change_deg=-53.66281841871696+yaw,
                    cube_contacts=contacts,cube_qpos=d.qpos[-7:].tolist(),hand_qpos=d.qpos[:16].tolist()))
            if metrics['self_mm']>1e-6 or metrics['support_mm']>.15 or metrics['cube_mm']>1 or metrics['tracking_rad']>.18 or metrics['native_excess_rad']>1e-6 or longest>.25 or external:
                bad='unchanged_contact_range_tracking_or_retention_bound';break
            if step==4000:
                name=f'gain{gain:g}_mu{mu:g}';mujoco.mj_saveModel(m,str(OUT/(name+'.mjb')),None)
                spec=mujoco.mjtState.mjSTATE_FULLPHYSICS;s=np.empty(mujoco.mj_stateSize(m,spec));mujoco.mj_getState(m,d,s,spec)
                np.savez_compressed(OUT/(name+'.npz'),state=s,state_spec=int(spec),ctrl=d.ctrl.copy(),warmstart=d.qacc_warmstart.copy())
        row=dict(gain_scale=gain,friction=mu,passed=bad is None,failure=bad,last_second=step*.001,**metrics,external_cube_contact=external,
            longest_no_hand_s=longest,center_shift_mm=float(np.linalg.norm(d.qpos[-7:-4]-c0)*1000),history=history)
        rows.append(row);print(json.dumps({k:v for k,v in row.items() if k!='history'}),flush=True)
        (OUT/'review.json').write_text(json.dumps(dict(status='conditional_observed54_free_settling',cases=rows,initial_fit_not_registered=True,original_bounds_unchanged=True,
            no_coordinates_overwritten_after_initialization=True,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
