"""Conditional continuation dynamics with no prescribed Cube/hand movement."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from cube_contact_material_policy_20261029 import apply_material
ROOT=Path(__file__).resolve().parent
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--candidate',required=True);ap.add_argument('--all',action='store_true');a=ap.parse_args()
    src=ROOT/a.candidate;out=src.parent/'conditional_dynamics';out.mkdir(exist_ok=True);path=np.load(src)
    z=np.load(ROOT/'right_hand_rotation_20261029/observed_inverse_contact_fit/gapped_inverse_trial_20261029_184252_272166_conditional_state_v3.npz')
    tail=path['tail'];goal=path['tail_start'];release=path['goals'][path['phases']=='stage_9_loaded_open']
    goals=np.vstack([tail,np.repeat(tail[-1][None,:],40,0),release,np.repeat(release[-1][None,:],20,0)])
    phases=['tail']*len(tail)+['tail_hold']*40+['release']*len(release)+['open_hold']*20
    assert np.max(abs(np.diff(np.vstack([goal,goals]),axis=0)))<=2
    m=mujoco.MjModel.from_binary_path(str(ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint/checkpoint_model.mjb'));m.opt.timestep=.001
    cube=m.geom('cube').id;body=m.body('cube').id;up=-m.opt.gravity/np.linalg.norm(m.opt.gravity);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();lo,hi=limits();idx=np.array(SIM_TO_MOTOR_IDS)-1
    assert np.all(goals>=lo) and np.all(goals<=hi);lo=(lo[idx]-2048)*TICK;hi=(hi[idx]-2048)*TICK;cases=[]
    for gain,mu in ([(g,f) for g in (1.,.5,1.5) for f in (.3,.2,.5)] if a.all else [(1.,.3)]):
        m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=gain;m.actuator_biasprm[:,1]*=gain;apply_material(m,mu,hand,cube)
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(z['actual']);d.ctrl[:]=ticks_to_sim(z['goal']);d.qpos[-7:]=z['cube_qpos'];mujoco.mj_forward(m,d)
        R0=d.xmat[body].reshape(3,3).copy();C0=d.qpos[-7:-4].copy();metrics=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,native_excess_rad=0.)
        gap=longest=0.;external=False;bad=None;history=[];hold=None
        for k,(q,phase) in enumerate(zip(goals,phases)):
            d.ctrl[:]=ticks_to_sim(q)
            for step in range(75):
                mujoco.mj_step(m,d);su,sp=inspect(d)
                metrics['self_mm']=max(metrics['self_mm'],max(sp.values(),default=0.));metrics['support_mm']=max(metrics['support_mm'],max(su.values(),default=0.))
                metrics['tracking_rad']=max(metrics['tracking_rad'],float(np.max(abs(d.qpos[:16]-d.ctrl))))
                metrics['native_excess_rad']=max(metrics['native_excess_rad'],0.,float(np.max(np.maximum(lo-d.qpos[:16],d.qpos[:16]-hi))))
                inhand=False
                for c in d.contact:
                    aa,b=int(c.geom1),int(c.geom2)
                    if cube in (aa,b):
                        other=b if aa==cube else aa
                        if int(m.geom_bodyid[other]) in hand:
                            metrics['cube_mm']=max(metrics['cube_mm'],max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004
                        elif other in support and c.dist<.0004:external=True
                gap=0. if inhand else gap+.001;longest=max(longest,gap)
                if metrics['self_mm']>1e-6 or metrics['support_mm']>.15 or metrics['cube_mm']>1 or metrics['tracking_rad']>.18 or metrics['native_excess_rad']>1e-6 or longest>.25 or external:
                    bad='original_contact_range_tracking_retention_or_external_bound';break
            quat=Rotation.from_matrix(d.xmat[body].reshape(3,3)@R0.T).as_quat();delta=float(np.degrees(2*np.arctan2(quat[:3]@up,quat[3])))
            if k%40==0 or phase=='tail_hold':history.append(dict(sample=k+1,phase=phase,delta_deg=delta,observed_plus_delta_deg=-53.66281841871696+delta))
            if phase=='tail_hold':hold=-53.66281841871696+delta
            if bad:break
        row=dict(gain_scale=gain,friction=mu,passed=bad is None,stop=bad,last_phase=phase,last_sample=k+1,samples=len(goals),
            observed_plus_held_delta_deg=hold,observed_plus_final_delta_deg=-53.66281841871696+delta,center_shift_mm=float(np.linalg.norm(d.qpos[-7:-4]-C0)*1000),
            longest_no_hand_s=longest,external_cube_contact=external,history=history,**metrics)
        cases.append(row);print(json.dumps({key:value for key,value in row.items() if key!='history'}),flush=True)
        (out/'review.json').write_text(json.dumps(dict(status='conditional_observed54_continuation',candidate=a.candidate,candidate_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),cases=cases,
            all_passed=all(r['passed'] for r in cases),initial_state_not_registered=True,full_prefix_not_replayed=True,no_coordinates_prescribed_after_initialization=True,
            original_bounds_unchanged=True,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
