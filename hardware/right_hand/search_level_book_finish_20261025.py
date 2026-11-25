"""OFFLINE continuation candidates under the level-book setup hypothesis."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import itertools,json,argparse
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
ROOT=Path(__file__).resolve().parent;BASE=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/level_book_seating'
def rot(q):return Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--extras',nargs='+',type=int,default=[128,256,384,512,576,640]);ap.add_argument('--seats',nargs='+',type=int,default=[44]);ap.add_argument('--timestep',type=float,default=.002);ap.add_argument('--contact-time',type=float);a=ap.parse_args()
    dest=BASE if a.contact_time is None and a.timestep==.002 else BASE/f'contact_dt{a.timestep:.4f}_tc{a.contact_time}'
    dest.mkdir(exist_ok=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,dest/'finish_scene.xml');set_cube_physics(m,.055,.1);m.dof_armature[:16]=1e6;m.opt.timestep=a.timestep
    original_solref=m.geom_solref.copy()
    if a.contact_time is not None:
        if a.contact_time<2*a.timestep:raise ValueError('Contact time must be at least twice the time step')
        positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],a.contact_time)
    probe=mujoco.MjData(m);mujoco.mj_forward(m,probe);palm=m.body('palm').id;R=probe.xmat[palm].reshape(3,3);up=-R[:,2];up=up if up[2]>0 else -up;x=R[:,0].copy();x-=up*(x@up);x/=np.linalg.norm(x);m.opt.gravity[:]=-9.81*up
    hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};cube=m.geom('cube').id;zeros=ticks_to_sim(np.zeros(16,dtype=int));cases=[];step=float(m.opt.timestep)
    source=json.loads((BASE/'seating_grid.json').read_bytes());seeds=[c for c in source['cases'] if c['book_frame_position_mm'][0] in a.seats and c['book_frame_position_mm'][1]==15 and c['valid_two_link_upright_start']]
    for seed,extra in itertools.product(seeds,a.extras):
        s,yy,_=seed['book_frame_position_mm'];store=np.load(BASE/f'case_s{s}_y{yy}.npz');seedq=store['qpos'][-1].copy();target=target_quaternion(store['settled_cube'][-4:]);delta=np.zeros(16);delta[[7,11]]=extra;dq=ticks_to_sim(delta)-zeros
        duration=max(.05,extra/2*.05);ts=np.arange(0,duration+3+step/2,step);alpha=np.clip(ts/duration,0,1);poses=seedq[:16]+alpha[:,None]*dq;vel=np.gradient(poses,step,axis=0);row=dict(seat_s_mm=s,seat_y_mm=yy,extra_root_ticks=extra)
        if np.any(poses<m.jnt_range[:16,0]-1e-6) or np.any(poses>m.jnt_range[:16,1]+1e-6):row['range_rejected']=True;cases.append(row);continue
        d=mujoco.MjData(m);d.qpos[:]=seedq;d.qvel[:]=0;depth=selfdepth=supportdepth=gap=maxgap=0.;external=False;errors=[];trace=[];endnames=set()
        for i,t in enumerate(ts):
            d.qpos[:16]=poses[i];d.qvel[:16]=vel[i];d.ctrl[:]=poses[i];mujoco.mj_forward(m,d);names=set()
            for c in d.contact:
                b1,b2=int(c.geom1),int(c.geom2);h1,h2=int(m.geom_bodyid[b1]) in hand,int(m.geom_bodyid[b2]) in hand
                if c.dist<0 and h1 and h2:selfdepth=max(selfdepth,-float(c.dist)*1000)
                if c.dist<0 and (b1 in support and h2 or b2 in support and h1):supportdepth=max(supportdepth,-float(c.dist)*1000)
                if cube in (b1,b2):
                    other=b2 if b1==cube else b1
                    if int(m.geom_bodyid[other]) in hand:
                        depth=max(depth,-float(c.dist)*1000)
                        if c.dist<.0003:names.add(m.geom(other).name)
                    elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):external=True
            gap=0 if names else gap+step;maxgap=max(maxgap,gap);endnames=names;errors.append(float(np.degrees(2*np.arccos(np.clip(abs(d.qpos[-4:]@target)/np.linalg.norm(d.qpos[-4:]),0,1)))));trace.append(d.qpos.copy())
            if selfdepth>.01:break
            mujoco.mj_step(m,d)
        errors=np.array(errors);trace=np.array(trace);Rf=rot(trace[-1,-4:]);red=float(np.degrees(np.arccos(np.clip(Rf[:,0]@up,-1,1))));green=float(np.degrees(np.arccos(np.clip((-Rf[:,1])@(-x),-1,1))));hold=float(errors[max(0,len(errors)-round(3/step)):].max());complete=len(errors)==len(ts)
        row.update(complete=complete,final_error_deg=float(errors[-1]),hold_error_deg=hold,red_up_error_deg=red,green_wrist_error_deg=green,max_self_overlap_mm=selfdepth,max_support_overlap_mm=supportdepth,max_cube_overlap_mm=depth,max_no_hand_contact_s=maxgap,external_cube_contact=external,final_supports=sorted(endnames),
            candidate_accepted=bool(complete and errors[-1]<=20 and hold<=20 and red<=20 and green<=20 and selfdepth<=1e-6 and supportdepth<=.15 and depth<=1 and endnames and maxgap<=.1 and not external))
        np.savez_compressed(dest/f'finish_s{s}_y{yy}_extra{extra}.npz',times=ts[:len(trace)],qpos=trace,goal_errors_deg=errors);cases.append(row);print('CASE',json.dumps(row),flush=True)
    result=dict(cases=cases,model=manifest,timestep=a.timestep,hypothetical_contact_time=a.contact_time,contact_parameters_identified=False,contact_geoms_changed=int(np.any(m.geom_solref!=original_solref,axis=1).sum()),level_book_setup_is_hypothesis=True,measured_loaded_path_prescribed=True,physical_friction_identified=False,hardware_execution_allowed=False,hardware_commands_sent=0)
    (dest/'finish_root_sweep.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print('SUMMARY',sum(c.get('candidate_accepted',False) for c in cases),flush=True)
if __name__=='__main__':main()
