"""OFFLINE small thumb clearance candidates from a reproduced partial turn."""
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
ROOT=Path(__file__).resolve().parent;BASE=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/level_book_seating';OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/thumb_clearance_search';OUT.mkdir(exist_ok=True)
def rot(q):return Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--large-roots',action='store_true');a=ap.parse_args();dest=OUT/'larger_root_clearance' if a.large_roots else OUT;dest.mkdir(exist_ok=True)
    stored=np.load(BASE/'measured_loaded_005744_root_seating/case_s40_y15.npz');seedq=stored['qpos'][-1].copy();target=target_quaternion(stored['settled_cube'][-4:])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,dest/'search_scene.xml');set_cube_physics(m,.055,.1);m.dof_armature[:16]=1e6;m.opt.timestep=.001
    mask=m.geom_solref[:,0]>0;m.geom_solref[mask,0]=np.minimum(m.geom_solref[mask,0],.004)
    d=mujoco.MjData(m);mujoco.mj_forward(m,d);palm=m.body('palm').id;R=d.xmat[palm].reshape(3,3);up=-R[:,2];up=up if up[2]>0 else -up;x=R[:,0].copy();x-=up*(x@up);x/=np.linalg.norm(x);m.opt.gravity[:]=-9.81*up
    hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};cube=m.geom('cube').id;zeros=ticks_to_sim(np.zeros(16));step=float(m.opt.timestep)
    def evaluate(delta,extra):
        delta=np.array(delta,dtype=int);a=np.zeros(16,dtype=int);a[:4]=delta;b=a.copy();b[[7,11]]+=extra;dq0=ticks_to_sim(a)-zeros;dq1=ticks_to_sim(b)-zeros;duration=max(.05,max(abs(delta))/2*.05);root_time=max(.05,extra/2*.05)
        t=np.array([0,duration,duration+root_time,duration+root_time+3]);p=np.vstack([seedq[:16],seedq[:16]+dq0,seedq[:16]+dq1,seedq[:16]+dq1]);ts=np.arange(0,t[-1]+step/2,step);poses=np.column_stack([np.interp(ts,t,p[:,j]) for j in range(16)]);vel=np.gradient(poses,step,axis=0);row=dict(thumb_delta_ticks=delta.tolist(),additional_root_ticks=extra)
        if np.any(poses<m.jnt_range[:16,0]-1e-6) or np.any(poses>m.jnt_range[:16,1]+1e-6):return dict(**row,range_rejected=True),None
        d=mujoco.MjData(m);d.qpos[:]=seedq;d.qvel[:]=0;depth=selfdepth=supportdepth=gap=maxgap=0.;external=False;errors=[];trace=[];endnames=set();peaks={}
        for i,ti in enumerate(ts):
            d.qpos[:16]=poses[i];d.qvel[:16]=vel[i];d.ctrl[:]=poses[i];mujoco.mj_forward(m,d);names=set()
            for c in d.contact:
                g1,g2=int(c.geom1),int(c.geom2);h1,h2=int(m.geom_bodyid[g1]) in hand,int(m.geom_bodyid[g2]) in hand
                if c.dist<0 and h1 and h2:selfdepth=max(selfdepth,-float(c.dist)*1000)
                if c.dist<0 and (g1 in support and h2 or g2 in support and h1):supportdepth=max(supportdepth,-float(c.dist)*1000)
                if cube in (g1,g2):
                    other=g2 if g1==cube else g1
                    if int(m.geom_bodyid[other]) in hand:
                        dd=max(0,-float(c.dist)*1000);depth=max(depth,dd);name=m.geom(other).name;peaks[name]=max(peaks.get(name,0),dd)
                        if c.dist<.0003:names.add(name)
                    elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):external=True
            gap=0 if names else gap+step;maxgap=max(maxgap,gap);endnames=names;errors.append(float(np.degrees(2*np.arccos(np.clip(abs(d.qpos[-4:]@target)/np.linalg.norm(d.qpos[-4:]),0,1)))));trace.append(d.qpos.copy())
            if selfdepth>.01 or supportdepth>.15:break
            mujoco.mj_step(m,d)
        trace=np.array(trace);errors=np.array(errors);Rf=rot(trace[-1,-4:]);red=float(np.degrees(np.arccos(np.clip(Rf[:,0]@up,-1,1))));green=float(np.degrees(np.arccos(np.clip((-Rf[:,1])@(-x),-1,1))));hold=float(errors[max(0,len(errors)-round(3/step)):].max());complete=len(errors)==len(ts)
        row.update(complete=complete,final_error_deg=float(errors[-1]),hold_error_deg=hold,red_up_error_deg=red,green_wrist_error_deg=green,max_self_overlap_mm=selfdepth,max_support_overlap_mm=supportdepth,max_cube_overlap_mm=depth,max_no_hand_contact_s=maxgap,external_cube_contact=external,final_supports=sorted(endnames),cube_contact_peak_depths_mm=peaks,
            candidate_accepted=bool(complete and errors[-1]<=20 and hold<=20 and red<=20 and green<=20 and selfdepth<=1e-6 and supportdepth<=.15 and depth<=1 and endnames and maxgap<=.1 and not external))
        return row,(ts[:len(trace)],trace,errors)
    deltas=[[0,0,0,0]]
    for motor,value in itertools.product(range(4),(-128,-64,64,128)):
        delta=[0]*4;delta[motor]=value;deltas.append(delta)
    extras=(0,64,128)
    if a.large_roots:
        deltas=[[0,0,0,0],[0,-128,0,0],[0,128,0,0],[0,0,0,-128],[0,0,0,128]];extras=(256,384,512)
    results=[]
    for number,(delta,extra) in enumerate(itertools.product(deltas,extras)):
        row,trace=evaluate(delta,extra);row['case_index']=number;results.append(row)
        if row.get('candidate_accepted'):
            ts,q,e=trace;np.savez_compressed(dest/f'accepted_{number:03d}.npz',times=ts,qpos=q,goal_errors_deg=e);print('PASS',json.dumps(row),flush=True)
    result=dict(cases=results,model=manifest,source_stall_s40_unregistered=True,source_stall_matches_observed_partial_turn_only_qualitatively=True,
        user_report_thumb_obstruction=True,gravity_contact_and_initial_pose_are_assumptions=True,physical_contact_friction_identified=False,hardware_execution_allowed=False,hardware_commands_sent=0)
    (dest/'search.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print('SUMMARY',json.dumps(dict(cases=len(results),accepted=sum(x.get('candidate_accepted',False) for x in results),best=sorted([r for r in results if r.get('complete') and r['max_self_overlap_mm']<=1e-6],key=lambda r:r['final_error_deg'])[:5])),flush=True)
if __name__=='__main__':main()
