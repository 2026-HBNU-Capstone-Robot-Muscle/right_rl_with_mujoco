"""Offline IK proposals and free rigid-cube continuations of a saved checkpoint.

Pinned cube poses are used only to propose geometry. Dynamic evaluation never
resets or welds the cube and retains the saved actuator, contact and range rules.
No physical device access and no executable operator handoff are provided.
"""
import argparse, json, hashlib, time
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from hand_motor_mapping import SIM_TO_MOTOR_IDS, TICK, ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from research_contact_inspector_20261104 import cached_contacts

R=Path(__file__).resolve().parent
CP=R/'right_hand_rotation_20261104/raised_cradle_160_yaw0/trace_nominal'
OUT=R/'right_hand_rotation_20261104/rotation_hold_session'
ACTIVE=np.array([4,5,6,7,12,13,14,15])

def native(m):
    lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1
    return lo,hi,(lo[ids]-2048)*TICK,(hi[ids]-2048)*TICK

def motor(q):
    raw=np.zeros(16,dtype=int)
    raw[np.array(SIM_TO_MOTOR_IDS)-1]=np.rint(q/TICK+2048).astype(int)
    return raw

def restore(m,z):
    d=mujoco.MjData(m)
    for k in ('qpos','qvel','ctrl','act','qacc_warmstart'):getattr(d,k)[:]=z[k]
    d.time=float(z['time']);mujoco.mj_forward(m,d);return d

def angle(B,base,up):
    q=Rotation.from_matrix(B@base.T).as_quat()
    return float(np.degrees(2*np.arctan2(q[:3]@up,q[3])))

def inspect_for(m):
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    sup={g for g in range(m.ngeom) if (m.geom(g).name or '').startswith(('support_','leap_mount_collision')) or m.geom(g).name=='floor'}
    return hand,sup,cached_contacts(m,hand,sup)

def propose(m,z,turn,lower_mm,normal_weight,translation_mm=(0,0,0),middle_lower_mm=0.):
    d=restore(m,z);p=restore(m,z);cube=m.geom('cube').id
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
    up=z['up'];base=d.qpos.copy();pd=d.ctrl-d.qpos[:16]
    lo,hi,nlo,nhi=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1
    low=(np.ceil(lo[ids])+4-2048)*TICK-pd
    high=(np.floor(hi[ids])-4-2048)*TICK-pd
    hand,support,check=inspect_for(m);tips=[]
    for name in ('mf_tip','th_tip'):
        g=m.geom(name).id
        c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist)
        G=d.geom_xmat[g].reshape(3,3)
        normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
        tips.append((g,G.T@(c.pos-d.geom_xpos[g]),G.T@normal,B.T@(c.pos-C),B.T@normal))
    current=d.qpos[:16].copy();rows=[];records=[];previous=motor(d.ctrl)
    # Small continuous targets prevent one large IK jump. Native command margins
    # account for quantization; no illegal target is clipped into acceptance.
    segments=max(12,int(abs(turn)/1.5),int(abs(lower_mm)))
    for segment in range(1,segments+1):
        f=segment/segments;Br=Rotation.from_rotvec(up*np.radians(turn*f)).as_matrix()@B
        Cr=C+np.asarray(translation_mm)*f/1000
        quat=Rotation.from_matrix(Br).as_quat();cube_pose=np.r_[Cr,quat[3],quat[:3]]
        seed=current[ACTIVE].copy()
        targets=[]
        for role,(g,local,local_n,clocal,cn) in enumerate(tips):
            cl=clocal.copy()
            if role==1:cl[1]+=lower_mm/1000*f
            else:cl[1]+=middle_lower_mm/1000*f
            targets.append((g,local,local_n,Cr+Br@cl,Br@cn))
        def measure(x):
            p.qpos[:]=base;p.qpos[:16]=current;p.qpos[ACTIVE]=x;p.qpos[-7:]=cube_pose
            p.ctrl[:]=p.qpos[:16]+pd;p.qvel[:]=0;mujoco.mj_forward(m,p)
            e=[];ne=[]
            for g,local,local_n,target,target_n in targets:
                G=p.geom_xmat[g].reshape(3,3)
                e.extend((p.geom_xpos[g]+G@local-target)*1000)
                ne.extend((G@local_n-target_n)*normal_weight)
            su,sp=check(p);cube_pen=0.
            for c in p.contact:
                if cube in (int(c.geom1),int(c.geom2)):cube_pen=max(cube_pen,max(0.,-float(c.dist)*1000))
            return np.r_[e,ne,(x-seed)*.3,max(sp.values(),default=0.)*500,max(su.values(),default=0.)*500,max(0,cube_pen-.8)*100]
        local_low=np.maximum(low[ACTIVE],seed-.15);local_high=np.minimum(high[ACTIVE],seed+.15)
        fit=least_squares(measure,seed,bounds=(local_low,local_high),max_nfev=100,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        residual=measure(fit.x);current[ACTIVE]=fit.x
        command=current+pd;q=motor(command)
        if np.any(q<lo) or np.any(q>hi):raise RuntimeError('Native command target rejected')
        err=float(np.max(np.linalg.norm(residual[:6].reshape(2,3),axis=1)))
        su,sp=check(p)
        cube_contacts=[dict(geom=m.geom(int(c.geom2) if int(c.geom1)==cube else int(c.geom1)).name,depth_mm=max(0.,-float(c.dist)*1000)) for c in p.contact if cube in (int(c.geom1),int(c.geom2))]
        records.append(dict(segment=segment,tip_error_mm=err,self_mm=max(sp.values(),default=0.),support_mm=max(su.values(),default=0.),cube_contacts=cube_contacts,goal=q.tolist()))
        if err>.8 or max(sp.values(),default=0.)>1e-6 or max(su.values(),default=0.)>.15:
            return None,records,'IK/contact endpoint rejected'
        count=max(1,int(np.ceil(max(abs(q-previous))/2)))
        rows.extend(np.rint(previous+(q-previous)*j/count).astype(int) for j in range(1,count+1))
        previous=q
    rows.extend(previous.copy() for _ in range(67))
    return np.asarray(rows),records,None

def evaluate(m,z,rows,save=None):
    d=restore(m,z);cube=m.geom('cube').id;body=m.body('cube').id
    hand,support,check=inspect_for(m);lo,hi,nlo,nhi=native(m)
    C=d.qpos[-7:-4].copy();B=d.xmat[body].reshape(3,3).copy();up=z['up']
    worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.)
    stop=None;gap=longest=0.;contacts={};trace=[];hold_B=hold_C=None;drift=shift=0.;body_contact_good=0;hold_n=0
    for index,q in enumerate(rows):
        d.ctrl[:]=ticks_to_sim(q)
        for sub in range(75):
            mujoco.mj_step(m,d)
            su,sp=check(d)
            worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.))
            worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.))
            worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))))
            worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
            inhand=False;external=False;contacts={};body_contacts={}
            CB=d.xmat[body].reshape(3,3)
            for ci,c in enumerate(d.contact):
                a,b=int(c.geom1),int(c.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    inhand|=c.dist<.0004
                    worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000))
                    ff=np.zeros(6);mujoco.mj_contactForce(m,d,ci,ff);name=m.geom(other).name
                    contacts[name]=contacts.get(name,0.)+float(ff[0])
                    height=float(-(CB.T@(c.pos-d.qpos[-7:-4]))[1]*1000)
                    if height<9.1666667 and name in ('mf_tip','th_tip'):body_contacts[name]=body_contacts.get(name,0.)+float(ff[0])
                elif other in support and c.dist<.0004:external=True
            gap=0. if inhand else gap+.001;longest=max(longest,gap)
            if index>=len(rows)-54:
                if hold_B is None:hold_B=CB.copy();hold_C=d.qpos[-7:-4].copy()
                hold_n+=1;drift=max(drift,abs(angle(CB,hold_B,up)));shift=max(shift,float(np.linalg.norm(d.qpos[-7:-4]-hold_C)*1000))
                body_contact_good+=all(body_contacts.get(n,0.)>.2 for n in ('mf_tip','th_tip'))
            if worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or np.linalg.norm(d.qpos[-7:-4]-C)>.08:
                stop='unchanged contact/range/tracking/retention bound';break
        if index%50==0 or index==len(rows)-1 or stop:
            trace.append(dict(sample=index+1,whole_yaw_deg=angle(d.xmat[body].reshape(3,3),z['initial_cube_rotation'],up),contact_N=contacts.copy()))
        if stop:break
    final_B=d.xmat[body].reshape(3,3)
    if save:
        np.savez_compressed(save,qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up,initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    return dict(safety_passed=stop is None,stop=stop,samples_completed=index+1,whole_yaw_deg=angle(final_B,z['initial_cube_rotation'],up),continuation_yaw_deg=angle(final_B,B,up),tilt_deg=float(np.degrees(np.arccos(np.clip(final_B[:,1]@B[:,1],-1,1)))),worst=worst,final_contact_N=contacts,hold_drift_deg=drift,hold_shift_mm=shift,lower_body_opposed_contact_fraction=body_contact_good/max(1,hold_n),hold_evaluated_steps=hold_n,trace=trace)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--turn',type=float,required=True);ap.add_argument('--lower-mm',type=float,default=0);ap.add_argument('--normal-weight',type=float,default=10);a=ap.parse_args()
    name=f'turn_{a.turn:g}_lower_{a.lower_mm:g}_normal_{a.normal_weight:g}';O=OUT/name;O.mkdir(parents=True,exist_ok=True)
    began=time.perf_counter();m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));z=np.load(CP/'closest_state.npz')
    rows,ik,stop=propose(m,z,a.turn,a.lower_mm,a.normal_weight)
    report=dict(status='offline_geometry_and_free_cube_hold_research',turn_target_increment_deg=a.turn,thumb_contact_lowering_mm=a.lower_mm,IK=ik,geometry_stop=stop,hardware_execution_allowed=False,devices_opened=False,physical90_verified=False,checkpoint_sha256=hashlib.sha256((CP/'closest_state.npz').read_bytes()).hexdigest())
    if rows is not None:
        np.savez_compressed(O/'research_suffix.npz',start=motor(z['ctrl']),goals=rows)
        report['dynamic']=evaluate(m,z,rows,O/'end_state.npz')
        report['suffix_goals']=len(rows)
    report['elapsed_seconds']=time.perf_counter()-began
    (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='IK'}),flush=True)
if __name__=='__main__':main()
