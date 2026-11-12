"""Offline natural-grasp reconstruction and user-taught curvature research.

No motor/camera imports, no executable hardware handoff. All contact/range
criteria are inherited unchanged. The torque cap and friction are hypotheses.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse, hashlib, json, time
from pathlib import Path
import numpy as np
import mujoco
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS, TICK
from research_rotation_body_hold_20261104 import restore, inspect_for, native, motor
from replay_body_hold_from_placement_20261104 import initial
from canonical_grip_dynamics_20261105 import run
from upper_layer_cylinder_clearance_20261105 import precompute, gaps

R=Path(__file__).resolve().parent
O=R/'right_hand_rotation_20261106/taught_upper_clearance'
SOURCE=R/'right_hand_rotation_20261030/natural_cradle_grasp/candidate.npz'
TEACH=R/'manual_grasp_teach_20261106_183247_578628/result.json'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save_json(p,r):p.write_text(json.dumps(r,indent=2,allow_nan=True)+'\n',encoding='utf-8')
def save_state(p,z):np.savez_compressed(p,**z)

def ancestor_instructions():
    for p in (R,*R.parents):
        f=p/'AGENTS.md'
        if f.exists():print('APPLICABLE INSTRUCTIONS',str(f),f.read_text(encoding='utf-8'),flush=True)

def details(m,z):
    d=restore(m,z);hand,_,_=inspect_for(m);cube=m.geom('cube').id
    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
    clearance=gaps(m,d,C,B,precompute(m,hand),True)
    contacts=[]
    for ci,c in enumerate(d.contact):
        a,b=int(c.geom1),int(c.geom2)
        if cube not in (a,b):continue
        other=b if a==cube else a
        f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f)
        contacts.append(dict(geom=m.geom(other).name,normal_N=float(f[0]),depth_mm=max(0.,-float(c.dist)*1000),height_from_center_mm=float(-((c.pos-C)@B[:,1])*1000)))
    su,sp=inspect_for(m)[2](d)
    return dict(goal_ticks=motor(d.ctrl).tolist(),actual_ticks=motor(d.qpos[:16]).tolist(),upper_gaps_mm=sorted(clearance,key=lambda x:x[1]),contacts=contacts,self_contact_mm=sp,support_contact_mm=su)

def build_model(cap):
    from mujoco_playground import registry
    from fingertip_cad_model_20261025 import build
    from loaded_quarter_model_policy_20261025 import correct_model
    from sac_cube_model import set_cube_physics
    from cube_contact_material_policy_20261029 import apply_material
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,O/'scene.xml',book_projection_mm=-10.)
    prior=json.loads((R/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    correct_model(m,np.array(prior['start_ticks']));set_cube_physics(m,.055,.1)
    up=np.array(json.loads((R/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world'])
    m.opt.timestep=.001;m.opt.gravity[:]=-9.81*up
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    hand,_,_=inspect_for(m);material=apply_material(m,.3,hand,m.geom('cube').id)
    m.actuator_forcelimited[:]=1;m.actuator_forcerange[:]=[-cap,cap]
    mujoco.mj_saveModel(m,str(O/'model.mjb'),None)
    save_json(O/'model_assumptions.json',dict(cap_Nm=cap,cap_is_not_measured_450mA_conversion=True,friction=.3,friction_not_measured=True,material=material,book_front_mm=-10.,book_height_mm=20.,source_sha256=sha(Path(__file__))))
    return m,up

def baseline(cap):
    began=time.perf_counter();m,up=build_model(cap);a=np.load(SOURCE);start=a['start'].astype(int);rows=a['goals'][:860].astype(int)
    assert np.all(a['goals'][860:927]==rows[-1])
    rows=np.vstack([rows,np.tile(rows[-1],(134,1))])
    seed=np.array(json.loads((R/'right_hand_rotation_20261030/stable_support_research/natural_recess_hypothesis_seed.json').read_bytes())['cube_qpos'])
    z,placement=initial(m,start,up,seed);save_state(O/'initial.npz',z)
    report=dict(status='natural_grasp_baseline_offline',source=str(SOURCE.relative_to(R)),source_sha256=sha(SOURCE),teaching_source_sha256=sha(TEACH),placement=placement,devices_opened=False,hardware_execution_allowed=False,physical_success=False)
    if placement['passed']:
        np.savez_compressed(O/'baseline_path.npz',start=start,goals=rows)
        report['dynamic']=run(m,z,rows,860,O/'baseline_end.npz',O/'baseline_trajectory.npz',False)
        report['endpoint']=details(m,np.load(O/'baseline_end.npz'))
    report['elapsed_seconds']=time.perf_counter()-began
    save_json(O/'baseline_review.json',report)
    print(json.dumps(report),flush=True)

def proposals(deeper=False):
    """Pinned-cube IK only generates hypotheses; free replay grades them."""
    from scipy.optimize import least_squares
    from dynamic_common_grip_search_20261105 import ramp
    m=mujoco.MjModel.from_binary_path(str(O/'model.mjb'));z=np.load(O/'baseline_end.npz');d=restore(m,z);p=restore(m,z)
    hand,support,check=inspect_for(m);cache=precompute(m,hand);cube=m.geom('cube').id
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();base=d.qpos.copy();pd=d.ctrl-d.qpos[:16]
    active=np.r_[np.arange(4,8),np.arange(12,16)];qbase=d.qpos[:16].copy();start=motor(d.ctrl)
    taught=ticks_to_sim(np.array(json.loads(TEACH.read_bytes())['corrected_position_ticks']))
    lo,hi,nlo,nhi=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1
    lower=(np.ceil(lo[ids])+1-2048)*TICK-pd;upper=(np.floor(hi[ids])-1-2048)*TICK-pd
    tips=[]
    for name in ('mf_tip','th_tip'):
        g=m.geom(name).id;c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist)
        G=d.geom_xmat[g].reshape(3,3);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
        tips.append((g,G.T@(c.pos-d.geom_xpos[g]),G.T@n,B.T@(c.pos-C),n))
    cases=[];began=time.perf_counter()
    for weight in ((.5,1.) if deeper else (.25,.5,1.)):
      for thumb_lower in ((22.,26.) if deeper else (10.,14.,18.)):
        name=f'middle{weight:g}_thumb{thumb_lower:g}';folder=O/name;folder.mkdir(exist_ok=True)
        prior=qbase.copy();prior[4:8]=qbase[4:8]+weight*(taught[4:8]-qbase[4:8]);targets=[]
        for k,(g,local,local_n,clocal,n) in enumerate(tips):
            cl=clocal.copy();cl[1]+=(0. if k==0 else thumb_lower/1000)
            targets.append((g,local,local_n,C+B@cl,n))
        activecache=[item for item in cache if m.body(int(m.geom_bodyid[item[0]])).name.startswith(('mf_','th_'))]
        def measure(x):
            p.qpos[:]=base;p.qpos[active]=x;p.ctrl[:]=p.qpos[:16]+pd;p.qvel[:]=0;mujoco.mj_forward(m,p)
            contact=[];normal=[]
            for g,local,local_n,target,n in targets:
                G=p.geom_xmat[g].reshape(3,3);contact.extend((p.geom_xpos[g]+G@local-target)*1000);normal.extend((G@local_n-n)*2)
            clear=gaps(m,p,C,B,activecache,True);su,sp=check(p)
            depth=max((max(0.,-float(c.dist)*1000) for c in p.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.r_[np.asarray(contact)*4,normal,(x-prior[active])*3,[max(0.,.6-v)*4 for _,v in clear],max(sp.values(),default=0.)*1000,max(su.values(),default=0.)*1000,max(0.,depth-.8)*500]
        seed=np.clip(qbase[active],lower[active]+1e-8,upper[active]-1e-8)
        fit=least_squares(measure,seed,bounds=(lower[active],upper[active]),max_nfev=220,ftol=1e-7,xtol=1e-7,gtol=1e-7,diff_step=1e-5)
        residual=measure(fit.x);goal=motor(p.ctrl);endpoint=details(m,{**{k:getattr(p,k).copy() for k in ('qpos','qvel','ctrl','act','qacc_warmstart')},'time':p.time})
        su,sp=check(p);contact_error=float(max(np.linalg.norm(residual[:6].reshape(2,3)/4,axis=1)));clearance=min(v for _,v in endpoint['upper_gaps_mm'])
        geometry_ok=bool(contact_error<.8 and clearance>.3 and max(sp.values(),default=0.)<=1e-6 and max(su.values(),default=0.)<=.15 and np.all(goal>=lo) and np.all(goal<=hi))
        rows=ramp(start,goal,134);np.savez_compressed(folder/'proposed_path.npz',start=start,goals=rows)
        row=dict(name=name,middle_shape_fraction=weight,thumb_contact_lowering_mm=thumb_lower,geometry_only=dict(passed=geometry_ok,contact_error_mm=contact_error,minimum_upper_gap_mm=clearance,self_mm=max(sp.values(),default=0.),endpoint=endpoint),optimizer_evaluations=fit.nfev,goal=goal.tolist(),pinned_geometry_is_not_dynamic_evidence=True)
        # Replay even an imperfect endpoint to distinguish reach/contact loss.
        row['dynamic']=run(m,z,rows,1,folder/'end.npz',folder/'trajectory.npz',geometry_ok)
        row['free_endpoint']=details(m,np.load(folder/'end.npz'))
        row['passed']=bool(geometry_ok and row['dynamic']['whole_grip_task_passed'])
        cases.append(row);save_json(folder/'review.json',row)
        report=dict(status='taught_curvature_free_cube_search',cases=cases,all_other_finger_goals_unchanged=True,model_sha256=sha(O/'model.mjb'),baseline_state_sha256=sha(O/'baseline_end.npz'),teach_sha256=sha(TEACH),source_sha256=sha(Path(__file__)),native_and_contact_criteria_unchanged=True,devices_opened=False,hardware_execution_allowed=False,physical_success=False,elapsed_seconds=time.perf_counter()-began)
        save_json(O/('deep_proposal_review.json' if deeper else 'proposal_review.json'),report)
        print(json.dumps(dict(name=name,geometry_ok=geometry_ok,contact_error_mm=contact_error,gap_mm=clearance,safety=row['dynamic']['safety_passed'],completed=row['dynamic']['completed_goals'],passed=row['passed'],tail=row['dynamic']['settled_tail'])),flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=('baseline','proposals','deep'),default='baseline');p.add_argument('--cap',type=float,default=.15);a=p.parse_args()
    ancestor_instructions();O.mkdir(parents=True,exist_ok=True)
    if a.mode=='baseline':baseline(a.cap)
    else:proposals(a.mode=='deep')
if __name__=='__main__':main()
