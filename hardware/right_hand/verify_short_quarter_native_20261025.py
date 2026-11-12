"""Independent uncertainty replay and EMPTY replay of the new quarter turn."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib,argparse
from pathlib import Path
import numpy as np
import mujoco,mujoco.rollout
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
from hand_motor_mapping import ticks_to_sim
ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261024/original_sequence_clearance'
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_loaded_model'
SOURCE=OUT/'tip_clearance_search_v2/candidate_stop38_tip-16_ring-24.npz'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--candidate',type=Path,default=SOURCE)
    parser.add_argument('--empty-only',action='store_true');parser.add_argument('--output',type=Path,default=OUT/'short_quarter_native_verification.json')
    args=parser.parse_args();candidate=np.load(args.candidate);raw=candidate['goal_ticks'];q=ticks_to_sim(raw)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'short_verification.xml');set_cube_physics(m,.055,.1);cube=m.geom('cube').id
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    kind=mujoco.mjtState.mjSTATE_FULLPHYSICS;substeps=round(.05/m.opt.timestep)
    commands=np.r_[q[1:],np.repeat(q[-1:],80,axis=0)];ctrl=np.repeat(commands,substeps,axis=0)
    old=json.loads((BASE/'reverse_original_native_verification.json').read_bytes())
    refs=[r for r in old['runs'] if r.get('mode')=='moving_path' and r['valid_start']]
    gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();results=[]
    inspect=mujoco.MjData(m)
    def evaluate(state,ss,cc,metadata,empty=False):
        poses=ss[:,1:1+m.nq];target=target_quaternion(state[1+m.nq-4:1+m.nq])
        error=2*np.arccos(np.clip(abs(poses[:,-4:]@target)/np.maximum(np.linalg.norm(poses[:,-4:],axis=1),1e-12),0,1))
        tracking=float(np.max(abs(poses[:,:16]-cc)));goal_error=float(np.degrees(error[-round(4/m.opt.timestep):].max()))
        row=dict(**metadata,maximum_tracking_rad=tracking,hold_maximum_target_error_deg=None if empty else goal_error,
            raw_native_self_clear=False,first_self_contact=None,maximum_support_overlap_mm=0.,hardware_commands_sent=0)
        maximum=0.;supportmax=0.;gap=maxgap=0.;external=False;touch=False;first=None
        for i,s in enumerate(ss):
            mujoco.mj_setState(m,inspect,s,kind);inspect.ctrl[:]=cc[i];mujoco.mj_forward(m,inspect);touch=False
            for c in inspect.contact:
                a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                if c.dist<0 and ha and hb:
                    depth=-float(c.dist)*1000;maximum=max(maximum,depth)
                    if depth>1e-6:first=dict(time_seconds=(i+1)*m.opt.timestep,pair=[m.geom(a).name,m.geom(b).name],depth_mm=depth)
                if c.dist<0 and (a in support and hb or b in support and ha):supportmax=max(supportmax,-float(c.dist)*1000)
                if cube in (a,b) and c.dist<.0003:
                    other=b if a==cube else a
                    if int(m.geom_bodyid[other]) in hand:touch=True
                    if other in support or m.geom(other).name=='floor':external=True
            if first:break
            gap=0. if touch else gap+m.opt.timestep;maxgap=max(maxgap,gap)
        row.update(raw_native_self_clear=first is None,first_self_contact=first,maximum_self_overlap_until_failure_mm=maximum,
            maximum_support_overlap_mm=supportmax,maximum_no_hand_contact_seconds=None if empty else maxgap,
            external_cube_contact=False if empty else external)
        row['complete_model_pass']=bool(first is None and tracking<.12 and supportmax<=.15 and (empty or goal_error<15 and touch and maxgap<=.1 and not external))
        if not empty:
            R0=Rotation.from_quat(state[1+m.nq-4:1+m.nq][[1,2,3,0]]).as_matrix()
            R=Rotation.from_quat(poses[-1,-4:][[1,2,3,0]]).as_matrix()
            row['whole_cube_rotation_deg']=float(np.degrees(Rotation.from_matrix(R@R0.T).magnitude()))
        return row
    with mujoco.rollout.Rollout(nthread=4) as pool:
        datas=[mujoco.MjData(m) for _ in range(4)]
        for g in (() if args.empty_only else (.5,1.,1.5)):
          m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=g;m.actuator_biasprm[:,1:3]*=g
          for f in (.4,.8,1.2):
            m.geom_friction[cube,0]=f;group=[r for r in refs if r['gain_scale']==g and r['friction']==f]
            initial=np.array([r['initial_state'] for r in group]);controls=np.repeat(ctrl[None],len(group),axis=0)
            labels=[dict(mode='moving_path',gain_scale=g,friction=f,offset_mm=r['offset_from_nominal_mm']) for r in group]
            nominal=next((r for r in group if r['offset_from_nominal_mm']==[0,0]),None)
            if nominal is not None:
                initial=np.r_[initial,np.array(nominal['initial_state'])[None]]
                controls=np.r_[controls,np.repeat(q[:1],len(ctrl),axis=0)[None]]
                labels.append(dict(mode='matched_stationary_control',gain_scale=g,friction=f,offset_mm=[0,0]))
            if not len(initial):continue
            states,_=pool.rollout(m,datas,initial,controls)
            for state,ss,cc,label in zip(initial,states,controls,labels):
                row=evaluate(state,ss,cc,label);results.append(row)
                if g==1 and f==.8 and label['mode']=='moving_path' and label['offset_mm']==[0,0]:
                    np.savez_compressed(OUT/'short_quarter_nominal_verified_trace.npz',states=ss,controls=cc,goal_ticks=raw,initial_state=state)
                print('SHORT_VERIFY',json.dumps(row),flush=True)
        for g in (.5,1.,1.5):
            m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=g;m.actuator_biasprm[:,1:3]*=g
            d=mujoco.MjData(m);d.qpos[:16]=q[0];d.ctrl[:]=q[0];d.qpos[-7:]=[10,10,10,1,0,0,0];mujoco.mj_forward(m,d)
            for _ in range(round(1/m.opt.timestep)):mujoco.mj_step(m,d)
            initial=np.empty(mujoco.mj_stateSize(m,kind));mujoco.mj_getState(m,d,initial,kind)
            ss,_=pool.rollout(m,datas,initial[None],ctrl[None]);row=evaluate(initial,ss[0],ctrl,dict(mode='empty_path',gain_scale=g),empty=True)
            results.append(row);print('SHORT_EMPTY',json.dumps(row),flush=True)
    report=dict(candidate_sha256=hashlib.sha256(args.candidate.read_bytes()).hexdigest(),model=manifest,cases=results,
        source_old_initial_states_sha256=hashlib.sha256((BASE/'reverse_original_native_verification.json').read_bytes()).hexdigest(),
        raw_goal_ticks=raw.tolist(),all_command_steps_at_most_two_ticks=bool(np.max(abs(np.diff(raw,axis=0)))<=2),
        physical_placement_registration_verified=False,actual_motor_parameters_identified=False,
        hardware_execution_allowed=False,hardware_commands_sent=0)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')

if __name__=='__main__':main()
