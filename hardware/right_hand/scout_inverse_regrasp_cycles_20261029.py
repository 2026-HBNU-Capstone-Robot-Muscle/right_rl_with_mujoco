"""Offline regrasp-cycle hypotheses; uses a frozen finite-physics checkpoint.

This search opens no devices and never approves a physical run. Any useful
checkpoint result still requires a complete replay and independent route review.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from propose_inverse_regrasp_ik_20261029 import restore,interp
from new_pair_grasp_plan_20261027 import OPEN_PATH

ROOT=Path(__file__).resolve().parent
CHECK=ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint'
OUT=ROOT/'right_hand_rotation_20261029/inverse_regrasp_cycles'

def evaluate(m,rows,phases,checkpoint='yaw_hold_checkpoint.npz'):
    d=restore(m,CHECK/checkpoint)
    up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    B=d.xmat[m.body('cube').id].reshape(3,3).copy();C=d.qpos[-7:-4].copy()
    lo,hi=limits();simids=np.asarray(SIM_TO_MOTOR_IDS)-1
    nlo=(lo[simids]-2048)*TICK;nhi=(hi[simids]-2048)*TICK
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);cg=m.geom('cube').id
    worst=dict(max_self_mm=0.,max_support_mm=0.,max_cube_mm=0.,max_tracking_rad=0.,unapproved_range_excess_rad=0.,longest_no_hand_contact_seconds=0.)
    nohand=0.;stop=None;phase_results={};hold_angle=None
    def angle():
        q=Rotation.from_matrix(d.xmat[m.body('cube').id].reshape(3,3)@B.T).as_quat()
        return float(np.degrees(2*np.arctan2(q[:3]@up,q[3])))
    for sample,(phase,q) in enumerate(zip(phases,rows),1):
        d.ctrl[:]=ticks_to_sim(q)
        for _ in range(round(.075/m.opt.timestep)):
            mujoco.mj_step(m,d);su,sp=inspect(d)
            worst['max_self_mm']=max(worst['max_self_mm'],max(sp.values(),default=0.))
            worst['max_support_mm']=max(worst['max_support_mm'],max(su.values(),default=0.))
            worst['max_tracking_rad']=max(worst['max_tracking_rad'],float(np.max(abs(d.qpos[:16]-d.ctrl))))
            worst['unapproved_range_excess_rad']=max(worst['unapproved_range_excess_rad'],float(np.max(np.maximum(nlo-d.qpos[:16],d.qpos[:16]-nhi))))
            inhand=False;external=False
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2)
                if cg in (a,b):
                    other=b if a==cg else a
                    if int(m.geom_bodyid[other]) in hand:
                        worst['max_cube_mm']=max(worst['max_cube_mm'],max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004
                    elif other in support and c.dist<.0004:external=True
            nohand=0. if inhand else nohand+m.opt.timestep
            worst['longest_no_hand_contact_seconds']=max(worst['longest_no_hand_contact_seconds'],nohand)
            if (worst['max_self_mm']>1e-6 or worst['max_support_mm']>.15 or worst['max_cube_mm']>1 or
                worst['max_tracking_rad']>.18 or worst['unapproved_range_excess_rad']>1e-6 or external or nohand>.25):
                stop='original contact/range/tracking/retention bound';break
        phase_results[phase]=dict(additional_yaw_deg=angle(),cube_center=d.qpos[-7:-4].tolist())
        if phase=='hold':hold_angle=angle()
        if stop:break
    return dict(passed=stop is None,stop=stop,last_sample=sample,last_phase=phase,checkpoint_additional_yaw_deg=hold_angle,
        approximate_combined_yaw_deg=None if hold_angle is None else -31.232+hold_angle,final_additional_yaw_deg=angle(),
        center_translation_mm=float(np.linalg.norm(d.qpos[-7:-4]-C)*1000),phases=phase_results,**worst)

def main():
    OUT.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CHECK/'checkpoint_model.mjb'))
    old=np.load(ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/candidate.npz')
    yaw=old['goals'][old['phases']=='stage_7_horizontal_yaw'];ready=yaw[0].copy();end=yaw[-1].copy()
    opened=np.load(OPEN_PATH)['open_goal'].astype(int);reports=[]
    for opening,ring in ((64,0),(128,0),(192,-128),(256,-128)):
        name=f'open{opening}_ring{ring}';folder=OUT/name;folder.mkdir(exist_ok=True)
        rows=[];phases=[];previous=end.copy()
        def move(label,target,hold=0):
            nonlocal previous
            block=interp(previous,target);rows.extend(block);phases.extend([label]*len(block))
            rows.extend([target.copy() for _ in range(hold)]);phases.extend([label]*hold);previous=target.copy()
        q=end.copy();q[13]+=ring;move('ring_clear',q)
        q=previous.copy();q[0]-=opening;q[1]-=opening//2;q[10]-=opening//2;move('release_tips',q,20)
        q=ready.copy();q[13]+=ring;q[0]-=opening;q[1]-=opening//2;q[10]-=opening//2;move('reset_open',q,20)
        q=ready.copy();q[13]+=ring;move('regrasp',q,40)
        for q0 in yaw:
            q=q0.copy();q[13]+=ring;move('turn',q)
        move('hold',previous,40)
        q=opened.copy();q[13]+=ring;move('open',q);move('restore_ring',opened,20)
        rows=np.asarray(rows,dtype=int);phases=np.asarray(phases)
        lo,hi=limits();values=np.vstack([end,rows]);valid=bool(np.all(values>=lo) and np.all(values<=hi) and np.max(abs(np.diff(values,axis=0)))<=2)
        row=dict(name=name,native_and_step_bounds=valid,devices_opened=False,hardware_execution_allowed=False)
        if valid:
            row.update(evaluate(m,rows,phases))
            front=old['goals'][np.isin(old['phases'],['stage_5_loaded_grasp','stage_6_grasp_hold','stage_7_horizontal_yaw'])]
            frontph=old['phases'][np.isin(old['phases'],['stage_5_loaded_grasp','stage_6_grasp_hold','stage_7_horizontal_yaw'])]
            front=np.vstack([front,np.repeat(end[None,:],40,axis=0)]);frontph=np.r_[frontph,['stage_7_horizontal_yaw']*40]
            outph=np.asarray(['stage_8_horizontal_hold' if p=='hold' else 'stage_9_loaded_open' if p in ('open','restore_ring') else 'stage_7_horizontal_yaw' for p in phases])
            goals=np.vstack([front,rows]);path=folder/'candidate.npz'
            np.savez_compressed(path,start=old['start'],goals=goals,phases=np.r_[frontph,outph],empty_preposition=old['empty_preposition'],empty_gap_open=old['empty_gap_open'])
            row.update(candidate=path.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                loaded_goals=len(goals),nominal_motion_with_previous_recovery_s=(len(goals)+1327)*.075)
        reports.append(row);(folder/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
        (OUT/'review.json').write_text(json.dumps(dict(cases=reports,devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in row.items() if k!='phases'}),flush=True)
if __name__=='__main__':main()
