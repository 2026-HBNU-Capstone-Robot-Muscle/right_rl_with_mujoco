"""Conditional square-seating research from uncertain, observed yaw angles.

These independently seated scenarios are NOT full-prefix predictions or
camera-to-robot registration. They cannot authorize physical execution.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
import scout_inverse_regrasp_cycles_20261029 as replay
from propose_inverse_regrasp_ik_20261029 import restore,interp
from new_pair_grasp_plan_20261027 import OPEN_PATH
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint'
OUT=ROOT/'right_hand_rotation_20261029/conditional_inverse_seating'

def main():
    OUT.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(SOURCE/'checkpoint_model.mjb'))
    old=np.load(ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/candidate.npz')
    yaw=old['goals'][old['phases']=='stage_7_horizontal_yaw'];end=yaw[-1].copy()
    ready=yaw[0].copy();opened=np.load(OPEN_PATH)['open_goal'].astype(int)
    up=-m.opt.gravity/np.linalg.norm(m.opt.gravity);cube=m.geom('cube').id
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);lo,hi=limits();simids=np.asarray(SIM_TO_MOTOR_IDS)-1
    native_lo=(lo[simids]-2048)*TICK;native_hi=(hi[simids]-2048)*TICK
    reports=[]
    for angle in (-54.,-48.,-60.):
      for opening in (64,128,192):
        folder=OUT/f'yaw{abs(round(angle))}_open{opening}';folder.mkdir(exist_ok=True)
        d=restore(m,SOURCE/'yaw_hold_checkpoint.npz');baseR=d.xmat[m.body('cube').id].reshape(3,3).copy()
        R=Rotation.from_rotvec(up*np.radians(angle+31.232)).as_matrix()@baseR
        quat=Rotation.from_matrix(R).as_quat();d.qpos[-4:]=quat[[3,0,1,2]]
        center=d.qpos[-7:-4].copy();d.qvel[:]=0
        def depth(h):
            d.qpos[-7:-4]=center+up*h;mujoco.mj_forward(m,d)
            return max((max(0.,-float(c.dist)*1000) for c in d.contact
                if cube in (int(c.geom1),int(c.geom2)) and int(m.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)]) in hand),default=0.)
        low=-.025;high=.04
        for _ in range(28):
            mid=(low+high)/2
            if depth(mid)>1e-6:low=mid
            else:high=mid
        depth(high);d.qvel[:]=0
        mx=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);seed_ok=True
        for k in range(2001):
            if k:mujoco.mj_step(m,d)
            su,sp=inspect(d);mx['self_mm']=max(mx['self_mm'],max(sp.values(),default=0.))
            mx['support_mm']=max(mx['support_mm'],max(su.values(),default=0.))
            mx['tracking_rad']=max(mx['tracking_rad'],float(max(abs(d.ctrl-d.qpos[:16]))))
            mx['range_rad']=max(mx['range_rad'],float(max(np.maximum(native_lo-d.qpos[:16],d.qpos[:16]-native_hi))))
            external=False;inhand=False
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    mx['cube_mm']=max(mx['cube_mm'],max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004
                elif other in support and c.dist<.0004:external=True
            if mx['self_mm']>1e-6 or mx['support_mm']>.15 or mx['cube_mm']>1 or mx['tracking_rad']>.18 or mx['range_rad']>1e-6 or external:
                seed_ok=False;break
        delta=Rotation.from_matrix(d.xmat[m.body('cube').id].reshape(3,3)@baseR.T).as_quat()
        seeded_angle=-31.232+float(np.degrees(2*np.arctan2(delta[:3]@up,delta[3])))
        report=dict(requested_whole_yaw_deg=angle,seeded_whole_yaw_deg=seeded_angle,seed_settling_passed=seed_ok and inhand,
            seed_maxima=mx,seed_vertical_adjustment_mm=high*1000,placement_is_conditional=True,
            robot_world_registration_verified=False,full_prefix_verified=False,devices_opened=False,hardware_execution_allowed=False)
        if report['seed_settling_passed']:
            state=np.empty(mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_FULLPHYSICS))
            mujoco.mj_getState(m,d,state,mujoco.mjtState.mjSTATE_FULLPHYSICS)
            np.savez_compressed(folder/'yaw_hold_checkpoint.npz',state=state,ctrl=d.ctrl,qpos=d.qpos,qvel=d.qvel,
                warmstart=d.qacc_warmstart,state_spec=int(mujoco.mjtState.mjSTATE_FULLPHYSICS))
            rows=[];phases=[];previous=end.copy()
            def move(label,target,hold=0):
                nonlocal previous
                block=interp(previous,target);rows.extend(block);phases.extend([label]*len(block))
                rows.extend([target.copy() for _ in range(hold)]);phases.extend([label]*hold);previous=target.copy()
            q=end.copy();q[0]-=opening;q[1]-=opening//2;q[10]-=opening//2;move('release_tips',q,20)
            q=ready.copy();q[0]-=opening;q[1]-=opening//2;q[10]-=opening//2;move('reset_open',q,20)
            move('regrasp',ready,40);move('hold',ready,40);move('open',opened,20)
            values=np.vstack([end,rows]);valid=bool(np.all(values>=lo) and np.all(values<=hi) and np.max(abs(np.diff(values,axis=0)))<=2)
            if valid:
                replay.CHECK=folder;result=replay.evaluate(m,np.asarray(rows),np.asarray(phases))
                result.pop('approximate_combined_yaw_deg',None)
                report.update(result);report['conditional_whole_yaw_deg']=None if result['checkpoint_additional_yaw_deg'] is None else seeded_angle+result['checkpoint_additional_yaw_deg']
                np.savez_compressed(folder/'extension.npz',start=end,goals=np.asarray(rows),phases=np.asarray(phases))
        reports.append(report);(folder/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        (OUT/'review.json').write_text(json.dumps(dict(cases=reports,conditional_initial_state_only=True,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='phases'}),flush=True)
if __name__=='__main__':main()
