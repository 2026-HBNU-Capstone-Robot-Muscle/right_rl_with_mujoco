"""Offline inverse extensions after the physically completed638-goal segment."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from hand_motor_mapping import ticks_to_sim,TICK
from new_pair_grasp_plan_20261027 import OPEN_PATH
from screen_loaded_repeat_20261026 import cached_contacts
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/inverse_extension_scout'
def interp(a,b,step=2):
    n=max(1,int(np.ceil(max(abs(b-a))/step)))
    return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)
def main():
    OUT.mkdir(exist_ok=True)
    source=ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/candidate.npz';old=np.load(source)
    opened=np.load(OPEN_PATH)['open_goal'].astype(int);yaw=old['goals'][old['phases']=='stage_7_horizontal_yaw']
    lo,hi=limits();cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,OUT/'geometry.xml',book_projection_mm=-10.)
    correct_model(m,old['start']);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);d=mujoco.MjData(m);d.qpos[-7:]=[0,0,10,1,0,0,0];reports=[]
    specs=[('net04_root2000',0),('net04_root2000_ring128',-128),('swivel128_ring128',-128)]
    for name,ring in specs:
        end=yaw[-1].copy()
        if name.startswith('net'):
            end=np.rint(end+.4*(end-yaw[0])).astype(int);end[11]=max(end[11],2000)
            end[10]=int(yaw[-1][10]) # maintain middle pad reach while swivelling
        else:end[[2,9]]+=128
        changed=np.flatnonzero(end!=yaw[-1])
        if np.any(end[changed]<lo[changed]+.18/TICK) or np.any(end[changed]>hi[changed]-.18/TICK):
            raise RuntimeError('New extension endpoint lacks original tracking-stop margin to native bounds')
        turn=np.vstack([yaw,interp(yaw[-1],end,1)])
        front=old['goals'][np.isin(old['phases'],['stage_5_loaded_grasp','stage_6_grasp_hold'])]
        front_phases=old['phases'][np.isin(old['phases'],['stage_5_loaded_grasp','stage_6_grasp_hold'])]
        # Retain ring clearance during opening, then restore it after all fingers open.
        stages=[*front_phases,*(['stage_7_horizontal_yaw']*len(turn)),*(['stage_8_horizontal_hold']*40)]
        qs=np.vstack([front,turn,np.repeat(end[None,:],40,axis=0)])
        qs[:,13]+=ring;start=old['start'].copy();start[13]+=ring
        held_open=opened.copy();held_open[13]+=ring
        release=np.vstack([interp(qs[-1],held_open),interp(held_open,opened)])
        goals=np.vstack([qs,release,np.repeat(opened[None,:],20,axis=0)])
        phases=np.asarray(stages+['stage_9_loaded_open']*len(release)+['stage_10_remove_hold']*20)
        values=np.vstack([start,goals]);valid=bool(np.all(values>=lo) and np.all(values<=hi) and np.max(abs(np.diff(values,axis=0)))<=2)
        folder=OUT/name;folder.mkdir(exist_ok=True);path=folder/'candidate.npz'
        np.savez_compressed(path,start=start,goals=goals,phases=phases,
            empty_ring_preclear=interp(opened,held_open),
            empty_preposition=old['empty_preposition']+np.eye(1,16,13,dtype=int).ravel()*ring,
            empty_gap_open=old['empty_gap_open']+np.eye(1,16,13,dtype=int).ravel()*ring)
        row=dict(name=name,candidate=path.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            goals=len(goals),yaw_goals=len(turn),extra_yaw_goals=len(turn)-638,extra_goal_ticks=(end-yaw[-1]).tolist(),
            native_and_two_tick_bounds=valid,new_endpoint_has_original_tracking_margin=True,geometry_passed=False)
        if valid:
            worstself=worstsup=0.;failure=None;pair=None
            for i,q in enumerate(goals):
                d.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,d);su,sp=inspect(d)
                worstself=max(worstself,max(sp.values(),default=0.));worstsup=max(worstsup,max(su.values(),default=0.))
                if worstself>1e-6 or worstsup>.15:
                    failure=i;pair=max(sp,key=sp.get) if sp else None;break
            row.update(geometry_passed=failure is None,max_self_mm=worstself,max_support_mm=worstsup,
                first_failure_sample=failure,worst_self_pair=pair)
        reports.append(row);print(json.dumps(row),flush=True)
        (OUT/'geometry_review.json').write_text(json.dumps(dict(cases=reports,devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
