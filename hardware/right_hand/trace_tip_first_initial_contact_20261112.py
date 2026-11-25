"""Offline trace of the saved empty-hand approach. Opens no devices."""
import json
import os
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261112/tip_first_empty_initial_scope'

def main():
    os.environ.setdefault('MUJOCO_GL','egl')
    import mujoco
    from mujoco_playground import registry
    from fingertip_cad_model_20261025 import build
    from loaded_quarter_model_policy_20261025 import correct_model
    from screen_loaded_repeat_20261026 import cached_contacts
    from hand_motor_mapping import ticks_to_sim
    OUT.mkdir(parents=True,exist_ok=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    folder=ROOT/'tip_first_trial_20261112_173936_071900'
    z=np.load(folder/'empty_recovery_screen/prefix_path.npz',allow_pickle=False)
    m,_=build(env,OUT/'trace_scene.xml',book_projection_mm=-10.)
    correct_model(m,z['start']);m.opt.timestep=.005
    mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support)
    d=mujoco.MjData(m);pair='if_bs_collision_1|th_tip';rows=[]
    phases=['initial_hold']+z['phases'].tolist();goals=np.vstack([z['start'],z['goals']])
    for i,(phase,q) in enumerate(zip(phases,goals)):
        d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0]
        mujoco.mj_forward(m,d);_,pairs=inspect(d)
        rows.append(dict(sample=i,phase=phase,depth_mm=pairs.get(pair,0.),other_pairs={k:v for k,v in pairs.items() if k!=pair}))
    depth=np.array([r['depth_mm'] for r in rows]);first_clear=next((r for r in rows if r['depth_mm']<=1e-6),None)
    result=dict(source=folder.relative_to(ROOT).as_posix(),pair=pair,initial_depth_mm=float(depth[0]),maximum_depth_mm=float(depth.max()),
                maximum_increase_over_initial_mm=float(max(0,depth.max()-depth[0])),maximum_step_increase_mm=float(max(0,np.diff(depth).max())),
                first_clear=first_clear,final_depth_mm=float(depth[-1]),other_pairs_present=any(r['other_pairs'] for r in rows),
                phase_summary=[dict(phase=p,first=next(r['sample'] for r in rows if r['phase']==p),max_mm=max(r['depth_mm'] for r in rows if r['phase']==p),last_mm=[r['depth_mm'] for r in rows if r['phase']==p][-1]) for p in dict.fromkeys(phases)],
                hardware_access=False)
    (OUT/'trace.json').write_text(json.dumps(dict(summary=result,rows=rows),indent=2),encoding='utf-8')
    print(json.dumps(result),flush=True)

if __name__=='__main__':main()
