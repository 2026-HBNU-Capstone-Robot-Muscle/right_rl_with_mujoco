"""Model-only identification of the small finish contact; no device imports."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim,sim_to_ticks
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261027/pair_yaw_finish_contact'

def main():
    OUT.mkdir(exist_ok=True);source=np.load(ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2/nominal_grasp_state.npz')
    folder=ROOT/'right_hand_rotation_20261027/pair_free_yaw_optimization_v3';candidate=np.load(folder/'candidate.npz');report=json.loads((folder/'review.json').read_bytes())
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';m,_=build(registry.load('LeapCubeFaceUp',config=cfg),OUT/'scene.xml')
    proof=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());correct_model(m,np.array(proof['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002
    up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world']);m.opt.gravity[:]=-9.81*up;m.geom_friction[m.geom('cube').id,0]=.3
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    d=mujoco.MjData(m);d.qpos[:]=source['qpos'];d.qvel[:]=source['qvel'];d.ctrl[:]=source['ctrl']
    for _ in range(1000):mujoco.mj_step(m,d)
    B=d.xmat[m.body('cube').id].reshape(3,3).copy();C=d.qpos[-7:-4].copy();hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support)
    kind=mujoco.mjtState.mjSTATE_INTEGRATION;size=mujoco.mj_stateSize(m,kind)
    def state():
        a=np.empty(size);mujoco.mj_getState(m,d,a,kind);return a
    def restore(a):mujoco.mj_setState(m,d,a,kind);mujoco.mj_forward(m,d)
    def advance(goals):
        maximum={};steps=0
        for i,q in enumerate(goals,1):
            d.ctrl[:]=ticks_to_sim(q);target=round(i*.075/.002)
            for _ in range(target-steps):
                mujoco.mj_step(m,d);_,sp=inspect(d)
                for pair,depth in sp.items():maximum[pair]=max(maximum.get(pair,0.),depth)
            steps=target
        R=d.xmat[m.body('cube').id].reshape(3,3);delta=Rotation.from_matrix(R@B.T).as_quat();yaw=float(np.degrees(2*np.arctan2(delta[:3]@up,delta[3])))
        return dict(maximum_self_pairs_mm=maximum,final_yaw_deg=yaw,actual_ticks=sim_to_ticks(d.qpos[:16]).tolist())
    result=dict(prefix=advance(candidate['goals']));end=state();np.savez_compressed(OUT/'prefix_end_state.npz',integration=end,base_rotation=B,base_center=C,goal=candidate['goals'][-1])
    result['three_second_hold']=advance([candidate['goals'][-1]]*40)
    rejected=report['steps'][-1]['rejected'][0]['goal'];restore(end);q=candidate['goals'][-1];path=np.rint([q+(np.array(rejected)-q)*i/60 for i in range(1,61)]).astype(int)
    result['rejected_near_90']=advance(np.vstack([path,np.repeat(np.array(rejected)[None,:],40,axis=0)]))
    result.update(hardware_execution_allowed=False,devices_opened=False)
    (OUT/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result),flush=True)
if __name__=='__main__':main()
