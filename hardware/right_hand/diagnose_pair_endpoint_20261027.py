"""Read-only model sensitivity at the rejected 69 degree endpoint."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent

def main():
    folder=ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2'
    state=np.load(folder/'nominal_grasp_state.npz')
    data=np.load(ROOT/'right_hand_rotation_20261027/reached_pair_contact_yaw_extension/continued.npz')
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    m,_=build(registry.load('LeapCubeFaceUp',config=cfg),folder/'sensitivity_scene.xml')
    set_cube_physics(m,.055,.1)
    d=mujoco.MjData(m);d.qpos[:]=state['qpos'];d.ctrl[:]=state['ctrl'];mujoco.mj_forward(m,d)
    C=d.qpos[-7:-4].copy();B=d.xmat[m.body('cube').id].reshape(3,3).copy()
    up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world'])
    cube=m.geom('cube').id;tips=[m.geom(n+'_tip').id for n in ('th','mf')]
    q=data['goals'][-1].astype(float);bias=data['bias_ticks'];R=Rotation.from_rotvec(up*np.radians(69)).as_matrix()@B
    def evaluate(value):
        d.qpos[:16]=ticks_to_sim(value+bias);d.qvel[:]=0;d.qpos[-7:-4]=C+up*.004;d.qpos[-4:]=Rotation.from_matrix(R).as_quat()[[3,0,1,2]];mujoco.mj_forward(m,d)
        result=[]
        for tip in tips:
            line=np.zeros(6);result.append(mujoco.mj_geomDistance(m,d,cube,tip,1.,line)*1000)
        return np.array(result)
    base=evaluate(q);rows=[]
    for j in (0,1,2,3,8,9,10,11):
        vals=[]
        for amount in (.25,1.,10.,-10.):
            test=q.copy();test[j]+=amount;vals.append(dict(delta_ticks=amount,gaps_mm=evaluate(test).tolist()))
        rows.append(dict(id=j+1,baseline_gaps_mm=base.tolist(),perturbations=vals))
    print(json.dumps(rows),flush=True)

if __name__=='__main__':main()
