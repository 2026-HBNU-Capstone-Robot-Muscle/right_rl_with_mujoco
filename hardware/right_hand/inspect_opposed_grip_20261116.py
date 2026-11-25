"""Read saved opposed-contact geometry; no device I/O or model mutation."""
from pathlib import Path
import hashlib,json,sys

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import numpy as np,mujoco,scipy


def main():
    print(json.dumps(dict(mujoco=mujoco.__version__,scipy=scipy.__version__),ensure_ascii=False),flush=True)
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    source=ROOT/'right_hand_rotation_20261105/canonical_body_grip/lower_countergrip_station_force2_span12_equilibrium/seed4/static_proposal.npz'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()})
    z=np.load(source,allow_pickle=False);d=mujoco.MjData(m)
    d.qpos[:16]=z['actual_joint_proposal'];d.qpos[-7:-4]=z['cube_center']
    from scipy.spatial.transform import Rotation
    quat=Rotation.from_matrix(z['cube_rotation']).as_quat();d.qpos[-4:]=np.r_[quat[3],quat[:3]]
    d.ctrl[:]=z['force_balanced_control'];mujoco.mj_forward(m,d)
    from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
    goal=np.zeros(16,dtype=int);goal[np.array(SIM_TO_MOTOR_IDS)-1]=np.rint(d.ctrl/TICK+2048).astype(int)
    contacts=[];cube=m.geom('cube').id
    for c in d.contact:
        if cube not in (int(c.geom1),int(c.geom2)):continue
        other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
        contacts.append(dict(geom=m.geom(other).name,distance_mm=float(c.dist*1000),
            cube_local_mm=((c.pos-z['cube_center'])@z['cube_rotation']*1000).tolist()))
    print(json.dumps(dict(model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),timestep=m.opt.timestep,
        goal_ticks=goal.tolist(),contacts=contacts,devices_opened=False),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
