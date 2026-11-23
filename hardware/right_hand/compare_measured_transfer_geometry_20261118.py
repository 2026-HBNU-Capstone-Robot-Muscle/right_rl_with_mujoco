"""Device-free support proxy comparison at the observed raw hand geometry."""
from pathlib import Path
import json
import mujoco
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for

ROOT=Path(__file__).resolve().parent

def main():
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()});d=mujoco.MjData(m)
    r=json.loads((ROOT/'receiving_waypoint_readonly_20261118_085240_074537/result.json').read_bytes())
    d.qpos[:16]=ticks_to_sim(r['snapshots'][-1]['position']);mujoco.mj_fwdPosition(m,d)
    support,selfdepth=inspect_for(m)[2](d)
    out=dict(support_proxy_depth_mm_at_observed_raw_pose=support,
        hand_self_depth_mm_at_observed_raw_pose=selfdepth,measured_encoder_geometry_only=True,
        physical_support_collision_verified=False)
    (ROOT/'right_hand_rotation_20261118/grip_to_rotation/measured_reference_geometry.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(out))

if __name__=='__main__':main()
