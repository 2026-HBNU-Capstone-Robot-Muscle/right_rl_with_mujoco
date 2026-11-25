"""Build settled, support-aware OFFLINE resets; no hardware pose export."""
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_support_separation import SupportSeparation
ROOT=Path(__file__).resolve().parent

def main():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    model,manifest=build(env);sep=SupportSeparation(model);data=mujoco.MjData(model)
    audit=json.loads((ROOT/'sac_stage0_reset_audit_result.json').read_text())
    obstacles=[model.geom(n).id for n in ('support_glass','support_wood')]
    q=np.r_[audit['joint_angles_sim_rad'],audit['cube_center_m'],audit['cube_quat_wxyz']]
    q[[0,4,8]]=np.maximum(q[[0,4,8]],.65)
    rows=[]
    for offset in ([0,0,0],[2.5,0,0],[5,0,0],[2.5,-2.5,0],[2.5,2.5,0]):
        mujoco.mj_resetData(model,data);data.qpos[:]=q;data.qpos[-7:-4]+=np.array(offset)/1000;data.ctrl[:]=q[:16]
        origin=data.qpos[-7:-4].copy();mujoco.mj_forward(model,data)
        # Initial native settling is part of building a candidate. Its contacts
        # are not counted as policy performance or silently included in reset.
        for _ in range(round(2/model.opt.timestep)):mujoco.mj_step(model,data)
        settled=data.qpos.copy();ctrl=np.clip(settled[:16],env._safe_lowers,env._safe_uppers);ctrl[[0,4,8]]=np.maximum(ctrl[[0,4,8]],.65)
        data.qvel[:]=0;data.ctrl[:]=ctrl;mujoco.mj_forward(model,data)
        initial_pen=max([max(0,-float(c.dist)*1000) for c in data.contact]or[0])
        min_gap=1e3;max_shift=0.;max_pen=0.
        for _ in range(round(2/model.opt.timestep)):
            mujoco.mj_step(model,data)
            min_gap=min(min_gap,1000*min(sep.lower_bound(data,g,o) for g in sep.vertices for o in obstacles))
            max_shift=max(max_shift,float(1000*np.linalg.norm(data.qpos[-7:-4]-settled[-7:-4])))
            max_pen=max(max_pen,max([max(0,-float(c.dist)*1000) for c in data.contact]or[0]))
        passed=initial_pen<=2 and min_gap>=2 and max_shift<=10 and max_pen<=2
        row=dict(offset_mm=offset,qpos=settled.tolist(),ctrl=ctrl.tolist(),initial_penetration_mm=initial_pen,
                 settle_displacement_mm=float(1000*np.linalg.norm(settled[-7:-4]-origin)),
                 hold_2s_min_housing_clearance_mm=min_gap,hold_2s_max_cube_drift_mm=max_shift,hold_2s_max_penetration_mm=max_pen,passed=passed)
        rows.append(row);print(json.dumps({k:v for k,v in row.items() if k not in ('qpos','ctrl')}),flush=True)
    sources=['sac_stage0_reset_audit_result.json','wood_glass_support_measurements.json','sac_wood_glass_model.py']
    report=dict(status='offline_settled_support_reset_candidates',motor_commands_sent=0,root_command_floor_rad=.65,
                model=manifest,mujoco_version=mujoco.__version__,sources_sha256={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in sources},
                candidates=rows,all_passed=all(r['passed'] for r in rows),
                caveat='Derived simulated poses, not measured hardware states. Native settling resolves the recorded reset self-overlap; MJX requires separate validation.')
    (ROOT/'sac_supported_reset_result.json').write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
