"""Validate collision masks and root-angle bounds across side rotation and size uncertainty."""
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_support_separation import SupportSeparation,test_known_boxes
ROOT=Path(__file__).resolve().parent

def main():
    test_known_boxes()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    reset=json.loads((ROOT/'sac_stage0_reset_audit_result.json').read_text())
    scenarios=[]
    for projection,glass in ((31,5),(34,6),(37,7)):
        model,manifest=build(env,projection,glass);data=mujoco.MjData(model);separation=SupportSeparation(model)
        q=np.zeros(model.nq);q[:16]=reset['joint_angles_sim_rad'];q[-7:-4]=reset['cube_center_m'];q[-4:]=reset['cube_quat_wxyz']
        data.qpos[:]=q;mujoco.mj_forward(model,data)
        obstacles=[model.geom(name).id for name in ('support_glass','support_wood')]
        def contacts():
            return [dict(a=model.geom(c.geom1).name,b=model.geom(c.geom2).name,distance_mm=float(c.dist*1000))
                    for c in data.contact if (c.geom1 in obstacles or c.geom2 in obstacles) and c.dist < -1e-5]
        original_hits=contacts()
        if not any('support_proxy' in c['a']+c['b'] for c in original_hits):
            raise RuntimeError('Original housing intersects support, but collision masks fail to report it')
        rows=[];safe_q=q.copy()
        for index,prefix in ((0,'if'),(4,'mf'),(8,'rf')):
            ids=[model.geom(prefix+s+'_visual_support_proxy').id for s in ('_bs','_px')]
            side_values=np.linspace(env._safe_lowers[index+1],env._safe_uppers[index+1],31)
            scan=[]
            for root in np.linspace(.45,.85,81):
                minimum=1.; worst_side=None
                for side in side_values:
                    data.qpos[:]=q;data.qpos[index]=root;data.qpos[index+1]=side;mujoco.mj_forward(model,data)
                    gap=min(separation.lower_bound(data,g,o) for g in ids for o in obstacles)
                    if gap<minimum:minimum=float(gap);worst_side=float(side)
                scan.append(dict(root_rad=float(root),minimum_clearance_mm=minimum*1000,worst_side_rad=worst_side))
            feasible=[r for r in scan if r['minimum_clearance_mm']>=2]
            chosen=feasible[0] if feasible else None
            if chosen:safe_q[index]=max(q[index],chosen['root_rad'])
            rows.append(dict(joint=model.joint(index).name,minimum_root=chosen,scan=scan))
        if any(row['minimum_root'] is None for row in rows):
            scenarios.append(dict(model=manifest,original_support_contacts=original_hits,joints=rows,
                                  candidate_qpos=None,hold_2s=None,status='no_certified_candidate_in_sampled_grid'))
            print(f'{projection}/{glass}: no certified candidate; dynamics skipped',flush=True)
            continue
        data.qpos[:]=safe_q;data.ctrl[:]=safe_q[:16];mujoco.mj_forward(model,data)
        candidate_hits=contacts();min_gap=float('inf');max_drift=0.;max_track=0.;worst_contacts=[]
        for _ in range(round(2/model.opt.timestep)):
            mujoco.mj_step(model,data)
            max_drift=max(max_drift,float(np.linalg.norm(data.qpos[-7:-4]-safe_q[-7:-4])*1000))
            max_track=max(max_track,float(np.max(abs(data.qpos[:16]-safe_q[:16]))))
            hits=contacts()
            if hits and (not worst_contacts or min(c['distance_mm'] for c in hits)<min(c['distance_mm'] for c in worst_contacts)):worst_contacts=hits
        result=dict(model=manifest,original_support_contacts=original_hits,joints=rows,candidate_qpos=safe_q.tolist(),
                    candidate_initial_support_contacts=candidate_hits,hold_2s=dict(max_cube_drift_mm=max_drift,max_joint_error_rad=max_track,worst_support_contacts=worst_contacts,final_qpos=data.qpos.tolist()))
        scenarios.append(result)
        print(json.dumps(dict(projection_mm=projection,glass_mm=glass,original_support_contacts=len(original_hits),
                             roots=[r['minimum_root'] for r in rows],candidate_contacts=len(candidate_hits),hold=result['hold_2s']),indent=2),flush=True)
    (ROOT/'sac_support_clearance_validate_result.json').write_text(json.dumps(dict(status='offline_support_grid_and_hold_validation',motor_commands_sent=0,scenarios=scenarios,
        clearance_method='obstacle-frame containing AABB distance lower bound; zero is inconclusive',
        caveat='Sampled grid and conservative convex CAD proxies; not a continuous proof or a physical measurement. Root-angle bounds apply only to this modeled support pose.'),indent=2)+'\n')

if __name__=='__main__':main()
