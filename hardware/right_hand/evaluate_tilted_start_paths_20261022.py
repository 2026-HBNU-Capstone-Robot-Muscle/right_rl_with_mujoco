"""Offline replay on the newly modeled tilted palm/root placements."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from sac_forward_geometry import measure,eligible

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'prepared_cube_review_20261022'


def main():
    observed=json.loads((OUT/'placement_review.json').read_text(encoding='utf-8'))
    support=json.loads((OUT/'support_settling_scan.json').read_bytes())
    old=json.loads((ROOT/'full_mapping_pose_review_20261019/inward_quantized/housing_sensitivity/path_OFFLINE_ONLY.json').read_bytes())
    repaired=json.loads((OUT/'projected_path_OFFLINE_ONLY.json').read_bytes())
    original=np.array([old['start_ticks']]+old['goals_id_order']);start=np.array(observed['placement_goal_ticks'])
    original=np.rint(original+np.maximum(0,1-np.arange(len(original))/20)[:,None]*(start-original[0])[None,:]).astype(int)
    paths={'original_intent_static_collision':ticks_to_sim(original),
           'geometric_projection':ticks_to_sim(np.array([repaired['start_ticks']]+repaired['goals_id_order']))}
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.100)
    palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_glass','support_wood')};cube=m.geom('cube').id
    rows=[]
    for name,goals in paths.items():
        for speed in (1,1.5,2):
            for i,reset in enumerate(support['dynamic_holds']):
                data=mujoco.MjData(m);data.qpos[:]=reset['qpos']
                data.qpos[:16]=ticks_to_sim(observed['placement_position_ticks'])
                data.ctrl[:]=goals[0];mujoco.mj_forward(m,data);origin=data.qpos[-7:].copy()
                overlap=tracking=sup=0.;best=streak=0;trace=[]
                controls=np.array([goals[0]]*20+list(goals)+[goals[-1]]*20)
                for k,goal in enumerate(controls):
                    data.ctrl[:]=goal
                    for _ in range(round(.05*speed/m.opt.timestep)):
                        mujoco.mj_step(m,data)
                        for c in data.contact:
                            a,b=int(c.geom1),int(c.geom2)
                            if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:overlap=max(overlap,-c.dist*1000)
                            elif (a in supports or b in supports) and a!=cube and b!=cube:sup=max(sup,-c.dist*1000)
                    tracking=max(tracking,float(np.max(abs(data.qpos[:16]-goal))))
                    tip,axis,error=map(float,measure(origin[-4:],data.qpos[-4:]))
                    shift=float(np.linalg.norm(data.qpos[-7:-4]-origin[:3]))
                    good=bool(eligible(tip,axis,error,shift));streak=streak+1 if good else 0;best=max(best,streak)
                    trace.append(data.qpos.copy())
                row=dict(path=name,time_scale=speed,reset=i,offset_palm_mm=reset['offset_palm_mm'],
                    max_self_overlap_mm=float(overlap),max_support_overlap_mm=float(sup),max_tracking_rad=tracking,
                    final_tip_deg=float(np.degrees(tip)),final_off_axis_deg=float(np.degrees(axis)),
                    final_error_deg=float(np.degrees(error)),final_shift_mm=shift*1000,target_hold_s=best*.05*speed,
                    final_qpos=data.qpos.tolist())
                rows.append(row)
            group=[r for r in rows if r['path']==name and r['time_scale']==speed]
            print(json.dumps(dict(path=name,time_scale=speed,
                tips=[round(r['final_tip_deg'],1) for r in group],
                shifts=[round(r['final_shift_mm'],1) for r in group],
                max_self_mm=max(r['max_self_overlap_mm'] for r in group),
                max_tracking=max(r['max_tracking_rad'] for r in group))),flush=True)
    report=dict(status='offline_tilted_start_path_comparison',paths_max_goal_step_ticks={
        'original':int(np.max(abs(np.diff(original,axis=0))))},model_manifest=manifest,
        seed_registration_verified=False,hardware_execution_allowed=False,motor_commands_sent=0,
        live_sac=False,trials=rows,
        caveat='All cube starts are synthesized from an uncertain historical transform and physical settling, not measured registered poses.')
    (OUT/'tilted_start_path_comparison.json').write_text(json.dumps(report,indent=2))


if __name__=='__main__':main()
