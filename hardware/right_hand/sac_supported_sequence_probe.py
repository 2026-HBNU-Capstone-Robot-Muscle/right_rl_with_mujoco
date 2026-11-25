"""Native offline replay including the original sequence's omitted return leg.

Root commands are constrained to the support-aware envelope. This is neither
an original-policy evaluation nor a hardware command generator.
"""
import json
import argparse
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_forward_geometry import measure
ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--all-offsets',action='store_true');args=parser.parse_args()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    model,manifest=build(env);data=mujoco.MjData(model)
    reset=json.loads((ROOT/'sac_supported_reset_result.json').read_text())
    seq=json.loads((ROOT/'sequence_flip_poses.json').read_text())
    perm=list(range(4,16))+list(range(4));targets=[]
    for pose in seq['poses']:
        raw=np.asarray(pose['raw_ticks'])*2*np.pi/4096-np.pi
        target=np.clip(raw[perm],env._safe_lowers,env._safe_uppers);target[[0,4,8]]=np.maximum(target[[0,4,8]],.65)
        targets.append(target)
    obstacle_ids={model.geom(n).id for n in ('support_glass','support_wood')}
    rows=[]
    for candidate in (reset['candidates'] if args.all_offsets else reset['candidates'][:1]):
        mujoco.mj_resetData(model,data);data.qpos[:]=candidate['qpos'];data.ctrl[:]=candidate['ctrl'];mujoco.mj_forward(model,data)
        start_quat=data.qpos[-4:].copy();start_center=data.qpos[-7:-4].copy();stages=[];trace=[]
        for stage in [1,2,3,4,5,6,0]:
            current=data.qpos[:16].copy();target=targets[stage];before=data.qpos[-4:].copy();max_pen=0.;stopped=False
            for t in range(1,211):
                data.ctrl[:]=current+(target-current)*(t/210);mujoco.mj_step(model,data)
                for c in data.contact:
                    if c.geom1 in obstacle_ids or c.geom2 in obstacle_ids:max_pen=max(max_pen,-float(c.dist)*1000)
                if t%10==0:trace.append(dict(stage=stage,qpos=data.qpos.tolist(),ctrl=data.ctrl.tolist()))
                if max_pen>.5 or np.linalg.norm(data.qpos[-7:-4]-start_center)>.1:
                    stopped=True;break
            if not stopped:
                for _ in range(30):mujoco.mj_step(model,data)
            tip,axis,error=measure(start_quat,data.qpos[-4:],xp=np)
            leg_angle=2*np.arccos(np.clip(abs(np.dot(before,data.qpos[-4:])),0,1))
            row=dict(stage=stage,return_leg=stage==0,total_tip_deg=float(np.rad2deg(tip)),off_axis_deg=float(np.rad2deg(axis)),
                     target_error_deg=float(np.rad2deg(error)),this_leg_orientation_change_deg=float(np.rad2deg(leg_angle)),
                     cube_displacement_mm=float(1000*np.linalg.norm(data.qpos[-7:-4]-start_center)),max_support_penetration_mm=max_pen,stopped_early=stopped)
            stages.append(row)
            print(json.dumps(row),flush=True)
            (ROOT/'sac_supported_sequence_probe_progress.json').write_text(json.dumps(dict(offset_mm=candidate['offset_mm'],stages=stages,trace=trace),indent=2)+'\n')
            if max_pen>.5 or row['cube_displacement_mm']>100:break
        result=dict(offset_mm=candidate['offset_mm'],stages=stages,trace=trace);rows.append(result)
        print(json.dumps({k:v for k,v in result.items() if k!='trace'}),flush=True)
    report=dict(status='native_offline_constrained_sequence_probe',motor_commands_sent=0,model=manifest,results=rows,
                original_repeat_count=3,checked_repeat_count=1,
                caveat='Return 6->0 is present in original repeats but absent in checked one-pass. Original direction function is not inferred from stage names. Joint targets here are clipped to configured limits and the support floor; this is not an unchanged original replay.')
    (ROOT/'sac_supported_sequence_probe_result.json').write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
