"""Replay saved commands in native MuJoCo to isolate numerical/contact sensitivity."""
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_cube_model import set_cube_physics
from sac_forward_geometry import measure,eligible

ROOT=Path(__file__).resolve().parent
def main():
    audit=json.loads((ROOT/'sac_stage0_reset_audit_result.json').read_text())
    report=json.loads((ROOT/'sac_stage0_teacher_v2_result.json').read_text())
    cfg=registry.get_default_config('LeapCubeFaceUp'); cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg); model=env.mj_model
    set_cube_physics(model,mass_kg=.100); model.opt.gravity[:]=audit['gravity_m_s2']
    rows=[]
    for row in report['results']:
        if row['success_step'] is None: continue
        for quantize in (False,True):
            data=mujoco.MjData(model); data.qpos[:]=row['trace'][0]['qpos']; data.ctrl[:]=row['trace'][0]['ctrl']
            start=data.qpos.copy(); mujoco.mj_forward(model,data)
            trace=[]; count=0; best=0; first_delta=None
            # Repeat final command for 1 s after the archived acquisition point.
            commands=[r['ctrl'] for r in row['trace'][1:]]+[row['trace'][-1]['ctrl']]*20
            for step,ctrl in enumerate(commands,1):
                ctrl=np.asarray(ctrl)
                if quantize:
                    ticks=np.rint((ctrl+3.14159)*4096/(2*np.pi))
                    ctrl=ticks*(2*np.pi/4096)-3.14159
                data.ctrl[:]=ctrl
                for _ in range(round(.05/model.opt.timestep)): mujoco.mj_step(model,data)
                tip,axis,err=measure(start[-4:],data.qpos[-4:])
                shift=np.linalg.norm(data.qpos[-7:-4]-start[-7:-4])
                count=count+1 if eligible(tip,axis,err,shift) else 0; best=max(best,count)
                trace.append(dict(step=step,tip_deg=float(np.rad2deg(tip)),off_axis_deg=float(np.rad2deg(axis)),
                                  target_error_deg=float(np.rad2deg(err)),center_shift_mm=float(shift*1000),
                                  qpos=data.qpos.tolist()))
                if step==1: first_delta=float(np.max(np.abs(data.qpos-row['trace'][1]['qpos'])))
            result=dict(offset_index=row['offset_index'],seed=row['seed'],quantized=quantize,
                        native_first_step_delta_vs_MJX=first_delta,longest_target_hold_seconds=best*.05,
                        dropped=bool(data.qpos[-5]<-.05),final={k:v for k,v in trace[-1].items() if k!='qpos'},trace=trace)
            rows.append(result); print(json.dumps({k:v for k,v in result.items() if k!='trace'}),flush=True)
    result=dict(status='offline_native_open_loop_crosscheck',motor_commands_sent=0,
                source='sac_stage0_teacher_v2_result.json',ctrl_dt=.05,sim_dt=float(model.opt.timestep),
                trials=rows,caveat='Saved command replay, not a closed-loop teacher evaluation. Quantized conversion is a sensitivity probe, not the hardware driver. Native and MJX solvers can differ.')
    (ROOT/'sac_native_replay_check_result.json').write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':main()
