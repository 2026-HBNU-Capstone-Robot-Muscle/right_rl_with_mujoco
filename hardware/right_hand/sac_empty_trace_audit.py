"""Native empty-hand support/contact check of the exported simulated goal trace."""
import hashlib
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--slow-factor',type=int,default=1);args=parser.parse_args()
    if not 1<=args.slow_factor<=4:raise ValueError('Slow factor must be 1..4')
    path=ROOT/'sac_forward_empty_trace.json';trace=json.loads(path.read_text())
    reset=json.loads((ROOT/'sac_cube55_resets.json').read_text())['candidates'][0]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    model,_=build(registry.load('LeapCubeFaceUp',config=cfg))
    cube=model.geom('cube').id;model.geom_contype[cube]=0;model.geom_conaffinity[cube]=0
    support={model.geom(n).id for n in ('support_glass','support_wood')}
    order=np.r_[np.arange(5,17),np.arange(1,5)]-1
    goals=np.asarray([trace['start_ticks']]+trace['goals_id_order'])[:,order]*(2*np.pi/4096)-np.pi
    if args.slow_factor>1:
        goals=np.r_[goals[:1],np.array([a+(b-a)*j/args.slow_factor for a,b in zip(goals[:-1],goals[1:]) for j in range(1,args.slow_factor+1)])]
    data=mujoco.MjData(model);data.qpos[:]=reset['qpos'];data.ctrl[:]=goals[0]
    records=[];max_penetration=0.;max_tracking=0.
    for i,goal in enumerate(goals):
        data.ctrl[:]=goal
        for _ in range(round(trace['dt_seconds']/model.opt.timestep)):
            mujoco.mj_step(model,data)
            pen=max([max(0,-float(c.dist)*1000) for c in data.contact if c.geom1 in support or c.geom2 in support]or[0])
            max_penetration=max(max_penetration,pen)
        error=np.abs(data.qpos[:16]-goal);max_tracking=max(max_tracking,float(error.max()))
        worst=int(error.argmax())
        records.append(dict(step=i,tracking_max_rad=float(error.max()),worst_motor_id=int(order[worst]+1),support_penetration_mm=pen,
                            actual_rad=float(data.qpos[worst]),goal_rad=float(goal[worst]),
                            contact_pairs=[dict(pair=[model.geom(c.geom1).name,model.geom(c.geom2).name],penetration_mm=-float(c.dist)*1000)
                                           for c in data.contact if c.dist<0],
                            actuator_force=data.actuator_force.tolist()))
    finite=bool(np.isfinite(data.qpos[:16]).all() and np.isfinite(data.qvel[:16]).all())
    out=dict(status='offline_empty_trace_audit',source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),slow_factor=args.slow_factor,
             passed=finite and max_penetration<=.5 and max_tracking<=.18,max_support_penetration_mm=max_penetration,
             max_simulated_tracking_rad=max_tracking,finite=finite,records=records,motor_commands_sent=0,
             actuator_gain0=model.actuator_gainprm[:,0].tolist(),actuator_bias12=model.actuator_biasprm[:,1:3].tolist(),
             worst_samples=sorted(records,key=lambda r:r['tracking_max_rad'],reverse=True)[:3],
             caveat='Native modeled clearance only; actual housings, actuator response and cables require supervised EMPTY validation.')
    stem='sac_forward_empty_trace_audit'+(f'_slow{args.slow_factor}' if args.slow_factor!=1 else '')
    (ROOT/f'{stem}.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:v for k,v in out.items() if k!='records'},indent=2))

if __name__=='__main__':main()
