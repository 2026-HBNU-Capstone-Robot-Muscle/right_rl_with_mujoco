"""Offline examine targets near achieved SAC joint poses; never exports a launcher.

Large requested-goal errors can hide behind simulated force saturation. Preserve
the achieved joint path, cap its feed-forward offset, and test empty AND loaded.
This is reference refinement, not a new SAC success claim.
"""
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_cube_model import set_cube_physics
from sac_forward_geometry import measure,eligible
ROOT=Path(__file__).resolve().parent

def main():
    source=json.loads((ROOT/'sac_forward_support_reference_v1_20k'/'supported_batch_eval_sac_cube55_resets_cube55mm.json').read_text())
    trace=next(r['trace'] for r in source['results'] if r['offset_index']==0 and r['seed']==0)
    ctrl=np.array([r['ctrl'] for r in trace]);actual=np.array([r['qpos'][:16] for r in trace])
    reset=json.loads((ROOT/'sac_cube55_resets.json').read_text())['candidates'][0]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';model,_=build(registry.load('LeapCubeFaceUp',config=cfg))
    set_cube_physics(model,side_m=.055,mass_kg=.1)
    cube=model.geom('cube').id;ctype=model.geom_contype[cube];affinity=model.geom_conaffinity[cube]
    support={model.geom(n).id for n in ('support_glass','support_wood')}
    results=[]
    for compensation in (0.,.04,.07,.10):
        targets=actual+np.clip(ctrl-actual,-compensation,compensation)
        targets=np.clip(targets,model.actuator_ctrlrange[:,0],model.actuator_ctrlrange[:,1]);targets[:,[0,4,8]]=np.maximum(targets[:,[0,4,8]],.65)
        targets[0]=ctrl[0]
        for factor in (1.,1.5):
            steps=int(np.ceil((len(targets)-1)*factor))+1
            goals=np.array([np.interp(np.arange(steps)/factor,np.arange(len(targets)),targets[:,j]) for j in range(16)]).T
            case=dict(compensation_rad=compensation,time_scale=factor,goals_sim_order=goals.tolist(),evaluations=[])
            for loaded in (False,True):
                model.geom_contype[cube]=ctype if loaded else 0;model.geom_conaffinity[cube]=affinity if loaded else 0
                d=mujoco.MjData(model);d.qpos[:]=reset['qpos'];d.ctrl[:]=goals[0];initial=d.qpos[-7:].copy()
                maxerror=0.;penetration=0.;hold=0;longest=0;tracking_by_id=np.zeros(16)
                for goal in np.r_[goals,np.repeat(goals[-1:],30,axis=0)]:
                    d.ctrl[:]=goal
                    for _ in range(round(.05/model.opt.timestep)):
                        mujoco.mj_step(model,d)
                        penetration=max(penetration,max([-float(c.dist)*1000 for c in d.contact if c.geom1 in support or c.geom2 in support]or[0]))
                    error=np.abs(d.qpos[:16]-goal);tracking_by_id=np.maximum(tracking_by_id,error);maxerror=max(maxerror,float(error.max()))
                    tip,axis,err=map(float,measure(initial[-4:],d.qpos[-4:],xp=np));shift=float(np.linalg.norm(d.qpos[-7:-4]-initial[:3]))
                    hold=hold+1 if eligible(tip,axis,err,shift,xp=np) and penetration<=.5 else 0;longest=max(longest,hold)
                case['evaluations'].append(dict(loaded=loaded,max_tracking_rad=maxerror,max_tracking_sim_order=tracking_by_id.tolist(),
                    max_support_penetration_mm=penetration,longest_hold_steps=longest,flip_success=bool(loaded and longest>=23),
                    final_target_error_deg=float(np.degrees(err)),final_tip_deg=float(np.degrees(tip)),
                    finite=bool(np.isfinite(d.qpos).all()),tracking_passed=maxerror<=.18))
            case['passed']=all(e['tracking_passed'] and e['max_support_penetration_mm']<=.5 and e['finite'] for e in case['evaluations']) and case['evaluations'][1]['flip_success']
            results.append(case)
            print(json.dumps({k:v for k,v in case.items() if k!='goals_sim_order'}),flush=True)
    out=dict(status='offline_reference_refinement_only',motor_commands_sent=0,hardware_authorized=False,results=results,
             caveat='Native model only, one source rollout/start. No actor retraining or physical trajectory is certified.')
    (ROOT/'sac_reachable_trace_search_result.json').write_text(json.dumps(out,indent=2)+'\n')

if __name__=='__main__':main()
