"""Privileged-model MF contact correction after a free-object arrival.

Only MF commands change. Exact simulated closest points and contact forces are
assumed; this is not a hardware controller or SAC learning.
"""
from pathlib import Path
from collections import deque
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,TICK,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'ring_thumb_MF_support_feedback';source=base/'ring_thumb_regrasp8_MF_arrival/seed31_force0.8/end_state.npz'
    if folder.exists():raise RuntimeError('Preserve prior MF servo trials')
    z=dict(np.load(source,allow_pickle=False));model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;tip=model.geom('mf_tip').id;active=np.arange(4,8);sid=np.asarray(SIM_TO_MOTOR_IDS)-1
    folder.mkdir();(folder/'source_servo_MF_temporary_support.py').write_bytes(Path(__file__).read_bytes());rows=[]
    def forces(d,g):
        load=sample_model(model,d,g.hand,cube)
        f={prefix:sum(v['normal_N'] for name,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith(prefix+'_')) for prefix in ('mf','rf','th')}
        return load,f
    for desired in (.2,.5):
        sub=folder/f'force{desired:g}';sub.mkdir();d=restore(model,z);g=Guard(model);C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();previous=motor(d.ctrl);start=previous.copy();commands=[];stop=[];window=deque(maxlen=4050);trace=[];passed=False
        for step in range(200):
            load,f=forces(d,g);points=np.zeros(6);gap=float(mujoco.mj_geomDistance(model,d,cube,tip,.2,points));n=points[3:]-points[:3]
            if np.linalg.norm(n)<1e-9:
                c=next((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}),None)
                n=(c.frame[:3]*(1 if int(c.geom1)==cube else -1)) if c is not None else d.geom_xpos[tip]-d.geom_xpos[cube]
            n=n/np.linalg.norm(n);J=np.zeros((3,model.nv));mujoco.mj_jac(model,d,J,None,points[3:],int(model.geom_bodyid[tip]));jac=J[:,active]
            distance=float(np.clip((desired-f['mf'])*.0003,-.0001,.0001));dq=jac.T@np.linalg.solve(jac@jac.T+1e-8*np.eye(3),-n*distance)
            dt=dq/TICK;dt=dt/max(1.,float(np.max(abs(dt)))/2);cmd=previous.copy();cmd[sid[active]]=np.clip(np.rint(previous[sid[active]]+dt),np.ceil(g.lo[sid[active]]),np.floor(g.hi[sid[active]]))
            assert np.max(abs(cmd-previous))<=2+1e-9;d.ctrl[active]=ticks_to_sim(cmd)[active];assert np.array_equal(d.ctrl[:4],z['ctrl'][:4]) and np.array_equal(d.ctrl[8:],z['ctrl'][8:])
            previous=cmd.copy();commands.append(cmd.copy())
            for _ in range(75):
                mujoco.mj_step(model,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                load,f=forces(d,g);shift=float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000);drift=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0)
                window.append((load['supported_sample'],*[f[p]>.1 for p in ('mf','rf','th')],shift,drift))
                if stop:break
            trace.append(dict(step=step+1,forces_N=f,gap_before_step_mm=gap*1000,shift_mm=shift,angle_deg=drift))
            if stop:break
            if len(window)==4050:
                w=np.asarray(window);passed=bool(np.all(np.mean(w[:,:4],axis=0)>.8) and np.max(w[:,4])<2 and np.max(w[:,5])<3)
                if passed:break
        w=np.asarray(window);np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        row=dict(path=sub.relative_to(OUT).as_posix(),steps=len(commands),stop=stop,desired_MF_model_normal_N=desired,temporary_three_finger_hold_passed=passed,window_samples=len(window),window_support_and_MF_RF_TH_fractions=np.mean(w[:,:4],axis=0).tolist(),window_maximum_shift_mm=float(np.max(w[:,4])),window_maximum_angle_deg=float(np.max(w[:,5])),final_model_forces_N=f,trace=trace,worst=g.worst,terminal_guard=g.last)
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps({k:v for k,v in row.items() if k!='trace'}),flush=True)
    save_json(folder/'review.json',dict(status='privileged_model_MF_contact_feedback',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,rows=rows,only_MF_commands_changed=True,original_native_bounds_and_guards_preserved=True,
        exact_model_distances_and_forces_assumed=True,not_a_hardware_sensor_contract=True,final_upper_space_and_next90_not_certified=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
