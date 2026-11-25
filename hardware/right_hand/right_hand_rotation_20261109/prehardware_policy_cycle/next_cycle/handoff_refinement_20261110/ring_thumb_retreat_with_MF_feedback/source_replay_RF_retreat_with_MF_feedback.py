"""Retain privileged-model MF feedback throughout an RF/TH retreat primitive."""
from pathlib import Path
from collections import deque
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg
from servo_MF_temporary_support import mf_tick_increment


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'ring_thumb_retreat_with_MF_feedback';source=base/'ring_thumb_MF_support_feedback_v2/force0.5/end_state.npz';goalfile=base/'ring_thumb_three_contact_retreat8/trial1/research_goals.npz'
    if folder.exists():raise RuntimeError('Preserve coordinated MF-feedback retreat trial')
    model,initial,_,_,provenance=assets('active_ccd50');z=dict(np.load(source,allow_pickle=False));path=np.load(goalfile,allow_pickle=False)['goals'];sid=np.asarray(SIM_TO_MOTOR_IDS)-1;mf=np.arange(4,8);moving=np.arange(8,16);cube=model.geom('cube').id;tip=model.geom('mf_tip').id
    folder.mkdir();(folder/'source_replay_RF_retreat_with_MF_feedback.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for desired in (.5,.8):
        sub=folder/f'force{desired:g}';sub.mkdir();d=restore(model,z);g=Guard(model);C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();previous=motor(d.ctrl);commands=[];window=deque(maxlen=4050);fractional=np.zeros(4);stop=[];trace=[]
        for step in range(len(path)+54):
            load=sample_model(model,d,g.hand,cube)
            force=sum(v['normal_N'] for name,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith('mf_'))
            delta,_=mf_tick_increment(model,d,cube,tip,desired,force);fractional+=delta;change=np.clip(np.rint(fractional),-2,2)
            command=previous.copy();command[sid[moving]]=path[min(step,len(path)-1),sid[moving]];command[sid[mf]]=np.clip(previous[sid[mf]]+change,np.ceil(g.lo[sid[mf]]),np.floor(g.hi[sid[mf]]));fractional-=command[sid[mf]]-previous[sid[mf]]
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[mf]=ticks_to_sim(command)[mf];d.ctrl[moving]=ticks_to_sim(command)[moving];assert np.array_equal(d.ctrl[:4],z['ctrl'][:4]);commands.append(command.copy());previous=command.copy()
            for _ in range(75):
                mujoco.mj_step(model,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                load=sample_model(model,d,g.hand,cube);forces={f:sum(v['normal_N'] for name,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith(f+'_')) for f in ('mf','rf','th')}
                shift=float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000);drift=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0)
                window.append((load['supported_sample'],*[forces[f]>.1 for f in ('mf','rf','th')],shift,drift))
                if stop:break
            trace.append(dict(step=step+1,forces_N=forces,shift_mm=shift,angle_deg=drift))
            if stop:break
        w=np.asarray(window);fractions=np.mean(w[:,:4],axis=0);passed=bool(not stop and len(commands)==len(path)+54 and len(window)==4050 and min(fractions)>.8 and np.max(w[:,4])<2 and np.max(w[:,5])<3)
        np.savez_compressed(sub/'research_goals.npz',start=motor(z['ctrl']),goals=commands);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        row=dict(path=sub.relative_to(OUT).as_posix(),steps=len(commands),stop=stop,desired_MF_model_normal_N=desired,coordinated_three_finger_retreat_passed=passed,window_samples=len(window),window_support_and_MF_RF_TH_fractions=fractions.tolist(),window_maximum_shift_mm=float(np.max(w[:,4])),window_maximum_angle_deg=float(np.max(w[:,5])),RF14_upper_room_deg=float(np.degrees(g.qhi[9]-d.qpos[9])),trace=trace,worst=g.worst,terminal_guard=g.last)
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps({k:v for k,v in row.items() if k!='trace'}),flush=True)
    save_json(folder/'review.json',dict(status='RF_TH_retreat_with_simultaneous_MF_feedback',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),retreat_goals=goalfile.relative_to(OUT).as_posix(),retreat_goals_sha256=sha(goalfile),feedback_helper_sha256=sha(OUT/'servo_MF_temporary_support.py'),model_provenance=provenance,rows=rows,
        original_native_bounds_and_guards_preserved=True,RF_TH_command_subsequence_preserved=True,IF_commands_bitwise_unchanged=True,actual_state_resets_after_start=0,privileged_model_distances_and_forces_assumed=True,not_a_hardware_sensor_contract=True,final_upper_space_and_next90_not_certified=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
