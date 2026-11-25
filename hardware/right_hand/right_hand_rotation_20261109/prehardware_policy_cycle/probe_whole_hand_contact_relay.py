"""Index/palm support -> acquire another contact -> return index support.

Finger contact identities may change. Exact simulator geometry and forces are
privileged teacher inputs, not real sensors. All native guards remain intact.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,TICK,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'whole_hand_contact_relay_20261111'
    if folder.exists():raise RuntimeError('Preserve prior contact-relay study')
    source=base/'whole_hand_support_transfer_20261111/ID8_delta+12/end_state.npz';index_reference=base/'reached_first90_reverse_quarter_probe/first90_space_hold_state.npz'
    z=dict(np.load(source,allow_pickle=False));ref=np.load(index_reference,allow_pickle=False);model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;sid=np.asarray(SIM_TO_MOTOR_IDS)-1
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90.);groups={'mf':np.arange(4,8),'th':np.arange(12,16)}
    geomids={f:{i for i in range(model.ngeom) if (model.body(int(model.geom_bodyid[i])).name or '').startswith(f+'_')} for f in groups}
    enabled={f:[i for i in ids if (int(model.geom_contype[i])&int(model.geom_conaffinity[cube])) or (int(model.geom_contype[cube])&int(model.geom_conaffinity[i]))] for f,ids in geomids.items()}
    folder.mkdir();(folder/'source_probe_whole_hand_contact_relay.py').write_bytes(Path(__file__).read_bytes());rows=[]
    def load_state(d,g):
        load=sample_model(model,d,g.hand,cube);forces={f:sum(v['normal_N'] for n,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(n).id])).name or '').startswith(f+'_')) for f in groups};return load,forces
    def increment(d,f,desired,force):
        contacts=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a
            if other in geomids[f] and c.dist<.0004:contacts.append(c)
        if contacts:
            c=min(contacts,key=lambda c:c.dist);a,b=int(c.geom1),int(c.geom2);other=b if a==cube else a;point=c.pos;n=c.frame[:3]*(1 if a==cube else -1)
        else:
            options=[]
            for other in enabled[f]:
                points=np.zeros(6);distance=mujoco.mj_geomDistance(model,d,cube,other,.2,points);options.append((distance,other,points.copy()))
            distance,other,points=min(options,key=lambda r:r[0]);point=points[3:];n=points[3:]-points[:3]
            if np.linalg.norm(n)<1e-9:n=d.geom_xpos[other]-d.geom_xpos[cube]
        n=n/np.linalg.norm(n);J=np.zeros((3,model.nv));mujoco.mj_jac(model,d,J,None,point,int(model.geom_bodyid[other]));jac=J[:,groups[f]]
        distance=float(np.clip((desired-force)*.0001,-.000025,.000025));dq=jac.T@np.linalg.solve(jac@jac.T+1e-8*np.eye(3),-n*distance);delta=dq/TICK
        return delta/max(1.,float(np.max(abs(delta)))/2)
    for pressure in (.5,1.):
        sub=folder/f'force{pressure:g}';sub.mkdir();d=restore(model,z);g=Guard(model);start=motor(d.ctrl);previous=start.copy();commands=[];trajectory=[d.qpos.copy()];frac={f:np.zeros(4) for f in groups};phase='acquire_contact';streak=0;acquired=False;hold_samples=supported=0;restore_steps=0;trace=[];stop=[]
        for step in range(340):
            load,forces=load_state(d,g)
            if phase=='acquire_contact':
                streak=streak+1 if min(forces.values())>.2 else 0
                if streak>=6:phase='return_index_base';acquired=True
                elif step>=180:break
            command=previous.copy()
            for f,active in groups.items():
                frac[f]+=increment(d,f,pressure,forces[f]);delta=np.clip(np.rint(frac[f]),-2,2);command[sid[active]]=np.clip(previous[sid[active]]+delta,np.ceil(g.lo[sid[active]]),np.floor(g.hi[sid[active]]));frac[f]-=command[sid[active]]-previous[sid[active]]
            if phase=='return_index_base':
                desired=motor(ref['ctrl'])[sid[0]];command[sid[0]]=previous[sid[0]]+np.clip(desired-previous[sid[0]],-2,2);restore_steps+=1
                if abs(command[sid[0]]-desired)<1e-9:phase='hold_relay'
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            changed=np.r_[0,4:8,12:16];d.ctrl[changed]=ticks_to_sim(command)[changed];assert np.array_equal(d.ctrl[1:4],z['ctrl'][1:4]) and np.array_equal(d.ctrl[8:12],z['ctrl'][8:12]);commands.append(command.copy());previous=command.copy()
            for _ in range(75):
                mujoco.mj_step(model,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if phase=='hold_relay':hold_samples+=1;supported+=load_state(d,g)[0]['supported_sample']
                if stop:break
            trajectory.append(d.qpos.copy());load,forces=load_state(d,g);trace.append(dict(step=step+1,phase=phase,finger_normals_N=forces,whole_hand_supported=load['supported_sample'],center_world_m=d.geom_xpos[cube].tolist(),error90_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target)))
            if stop or hold_samples>=4050:break
        passed=bool(not stop and acquired and hold_samples>=4050 and supported/hold_samples>.8)
        np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands);np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        row=dict(path=sub.relative_to(OUT).as_posix(),pressure_objective_N=pressure,steps=len(commands),stop=stop,acquired_MF_TH_contact_event=acquired,final_phase=phase,relay_hold_samples=hold_samples,whole_hand_hold_support_fraction=supported/max(1,hold_samples),final_MF_TH_normals_N=forces,final90_error_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target),intermediate_contact_relay_passed=passed,trace=trace,worst=g.worst,terminal_guard=g.last)
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps({k:v for k,v in row.items() if k!='trace'}),flush=True)
    save_json(folder/'review.json',dict(status='whole_hand_contact_relay_teacher',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),index_reference=index_reference.relative_to(OUT).as_posix(),index_reference_sha256=sha(index_reference),model_provenance=provenance,rows=rows,
        contact_identity_may_change=True,whole_hand_support_includes_palm=True,loaded_contact_event_is_stage_trigger_not_global_invariant=True,privileged_model_geometry_and_forces=True,original_guards_and_bounds_preserved=True,final_upper_space_and_next90_not_certified=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
