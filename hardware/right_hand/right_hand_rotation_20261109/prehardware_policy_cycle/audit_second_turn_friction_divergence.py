"""Locate where the same second-turn sequence diverges under lower friction."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target
from whole_hand_support import sample_model


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'reposition_then_second_rotation_20261111/friction_divergence'
    if folder.exists():raise RuntimeError('Preserve friction diagnosis')
    source=base/'initial_to_reconfigured_first90/end_state.npz';path=base/'initial_second_turn_hold_replay/research_goals.npz';goals=np.load(path,allow_pickle=False)['goals'][3405:-54]
    folder.mkdir();(folder/'source_audit_second_turn_friction_divergence.py').write_bytes(Path(__file__).read_bytes());contexts=[]
    for label,friction in (('nominal',None),('mu025',.25)):
        m,initial,_,_,provenance=assets('active_ccd50')
        if friction is not None:m.geom_friction[:,0]=friction
        contexts.append(dict(label=label,m=m,d=restore(m,dict(np.load(source,allow_pickle=False))),g=Guard(m),cube=m.geom('cube').id,initial=initial,stop=[],trace=[]))
    checkpoints={0:'entry',376:'partial_next',577:'MF_reset',768:'MF_contact_arrival',1268:'MF_pressure',1390:'TH_arrival',1630:'final_teacher'};first=None
    def capture(c,step,label):
        m,d,g,cube=c['m'],c['d'],c['g'],c['cube'];load=sample_model(m,d,g.hand,cube);forces={f:sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(f+'_')) for f in ('if','mf','rf','th')}
        p=folder/c['label'];p.mkdir(exist_ok=True);state=p/f'step{step}_{label}.npz';np.savez_compressed(state,**{k:getattr(d,k) for k in BOUND},time=d.time,up=c['initial']['up'],initial_cube_center=c['initial']['initial_cube_center'],initial_cube_rotation=c['initial']['initial_cube_rotation'])
        row=dict(step=step,phase=label,source=state.relative_to(OUT).as_posix(),source_sha256=sha(state),center_world_m=d.geom_xpos[cube].tolist(),error180_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),upright_yaw_target(c['initial']['initial_cube_rotation'],c['initial']['up'],180)),contact_forces_by_finger_N=forces,loaded_geoms=load['loaded_hand_geoms'],stop=c['stop']);c['trace'].append(row);print(json.dumps(dict(case=c['label'],**row)),flush=True)
    for c in contexts:capture(c,0,'entry')
    for i,command in enumerate(goals,1):
        for c in contexts:
            if c['stop']:continue
            m,d,g=c['m'],c['d'],c['g'];assert np.max(abs(command-motor(d.ctrl)))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi);d.ctrl[:]=ticks_to_sim(command)
            for _ in range(75):
                mujoco.mj_step(m,d);c['stop']=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-c['initial']['initial_cube_center'])>.08:c['stop']=list(c['stop'])+['cube_translation_over80mm']
                if c['stop']:break
        a,b=contexts;distance=float(np.linalg.norm(a['d'].geom_xpos[a['cube']]-b['d'].geom_xpos[b['cube']])*1000);rotation=orientation_error_deg(a['d'].geom_xmat[a['cube']].reshape(3,3),b['d'].geom_xmat[b['cube']].reshape(3,3))
        if first is None and (distance>2 or rotation>3):
            first=dict(after_goal=i,center_difference_mm=distance,orientation_difference_deg=rotation,thresholds_are_diagnostic_not_native_guards=True)
            for c in contexts:capture(c,i,'first_divergence')
        if i in checkpoints:
            for c in contexts:capture(c,i,checkpoints[i])
    rows=[dict(case=c['label'],stop=c['stop'],trace=c['trace'],worst=c['g'].worst) for c in contexts]
    nominal=contexts[0];expected=np.load(base/'initial_second_turn_hold_replay/trajectory.npz',allow_pickle=False)['qpos'][5035];parity=float(np.max(abs(nominal['d'].qpos-expected)));assert parity<1e-7,parity
    r=dict(status='same_entry_second_turn_friction_divergence',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),goals_source=path.relative_to(OUT).as_posix(),goals_source_sha256=sha(path),first_divergence=first,rows=rows,nominal_saved_qpos_difference=parity,original_native_guards_and_joint_limits_preserved=True,low_friction_reference_initialization_not_an_arrival=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',r)


if __name__=='__main__':main()
