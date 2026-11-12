"""Try MF arrival as temporary third support while RF/TH commands stay fixed.

The object is free during execution. This tests a contact-transition primitive,
not final upper space, a full turn or a hardware policy.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--geometry',default='next_cycle/handoff_refinement_20261110/ring_thumb_MF_third_support_geometry/review.json');parser.add_argument('--tag',default='ring_thumb_MF_third_support_arrival');args=parser.parse_args()
    base=OUT/'next_cycle/handoff_refinement_20261110';geometry=(OUT/args.geometry).resolve();folder=base/args.tag
    if not geometry.is_relative_to(OUT.resolve()) or not args.tag.replace('_','').isalnum():raise ValueError('Local geometry and tag required')
    if folder.exists():raise RuntimeError('Preserve earlier MF arrival probes')
    report=json.loads(geometry.read_text());source=OUT/report['source'];assert sha(source)==report['source_sha256']
    z=dict(np.load(source,allow_pickle=False));model,initial,_,_,provenance=assets('active_ccd50');guard=Guard(model);cube=model.geom('cube').id;active=np.arange(4,8);sid=np.asarray(SIM_TO_MOTOR_IDS)-1
    if not np.allclose(model.actuator_gear[:,0],1.) or not np.allclose(model.actuator_biasprm[:,1],-model.actuator_gainprm[:,0]):raise RuntimeError('Unexpected model actuator force contract')
    folder.mkdir();(folder/'source_probe_MF_third_support_arrival.py').write_bytes(Path(__file__).read_bytes());rows=[];selected=[]
    for candidate in report['rows']:
        if not candidate['static_contact_candidate']:continue
        path=OUT/candidate['proposal'];assert sha(path)==candidate['proposal_sha256'];proposal=dict(np.load(path,allow_pickle=False))
        if any(np.max(abs(motor(proposal['ctrl'])-old))<20 for old in selected):continue
        if len(selected)==2:break
        selected.append(motor(proposal['ctrl']));ik=restore(model,z);ik.qpos[:]=proposal['qpos'];ik.qvel[:]=0;mujoco.mj_forward(model,ik)
        contacts=[c for c in ik.contact if {int(c.geom1),int(c.geom2)}=={cube,model.geom('mf_tip').id}]
        if not contacts:continue
        c=min(contacts,key=lambda c:c.dist);other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
        J=np.zeros((3,model.nv));mujoco.mj_jac(model,ik,J,None,c.pos,int(model.geom_bodyid[other]));closing=-J[:,:16].T@n/model.actuator_gainprm[:,0]
        balanced=ik.qpos[:16]+((ik.qfrc_bias-ik.qfrc_passive)[:16]/model.actuator_gainprm[:,0])
        for preload in (0.,.4,.8):
            label=f'seed{candidate["seed"]}_force{preload:g}';sub=folder/label;sub.mkdir();desired=z['ctrl'].copy();desired[active]=balanced[active]+closing[active]*preload;target=motor(desired)
            if np.any(target<guard.lo) or np.any(target>guard.hi):
                row=dict(label=label,skipped='native_command_bounds',temporary_three_finger_hold_passed=False);rows.append(row);save_json(sub/'review.json',row);continue
            d=restore(model,z);g=Guard(model);C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();start=motor(d.ctrl);previous=start.copy();commands=[];stop=[];samples=supported=0;loaded={f:0 for f in ('mf','rf','th')}
            count=max(1,int(np.ceil(np.max(abs(target-start))/2)))
            for step in range(count+54):
                cmd=previous.copy();cmd[sid[active]]=np.rint(start[sid[active]]+(target[sid[active]]-start[sid[active]])*min(step+1,count)/count)
                assert np.max(abs(cmd-previous))<=2+1e-9 and np.all(cmd>=g.lo) and np.all(cmd<=g.hi)
                d.ctrl[active]=ticks_to_sim(cmd)[active];assert np.array_equal(d.ctrl[:4],z['ctrl'][:4]) and np.array_equal(d.ctrl[8:],z['ctrl'][8:])
                commands.append(cmd.copy());previous=cmd.copy()
                for _ in range(75):
                    mujoco.mj_step(model,d);stop=g.check(d)
                    if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if step>=count:
                        load=sample_model(model,d,g.hand,cube);samples+=1;supported+=load['supported_sample']
                        for f in loaded:
                            force=sum(v['normal_N'] for name,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith(f+'_'));loaded[f]+=force>.1
                    if stop:break
                if stop:break
            shift=float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000);drift=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0)
            passed=bool(not stop and samples==4050 and supported/4050>.8 and min(loaded.values())/4050>.8 and shift<2 and drift<3)
            np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
            row=dict(label=label,path=sub.relative_to(OUT).as_posix(),steps=len(commands),stop=stop,samples=samples,whole_hand_support_fraction=supported/max(1,samples),finger_loaded_fractions={f:n/max(1,samples) for f,n in loaded.items()},cube_shift_mm=shift,orientation_change_deg=drift,temporary_three_finger_hold_passed=passed,worst=g.worst,terminal_guard=g.last)
            rows.append(row);save_json(sub/'review.json',row);print(json.dumps(row),flush=True)
    save_json(folder/'review.json',dict(status='MF_temporary_third_support_arrival_probes',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),geometry=geometry.relative_to(OUT).as_posix(),geometry_sha256=sha(geometry),model_provenance=provenance,rows=rows,
        only_MF_commands_changed=True,cube_free_during_arrival=True,original_native_bounds_and_guards_preserved=True,final_upper_space_and_next90_not_certified=True,model_force_not_a450mA_conversion=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
