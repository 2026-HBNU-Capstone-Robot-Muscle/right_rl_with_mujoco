"""Compare finger-by-finger entry alignment from an actually translated cube."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target,face_up_error_deg
from whole_hand_support import sample_model


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';root=base/'reposition_then_second_rotation_20261111';folder=root/'finger_alignment_orders_v1'
    if folder.exists():raise RuntimeError('Preserve alignment order trials')
    source=base/'robust_mu025_reposition_to_nominal_entry_v1/end_state.npz';reference=base/'initial_to_reconfigured_first90/end_state.npz'
    groups={'TH':np.arange(0,4),'IF':np.arange(4,8),'MF':np.arange(8,12),'RF':np.arange(12,16)}
    cases=[(f'only_{f}',(f,)) for f in groups]+[("_".join(order),order) for order in (('MF','RF','TH','IF'),('RF','MF','TH','IF'),('TH','MF','RF','IF'),('IF','RF','MF','TH'),('RF','IF','MF','TH'),('MF','IF','RF','TH'))]
    folder.mkdir();(folder/'source_probe_entry_finger_alignment_orders.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for label,order in cases:
        m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=.25;z=dict(np.load(source,allow_pickle=False));d=restore(m,z);g=Guard(m);cube=m.geom('cube').id;ref=restore(m,dict(np.load(reference,allow_pickle=False)));target=motor(ref.ctrl);Cref=ref.geom_xpos[cube].copy();Bref=ref.geom_xmat[cube].reshape(3,3).copy()
        previous=motor(d.ctrl);commands=[];trajectory=[d.qpos.copy()];stages=[];stop=[];samples=supported=0;B0=d.geom_xmat[cube].reshape(3,3).copy();maximum_twist=0.
        def metric():
            B=d.geom_xmat[cube].reshape(3,3)
            return dict(center_error_mm=float(np.linalg.norm(d.geom_xpos[cube]-Cref)*1000),orientation_error_to_entry_deg=orientation_error_deg(B,Bref),first90_error_deg=orientation_error_deg(B,upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)),tilt_deg=face_up_error_deg(B,5,initial['up']),command_error_ticks=float(np.max(abs(motor(d.ctrl)-target))))
        start=metric()
        for finger in order:
            ids=groups[finger];desired=previous.copy();desired[ids]=target[ids];before=previous.copy();n=max(1,int(np.ceil(np.max(abs(desired-before))/2)))
            for k in range(n+2):
                command=np.rint(before+(desired-before)*min(k+1,n)/n)
                assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi)
                d.ctrl[:]=ticks_to_sim(command);previous=command.copy();commands.append(command.copy())
                for _ in range(75):
                    mujoco.mj_step(m,d);stop=g.check(d)
                    if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    samples+=1;supported+=sample_model(m,d,g.hand,cube)['supported_sample'];maximum_twist=max(maximum_twist,orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0))
                    if stop:break
                trajectory.append(d.qpos.copy())
                if stop:break
            stage=dict(finger=finger,after_goal=len(commands),stop=stop,**metric());stages.append(stage);print(json.dumps(dict(case=label,**stage)),flush=True)
            if stop:break
        sub=folder/label;sub.mkdir();np.savez_compressed(sub/'research_goals.npz',start=motor(z['ctrl']),goals=commands);np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075)
        np.savez_compressed(sub/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        result=dict(case=label,order=order,path=sub.relative_to(OUT).as_posix(),steps=len(commands),stop=stop,start=start,stages=stages,final=metric(),supported_samples=supported,all_samples=samples,maximum_orientation_change_from_source_deg=maximum_twist,worst=g.worst,actual_state_resets_after_source=0,hardware_execution_allowed=False,devices_opened=0)
        rows.append(result);save_json(sub/'review.json',result)
    save_json(folder/'review.json',dict(status='finger_by_finger_alignment_order_comparison',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),reference=reference.relative_to(OUT).as_posix(),reference_sha256=sha(reference),rows=rows,sliding_friction_hypothesis=.25,original_native_guards_and_joint_limits_preserved=True,entry_alignment_is_not_second_rotation_completion=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
