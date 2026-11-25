"""Small IF support-surface translations after the connected second hold.

Contact Jacobians propose bounded IF postures; free-object native dynamics, not
the kinematic proposal, decides whether the object returns to the old station.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from scipy.optimize import lsq_linear
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,TICK,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'second_hold_IF_support_recentering';source=base/'initial_second_turn_hold_replay/end_state.npz'
    if folder.exists():raise RuntimeError('Preserve prior support translation probes')
    model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;z=dict(np.load(source,allow_pickle=False));d=restore(model,z);guard=Guard(model);active=np.arange(4);sid=np.asarray(SIM_TO_MOTOR_IDS)-1
    ref_file=base/'initial_to_reconfigured_first90/end_state.npz';ref=restore(model,dict(np.load(ref_file,allow_pickle=False)));station=ref.geom_xpos[cube].copy();correction=station-d.geom_xpos[cube]
    jac=np.zeros((3,4));jacr=np.zeros((3,4));weight=0.
    for i,c in enumerate(d.contact):
        a,b=int(c.geom1),int(c.geom2)
        if cube not in (a,b):continue
        other=b if a==cube else a;body=int(model.geom_bodyid[other]);name=model.body(body).name or ''
        if not name.startswith('if_'):continue
        local=np.zeros(6);mujoco.mj_contactForce(model,d,i,local);w=max(0.,float(local[0]));jp=np.zeros((3,model.nv));jr=np.zeros_like(jp);mujoco.mj_jac(model,d,jp,jr,c.pos,body);jac+=w*jp[:,active];jacr+=w*jr[:,active];weight+=w
    assert weight>.2;jac/=weight;jacr/=weight
    folder.mkdir();(folder/'source_probe_second_hold_support_recentering.py').write_bytes(Path(__file__).read_bytes());rows=[]
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.)
    for label,fraction,rotation_weight in (('half_translation',.5,0.),('full_translation',1.,0.),('half_with_orientation_cost',.5,.02),('full_with_orientation_cost',1.,.02)):
        sub=folder/label;sub.mkdir();d=restore(model,z);g=Guard(model);lo=np.maximum(g.qlo[active]-d.qpos[active],-.21);hi=np.minimum(g.qhi[active]-d.qpos[active],.21)
        fit=lsq_linear(np.vstack([jac,rotation_weight*jacr,.0005*np.eye(4)]),np.r_[fraction*correction,np.zeros(7)],bounds=(lo,hi),max_iter=200)
        desired=z['ctrl'].copy();desired[active]+=fit.x;end=motor(desired);start=motor(d.ctrl);previous=start.copy();count=max(1,int(np.ceil(np.max(abs(end-start))/2)))
        if np.any(end<g.lo) or np.any(end>g.hi):raise RuntimeError('Kinematic proposal outside unchanged native command bounds')
        commands=[];trajectory=[d.qpos.copy()];stop=[];samples=supported=certified=0;minimum=float('inf');C0=B0=None;shift=drift=0.;cert=SweepCertificate(model,g.hand,.3)
        for step in range(count+54):
            command=previous.copy();command[sid[active]]=np.rint(start[sid[active]]+(end[sid[active]]-start[sid[active]])*min(step+1,count)/count)
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[active]=ticks_to_sim(command)[active];assert np.array_equal(d.ctrl[4:],z['ctrl'][4:]);commands.append(command.copy());previous=command.copy()
            for _ in range(75):
                mujoco.mj_step(model,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if step>=count:
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if C0 is None:C0=C.copy();B0=B.copy()
                    samples+=1;supported+=sample_model(model,d,g.hand,cube)['supported_sample'];lower,bad=cert.query(d,C,B);minimum=min(minimum,lower);certified+=not bad and lower>.3
                    shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                if stop:break
            trajectory.append(d.qpos.copy())
            if stop:break
        error=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target);center=float(np.linalg.norm(d.geom_xpos[cube]-station)*1000);tilt=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up']);pose=bool(not stop and samples==certified==4050 and supported/4050>.8 and error<5 and tilt<5 and shift<2 and drift<3)
        np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands);np.savez_compressed(sub/'trajectory.npz',qpos=trajectory,control_seconds=.075);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        r=dict(path=sub.relative_to(OUT).as_posix(),case=label,requested_translation_fraction=fraction,orientation_cost_weight_m=rotation_weight,proposed_IF_delta_deg=np.degrees(fit.x).tolist(),linear_predicted_center_error_mm=float(np.linalg.norm(jac@fit.x-correction)*1000),steps=len(commands),stop=stop,hold_samples=samples,upper_certified_samples=certified,upper_minimum_lower_mm=minimum if np.isfinite(minimum) else None,whole_hand_hold_support_fraction=supported/max(1,samples),hold_shift_mm=shift,hold_drift_deg=drift,final180_error_deg=error,tilt_deg=tilt,first_station_error_mm=center,rotation_and_clear_hold_passed=pose,same_first_station_hold_passed=bool(pose and center<2),worst=g.worst,terminal_guard=g.last)
        rows.append(r);save_json(sub/'review.json',r);print(json.dumps({k:v for k,v in r.items() if k not in ('worst','terminal_guard')}),flush=True)
    save_json(folder/'review.json',dict(status='IF_support_jacobian_recentering_probes',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),reference=ref_file.relative_to(OUT).as_posix(),reference_sha256=sha(ref_file),model_provenance=provenance,rows=rows,only_IF_commands_changed=True,other12_controls_bitwise_unchanged=True,model_support_jacobian_is_proposal_not_object_motion_proof=True,all_actual_cube_states_free=True,whole_hand_support_includes_palm=True,original_guards_and_bounds_preserved=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
