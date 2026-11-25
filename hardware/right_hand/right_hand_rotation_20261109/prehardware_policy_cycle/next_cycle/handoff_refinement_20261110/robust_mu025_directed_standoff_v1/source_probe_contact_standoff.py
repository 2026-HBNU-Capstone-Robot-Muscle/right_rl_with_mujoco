"""Reach an outward contact waypoint before closing on the intended cube face."""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from scipy.optimize import least_squares
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg
from directed_contact_state import tip_yaw_contact


def main():
    p=argparse.ArgumentParser();p.add_argument('--geometry',required=True);p.add_argument('--profile',required=True);p.add_argument('--tag',required=True);a=p.parse_args()
    if not a.tag.replace('_','').isalnum():raise ValueError('Local study tag required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/a.tag;geometry=(OUT/a.geometry).resolve()
    if not geometry.is_relative_to(OUT.resolve()) or folder.exists():raise ValueError('New local study required')
    meta=json.loads(geometry.read_text());row=next(r for r in meta['rows'] if r['profile']==a.profile);assert row['static_contact_candidate'];source=OUT/meta['source'];assert sha(source)==meta['source_sha256']
    m,initial,_,_,provenance=assets('active_ccd50');friction=meta.get('sliding_friction_hypothesis');assert friction==.25;m.geom_friction[:,0]=friction
    z=dict(np.load(source,allow_pickle=False));g=Guard(m);cube=m.geom('cube').id;finger=row['finger'];active=np.arange(4*('if','mf','rf','th').index(finger),4*('if','mf','rf','th').index(finger)+4);sid=np.asarray(SIM_TO_MOTOR_IDS)-1;tip=m.geom(finger+'_tip').id
    proposal=OUT/row['proposal'];assert sha(proposal)==row['proposal_sha256'];ik=restore(m,dict(np.load(proposal,allow_pickle=False)));C=ik.geom_xpos[cube].copy();B=ik.geom_xmat[cube].reshape(3,3).copy();axis=int(row['profile'][4]);sign=1 if '+' in row['profile'] else -1;normal=sign*B[:,axis]
    contacts=[c for c in ik.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}];point_target=np.asarray(row['target_point_cube_frame_mm']);contact=min(contacts,key=lambda c:np.linalg.norm((c.pos-C)@B*1000-point_target))
    G=ik.geom_xmat[tip].reshape(3,3).copy();anchor=G.T@(contact.pos-ik.geom_xpos[tip]);local_normal=G.T@(-normal);face_point=point_target.copy();face_point[axis]=sign*27.5
    across=2 if axis==0 else 0;face_point[across]=np.clip(face_point[across],-20,20);face_point[1]=np.clip(face_point[1],-20,20)
    folder.mkdir();(folder/'source_probe_contact_standoff.py').write_bytes(Path(__file__).read_bytes());results=[]
    for distance in (3.,5.,8.):
        target_point=C+B@face_point/1000+normal*distance/1000;branch=restore(m,z)
        def measure(q):
            branch.qpos[:]=z['qpos'];branch.qpos[active]=q;branch.ctrl[:]=z['ctrl'];branch.ctrl[active]=q;mujoco.mj_forward(m,branch)
            R=branch.geom_xmat[tip].reshape(3,3);point=branch.geom_xpos[tip]+R@anchor;normal_error=R@local_normal+normal;support,own=g.contacts(branch)
            cube_pen=0.
            for c in branch.contact:
                if cube not in (int(c.geom1),int(c.geom2)):continue
                other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
                if (m.body(int(m.geom_bodyid[other])).name or '').startswith(finger+'_'):cube_pen=max(cube_pen,-float(c.dist)*1000)
            residual=np.r_[(point-target_point)*1000,normal_error*5,max(own.values(),default=0.)*2000,max(support.values(),default=0.)*1500,max(0.,cube_pen)*2000,(q-ik.qpos[active])*.05]
            return residual,point,normal_error,own,support,cube_pen
        fit=least_squares(lambda q:measure(q)[0],np.clip(ik.qpos[active],g.qlo[active]+1e-7,g.qhi[active]-1e-7),bounds=(g.qlo[active],g.qhi[active]),max_nfev=300,diff_step=1e-5,xtol=1e-9,ftol=1e-9,gtol=1e-9)
        _,point,nerr,own,support,pen=measure(fit.x);position_error=float(np.linalg.norm(point-target_point)*1000);gap=tip_yaw_contact(m,branch,cube,tip,initial['up'])
        desired=z['ctrl'].copy();desired[active]=branch.qpos[active]+((branch.qfrc_bias-branch.qfrc_passive)[active]/m.actuator_gainprm[active,0]);target=motor(desired);sub=folder/f'gap{distance:g}mm';sub.mkdir()
        valid=bool(position_error<.8 and np.linalg.norm(nerr)<.4 and max(own.values(),default=0.)<=1e-6 and max(support.values(),default=0.)<=.15 and pen<=1e-6 and gap['gap_mm']>1 and np.all(target>=g.lo) and np.all(target<=g.hi))
        result=dict(distance_mm=distance,static_waypoint_valid=valid,position_error_mm=position_error,normal_error=float(np.linalg.norm(nerr)),static_tip_gap_mm=gap['gap_mm'],path=sub.relative_to(OUT).as_posix())
        if valid:
            d=restore(m,z);check=Guard(m);start=motor(d.ctrl);previous=start.copy();n=max(1,int(np.ceil(np.max(abs(target-start))/2)));commands=[];stop=[];C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();shift=drift=0.
            for step in range(n+30):
                command=previous.copy();command[sid[active]]=np.rint(start[sid[active]]+(target[sid[active]]-start[sid[active]])*min(step+1,n)/n)
                assert np.max(abs(command-previous))<=2 and np.all(command>=check.lo) and np.all(command<=check.hi)
                d.ctrl[active]=ticks_to_sim(command)[active];commands.append(command.copy());previous=command.copy()
                for _ in range(75):
                    mujoco.mj_step(m,d);stop=check.check(d);shift=max(shift,float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000));drift=max(drift,orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0))
                    if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if shift>2 or drift>3:stop=list(stop)+['standoff_disturbed_cube']
                    if stop:break
                if stop:break
            actual=tip_yaw_contact(m,d,cube,tip,initial['up']);result.update(steps=len(commands),stop=stop,maximum_cube_shift_mm=shift,maximum_cube_drift_deg=drift,actual_tip_contact=actual,actual_standoff_reached=bool(not stop and len(commands)==n+30 and actual['gap_mm']>.5 and actual['lever_mm']>3),worst=check.worst)
            np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        results.append(result);save_json(sub/'review.json',result);print(json.dumps(result),flush=True)
    save_json(folder/'review.json',dict(status='directed_contact_outward_waypoint_trials',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),geometry=geometry.relative_to(OUT).as_posix(),geometry_sha256=sha(geometry),rows=results,sliding_friction_hypothesis=friction,only_declared_finger_commands_changed=finger,original_native_guards_and_joint_limits_preserved=True,standoff_disturbance_is_additional_task_check=True,not_second_rotation_completion=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
