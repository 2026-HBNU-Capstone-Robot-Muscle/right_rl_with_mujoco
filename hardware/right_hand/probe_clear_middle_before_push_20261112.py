"""Compare retracting the upper middle links before thumb transfer.

The15mm waypoint probes the receiving corridor, not the final TH/MF/RF station.
All dynamic trials keep a free cube and the original native guards.
"""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from scipy.optimize import lsq_linear
from native_cycle import assets,restore,Guard,BOUND,motor,ticks_to_sim,sha,save_json
from cube_task_frame_20261105 import orientation_error_deg
from whole_hand_support import sample_model

PARENT=ROOT/'right_hand_rotation_20261112/support_first_transfer'
OUT=PARENT/'clear_middle_before_push'

def main():
    OUT.mkdir(exist_ok=False);results=[]
    for name in ('middle_roots_like_index','middle_shape_like_index'):
        source=PARENT/name/'end_state.npz';folder=OUT/name;folder.mkdir()
        m,initial,_,_,provenance=assets('active_ccd50');saved=dict(np.load(source,allow_pickle=False));d=restore(m,saved);g=Guard(m)
        cube=m.geom('cube').id;tip=m.geom('th_tip').id;IF=m.geom('if_bs_collision_1').id;MF=m.geom('mf_bs_collision_1').id
        up=np.asarray(initial['up']);up=up/np.linalg.norm(up)
        e=d.geom_xpos[MF]-d.geom_xpos[IF];e-=up*(e@up);e/=np.linalg.norm(e)
        upper=[i for i in range(m.ngeom) if (m.body(int(m.geom_bodyid[i])).name or '').startswith('mf_') and any(s in (m.geom(i).name or '') for s in ('_md','_ds','mf_tip'))]
        before_C=d.geom_xpos[cube].copy();before_B=d.geom_xmat[cube].reshape(3,3).copy();before_tip=d.geom_xpos[tip].copy()
        to_middle_before=float((d.geom_xpos[MF]-before_C)@e*1000)
        corridor=[];copy=restore(m,saved)
        for mm in (0,5,10,15):
            copy.qpos[:]=saved['qpos'];copy.qpos[-7:-4]+=e*mm/1000;mujoco.mj_forward(m,copy)
            penetrating=[]
            for contact in copy.contact:
                a,b=int(contact.geom1),int(contact.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a
                if other in upper and contact.dist<0:penetrating.append(dict(geom=m.geom(other).name,penetration_mm=-float(contact.dist)*1000))
            corridor.append(dict(translation_hypothesis_mm=mm,middle_upper_penetrating_geoms=penetrating))
        start=motor(d.ctrl);previous=start.copy();th=np.array([15,14,13,12]);TICK=2*np.pi/4096
        rows=[];trace=[];count=supported=middle=0;shift=drift=0.;checks=0;stop=[];jac=np.zeros((3,m.nv))
        for i in range(354):
            command=previous.copy()
            if i<300:
                desired=before_C+e*(.015*min(1.,(i+1)/240.))
                goal_tip=before_tip+(desired-before_C)+.5*(desired-d.geom_xpos[cube])
                velocity=np.clip((goal_tip-d.geom_xpos[tip])*1000,-.3,.3)
                mujoco.mj_jac(m,d,jac,None,d.geom_xpos[tip],int(m.geom_bodyid[tip]))
                A=jac[:,th]*TICK*1000;lo=np.maximum(-2,np.ceil(g.lo[:4])-previous[:4]);hi=np.minimum(2,np.floor(g.hi[:4])-previous[:4])
                delta=lsq_linear(np.vstack([A,.01*np.eye(4)]),np.r_[velocity,np.zeros(4)],bounds=(lo-1e-9,hi+1e-9)).x
                command[:4]=np.clip(previous[:4]+np.rint(delta).astype(int),np.ceil(g.lo[:4]),np.floor(g.hi[:4])).astype(int)
            if i==300:C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
            assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[:]=ticks_to_sim(command);rows.append(command);previous=command.copy()
            for _ in range(75):
                mujoco.mj_step(m,d);checks+=1;stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if i>=300:
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);load=sample_model(m,d,g.hand,cube)
                    count+=1;supported+=load['supported_sample'];middle+=sum(v['normal_N'] for n,v in load['by_geom'].items() if n.startswith(('mf_bs','mf_px')))>.1
                    shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                if stop:break
            if (i+1)%60==0 or stop:
                load=sample_model(m,d,g.hand,cube)
                row=dict(goal=i+1,stop=stop,shift_toward_middle_mm=float((d.geom_xpos[cube]-before_C)@e*1000),
                    total_shift_mm=float(np.linalg.norm(d.geom_xpos[cube]-before_C)*1000),loaded=load['loaded_hand_geoms'])
                trace.append(row);print(json.dumps(dict(case=name,**row)),flush=True)
            if stop:break
        stable=bool(not stop and count==4050 and supported/4050>.8 and shift<2 and drift<3)
        r=dict(case=name,goals=len(rows),stop=stop,stable_supported_hold=stable,hold_samples=count,supported_samples=supported,
            middle_lower_loaded_fraction=middle/max(count,1),requested_translation_mm=15.,
            achieved_toward_middle_mm=float((d.geom_xpos[cube]-before_C)@e*1000),
            hold_shift_mm=shift,hold_drift_deg=drift,corridor_static_diagnostic=corridor,
            corridor_cube_moves_only_in_hypothetical_copies=True,executed_cube_free=True,
            three_finger_final_grip_included=False,final_position_certified=False,source_sha256=sha(source),
            model_provenance=provenance,worst=g.worst,terminal_guard=g.last,trace=trace,
            cube_to_middle_center_projected_distance_before_mm=to_middle_before,
            native_guards_changed=False,devices_opened=0,physical_success=False,hardware_execution_allowed=False)
        np.savez_compressed(folder/'research_goals.npz',start=start,goals=rows)
        np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
        save_json(folder/'review.json',r);results.append(r);save_json(OUT/'review.json',dict(rows=results,devices_opened=0))
        print(json.dumps({k:v for k,v in r.items() if k not in ('corridor_static_diagnostic','model_provenance','worst','terminal_guard','trace')}),flush=True)

if __name__=='__main__':main()
