"""Free-cube model probe: receiving middle pad first, then thumb translation.

The5mm displacement is a bounded research waypoint, not measured hardware pose
or the final grasp station. No intermediate90deg gate and no guard weakening.
"""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from scipy.optimize import lsq_linear
from native_cycle import assets,restore,Guard,BOUND,ticks_to_sim,motor,sha,save_json
from cube_task_frame_20261105 import orientation_error_deg
from whole_hand_support import sample_model

OUT=ROOT/'right_hand_rotation_20261112/support_first_transfer'
SOURCE=OUT/'middle_roots_like_index/end_state.npz'

def main():
    folder=OUT/'thumb_push_5mm';folder.mkdir(exist_ok=False)
    m,initial,_,_,provenance=assets('active_ccd50');d=restore(m,dict(np.load(SOURCE,allow_pickle=False)));g=Guard(m)
    cube=m.geom('cube').id;tip=m.geom('th_tip').id;IF=m.geom('if_bs_collision_1').id;MF=m.geom('mf_bs_collision_1').id
    up=np.asarray(initial['up']);up/=np.linalg.norm(up)
    direction=d.geom_xpos[MF]-d.geom_xpos[IF];direction-=up*(direction@up);direction/=np.linalg.norm(direction)
    Cbegin=d.geom_xpos[cube].copy();Bbegin=d.geom_xmat[cube].reshape(3,3).copy();Pbegin=d.geom_xpos[tip].copy()
    start=motor(d.ctrl);previous=start.copy();th=np.array([15,14,13,12]);TICK=2*np.pi/4096
    rows=[];trace=[];count=supported=mf_loaded=0;shift=drift=0.;stop=[];jac=np.zeros((3,m.nv))
    for i in range(214):
        command=previous.copy()
        if i<160:
            desired=Cbegin+direction*(.005*min(1.,(i+1)/120.))
            tip_target=Pbegin+(desired-Cbegin)+.5*(desired-d.geom_xpos[cube])
            velocity=np.clip((tip_target-d.geom_xpos[tip])*1000,-.3,.3)
            mujoco.mj_jac(m,d,jac,None,d.geom_xpos[tip],int(m.geom_bodyid[tip]))
            matrix=jac[:,th]*TICK*1000
            lo=np.maximum(-2,g.lo[:4]-previous[:4]);hi=np.minimum(2,g.hi[:4]-previous[:4])
            delta=lsq_linear(np.vstack([matrix,.01*np.eye(4)]),np.r_[velocity,np.zeros(4)],bounds=(lo-1e-9,hi+1e-9)).x
            command[:4]=np.clip(previous[:4]+np.rint(delta).astype(int),g.lo[:4],g.hi[:4]).astype(int)
        if i==160:C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
        assert np.max(abs(command-previous))<=2
        d.ctrl[:]=ticks_to_sim(command);rows.append(command);previous=command.copy()
        for _ in range(75):
            mujoco.mj_step(m,d);stop=g.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=160:
                C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);load=sample_model(m,d,g.hand,cube)
                count+=1;supported+=load['supported_sample']
                mf_loaded+=sum(v['normal_N'] for name,v in load['by_geom'].items() if name.startswith(('mf_bs','mf_px')))>.1
                shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
            if stop:break
        if (i+1)%40==0 or stop:
            load=sample_model(m,d,g.hand,cube)
            row=dict(goal=i+1,stop=stop,cube_shift_toward_middle_mm=float((d.geom_xpos[cube]-Cbegin)@direction*1000),
                cube_total_shift_mm=float(np.linalg.norm(d.geom_xpos[cube]-Cbegin)*1000),loaded=load['loaded_hand_geoms'])
            trace.append(row);print(json.dumps(row),flush=True)
        if stop:break
    displacement=float((d.geom_xpos[cube]-Cbegin)@direction*1000)
    stable=bool(not stop and count==4050 and supported/4050>.8 and shift<2 and drift<3)
    r=dict(status='receiving_middle_pad_then_thumb_translation_probe',goals=len(rows),stop=stop,
        supported_hold_passed=stable,translation_waypoint_mm=5.,achieved_toward_middle_mm=displacement,
        sustained_middleward_translation_observed=stable and displacement>0.,
        waypoint_completion_certified=False,translation_waypoint_error_mm=abs(5.-displacement),
        waypoint_is_not_final_grasp_station=True,
        middle_lower_pad_loaded_hold_fraction=mf_loaded/max(count,1),hold_samples=count,supported_samples=supported,
        hold_shift_mm=shift,hold_drift_deg=drift,orientation_change_diagnostic_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),Bbegin),
        source_sha256=sha(SOURCE),model_provenance=provenance,worst=g.worst,terminal_guard=g.last,trace=trace,
        cube_free_at_all_executed_steps=True,other12_motor_commands_frozen=True,native_guards_unchanged=True,
        final_upper_space_and_left_acquisition_not_verified=True,physical_success=False,devices_opened=0)
    np.savez_compressed(folder/'research_goals.npz',start=start,goals=rows)
    np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
    save_json(folder/'review.json',r)
    print(json.dumps({k:v for k,v in r.items() if k not in ('model_provenance','worst','terminal_guard','trace')}),flush=True)

if __name__=='__main__':main()
