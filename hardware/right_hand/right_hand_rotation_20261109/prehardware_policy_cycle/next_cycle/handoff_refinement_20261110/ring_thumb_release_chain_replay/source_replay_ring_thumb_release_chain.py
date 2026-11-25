"""Replay actual RF/TH turn, clearance and release segments without state resets.

This starts at a separately initialized RF/TH grip, not the first90 endpoint.
Intermediate motion may change the cube angle; it is not a final hold certificate.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'ring_thumb_release_chain_replay'
    if folder.exists():raise RuntimeError('Preserve prior chain replay')
    parent=base/'ring_thumb_next90';checkpoint=base/'ring_thumb_turn_checkpoint'
    clear=base/'ring_tip_palm_clearance_waypoints/trial1'
    release=base/'ring_thumb_release_after_tip_clear/release1.6_retreat0'
    retreat=base/'ring_thumb_pair_clear_retreat_v2/trial2'
    source=parent/'start_state.npz';z=dict(np.load(source,allow_pickle=False))
    model,initial,_,_,provenance=assets('active_ccd50');d=restore(model,z);g=Guard(model);cube=model.geom('cube').id
    parts=[('original_turn_prefix304',checkpoint/'goals.npz',checkpoint/'state.npz',np.arange(16)),
        ('RF_palm_clear32',clear/'research_goals.npz',clear/'end_state.npz',np.arange(8,12)),
        ('pressure_release41',release/'research_goals.npz',release/'end_state.npz',np.arange(8,16)),
        ('RF_TH_palm_clear_and_RF14_retreat49',retreat/'research_goals.npz',retreat/'end_state.npz',np.arange(8,16))]
    folder.mkdir();(folder/'source_replay_ring_thumb_release_chain.py').write_bytes(Path(__file__).read_bytes())
    C0=d.geom_xpos[cube].copy();commands=[];trajectory=[d.qpos.copy()];seams=[];sources=[source];supported=count=0;previous=motor(d.ctrl)
    for label,goalsfile,endfile,active in parts:
        goals=np.load(goalsfile,allow_pickle=False)['goals'];sources.extend([goalsfile,endfile]);startC=d.geom_xpos[cube].copy();startB=d.geom_xmat[cube].reshape(3,3).copy()
        for command in goals:
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[active]=ticks_to_sim(command)[active];previous=command.copy();commands.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(model,d);assert not g.check(d),g.last
                assert np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])<=.08
                count+=1;supported+=sample_model(model,d,g.hand,cube)['supported_sample']
            trajectory.append(d.qpos.copy())
        expected=np.load(endfile,allow_pickle=False)
        errors={k:float(np.max(abs(getattr(d,k)-expected[k]))) if expected[k].size else 0. for k in BOUND}
        assert max(errors.values())==0.,(label,errors)
        seams.append(dict(segment=label,steps=len(goals),after_goal=len(commands),bound_state_errors=errors,
            segment_center_change_mm=float(np.linalg.norm(d.geom_xpos[cube]-startC)*1000),segment_angle_change_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),startB)))
        print(json.dumps(seams[-1]),flush=True)
    assert len(commands)==426
    np.savez_compressed(folder/'research_goals.npz',start=motor(z['ctrl']),goals=commands)
    np.savez_compressed(folder/'trajectory.npz',qpos=trajectory,control_seconds=.075)
    np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180.)
    report=dict(status='intermediate_chain_replay_passed',steps=len(commands),source=source.relative_to(OUT).as_posix(),source_files_sha256={p.relative_to(OUT).as_posix():sha(p) for p in sources},model_provenance=provenance,
        seams=seams,actual_state_resets_after_start=0,samples=count,whole_hand_supported_samples=supported,whole_hand_support_fraction=supported/count,worst=g.worst,
        final180_error_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target),center_change_from_source_mm=float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000),RF14_upper_room_deg=float(np.degrees(g.qhi[9]-d.qpos[9])),
        source_grip_was_initialized_separately=True,arrival_from_first90_not_certified=True,intermediate_motion_not_final_pose_hold=True,final4050_upper_clearance_not_certified=True,
        full_two_turn_cycle_passed=False,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',report);print(json.dumps({k:report[k] for k in ('status','steps','whole_hand_support_fraction','final180_error_deg','RF14_upper_room_deg')}),flush=True)


if __name__=='__main__':main()
