"""Independent dynamic replay and refreshed geometry for the found MF route."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,ticks_to_sim,motor,save_json,sha
from directed_contact_state import tip_yaw_contact
from whole_hand_support import sample_model


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';parent=base/'robust_mu025_free_MF_standoff_route_v1';folder=base/'reposition_then_second_rotation_20261111/free_standoff_replay_verification'
    if folder.exists():raise RuntimeError('Preserve route replay')
    prior=json.loads((parent/'review.json').read_text());assert not prior['stop'];source=OUT/prior['source'];assert sha(source)==prior['source_sha256'];z=dict(np.load(source,allow_pickle=False));goals=np.load(parent/'research_goals.npz',allow_pickle=False)['goals'];expected=np.load(parent/'end_state.npz',allow_pickle=False)
    m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=.25;d=restore(m,z);g=Guard(m);cube=m.geom('cube').id;tip=m.geom('mf_tip').id;mf_bodies={i for i in range(m.nbody) if (m.body(i).name or '').startswith('mf_')};supported=samples=0;previous=motor(d.ctrl)
    for i,command in enumerate(goals):
        assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi);d.ctrl[4:8]=ticks_to_sim(command)[4:8];previous=command.copy()
        for _ in range(75):
            mujoco.mj_step(m,d);assert not g.check(d),(i,g.last)
            assert not any(c.dist<0 and cube in (int(c.geom1),int(c.geom2)) and int(m.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)]) in mf_bodies for c in d.contact)
            samples+=1;supported+=sample_model(m,d,g.hand,cube)['supported_sample']
    errors={k:float(np.max(abs(getattr(d,k)-expected[k]))) if expected[k].size else 0. for k in BOUND};assert max(errors.values())==0.,errors
    contact=tip_yaw_contact(m,d,cube,tip,initial['up']);assert contact['gap_mm']>7 and contact['lever_mm']>20
    folder.mkdir();np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    r=dict(status='free_standoff_route_independently_replayed',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),parent_goals=(parent/'research_goals.npz').relative_to(OUT).as_posix(),parent_goals_sha256=sha(parent/'research_goals.npz'),parent_review_sha256=sha(parent/'review.json'),steps=len(goals),bound_state_errors=errors,all1ms_samples=samples,supported_samples=supported,no_MF_cube_penetration_contacts=True,actual_tip_contact=contact,actual_standoff_reached=True,geometry_read_from_forwarded_copy_without_modifying_dynamic_state=True,actual_state_resets_after_source=0,original_native_guards_and_joint_limits_preserved=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',r);print(json.dumps(r),flush=True)


if __name__=='__main__':main()
