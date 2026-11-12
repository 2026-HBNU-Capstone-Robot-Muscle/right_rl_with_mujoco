"""Re-execute a saved actual command prefix into an immutable loaded snapshot."""
from pathlib import Path
import argparse
import io
import json
import zipfile
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,sha,save_json,motor,ticks_to_sim
from whole_hand_support import sample_model


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--goals',type=int,required=True);p.add_argument('--tag',required=True);p.add_argument('--require-fingers',default='mf,th');a=p.parse_args()
    required=[f for f in a.require_fingers.split(',') if f]
    if any(f not in ('mf','th') for f in required):raise ValueError('Known optional contact fingers required')
    base=OUT/'next_cycle/handoff_refinement_20261110';parent=(OUT/a.source).resolve();folder=base/a.tag
    if not parent.is_relative_to(OUT.resolve()) or not a.tag.replace('_','').isalnum() or not 1<=a.goals<=800:raise ValueError('Bounded local capture required')
    if folder.exists():raise RuntimeError('Preserve existing loaded checkpoint')
    candidates=[parent/'research_goals.npz',parent/'checkpoint_goals.npz'];captured=None
    for _ in range(8):
        for path in candidates:
            if not path.exists():continue
            try:
                blob=path.read_bytes();snapshot=np.load(io.BytesIO(blob),allow_pickle=False);goals=snapshot['goals']
                if len(goals)>=a.goals:captured=(path,blob,goals[:a.goals].copy());break
            except (OSError,ValueError,zipfile.BadZipFile):continue
        if captured is not None:break
    if captured is None:raise RuntimeError('No complete saved actual command prefix available')
    path,blob,goals=captured;source=parent/'start_state.npz';z=dict(np.load(source,allow_pickle=False));model,initial,_,_,provenance=assets('active_ccd50');d=restore(model,z);g=Guard(model);previous=motor(d.ctrl)
    for cmd in goals:
        assert np.max(abs(cmd-previous))<=2+1e-9 and np.all(cmd>=g.lo) and np.all(cmd<=g.hi);d.ctrl[:]=ticks_to_sim(cmd);previous=cmd.copy()
        for _ in range(75):
            mujoco.mj_step(model,d);assert not g.check(d),g.last
            assert np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])<=.08
    load=sample_model(model,d,g.hand,model.geom('cube').id);forces={f:sum(v['normal_N'] for name,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith(f+'_')) for f in ('mf','th')}
    assert load['supported_sample'] and all(forces[f]>.1 for f in required)
    folder.mkdir();(folder/'source_capture_loaded_entry_checkpoint.py').write_bytes(Path(__file__).read_bytes());(folder/'parent_goals_at_capture.npz').write_bytes(blob)
    np.savez_compressed(folder/'goals.npz',start=motor(z['ctrl']),goals=goals);np.savez_compressed(folder/'state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    save_json(folder/'manifest.json',dict(status='actual_saved_prefix_reexecuted_into_loaded_checkpoint',parent=parent.relative_to(OUT).as_posix(),parent_source=source.relative_to(OUT).as_posix(),parent_source_sha256=sha(source),goals=a.goals,parent_goals_path_at_capture=path.relative_to(OUT).as_posix(),parent_goals_blob_sha256=sha(folder/'parent_goals_at_capture.npz'),goals_sha256=sha(folder/'goals.npz'),state_sha256=sha(folder/'state.npz'),model_provenance=provenance,MF_TH_normals_N=forces,required_contact_fingers=required,whole_hand_supported=load['supported_sample'],model_time=d.time,original_guards_and_bounds_preserved=True,devices_opened=0,new_SAC_training_steps=0,hardware_execution_allowed=False))
    print(json.dumps(dict(goals=a.goals,MF_TH_normals_N=forces,state=(folder/'state.npz').relative_to(OUT).as_posix())),flush=True)


if __name__=='__main__':main()
