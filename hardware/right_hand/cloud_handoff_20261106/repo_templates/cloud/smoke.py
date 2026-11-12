"""Check exact snapshot bytes and a saved model without opening devices."""
from pathlib import Path
import argparse
import hashlib
import json

ROOT=Path(__file__).resolve().parent.parent


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as source:
        for block in iter(lambda:source.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--verify-only',action='store_true');args=parser.parse_args()
    manifest=json.loads((ROOT/'cloud/upload_manifest.json').read_bytes())
    for entry in manifest['source_manifest']:
        path=ROOT/entry['path'];path.resolve().relative_to(ROOT)
        if digest(path)!=entry['sha256']:raise RuntimeError(f'Snapshot source differs: {entry["path"]}')
    for name,expected in manifest['frozen_source_bindings'].items():
        if digest(ROOT/name)!=expected:raise RuntimeError(f'Frozen source differs: {name}')
    result={'snapshot_files_verified':len(manifest['source_manifest']),
            'frozen_source_bindings_verified':len(manifest['frozen_source_bindings']),
            'devices_opened':0,'full_path_or_physical_success_claimed':False}
    if not args.verify_only:
        import numpy as np
        import mujoco
        if mujoco.__version__!='3.9.0':raise RuntimeError('Use MuJoCo 3.9.0 for the archived binary model')
        folder=ROOT/'right_hand_rotation_20261106/middle_clearance_probe'
        with np.load(folder/'candidate.npz',allow_pickle=False) as candidate:
            goals=candidate['goals'];start=candidate['start']
            if goals.shape!=(1991,16) or np.max(np.abs(np.diff(np.vstack([start,goals]),axis=0)))>2:
                raise RuntimeError('Candidate goal shape or maximum step changed')
        evidence=folder/'evidence';original=json.loads((evidence/'manifest.json').read_bytes())
        if digest(evidence/'model.mjb')!=original['model_sha256']:raise RuntimeError('Saved model differs')
        model=mujoco.MjModel.from_binary_path(str(evidence/'model.mjb'))
        with np.load(evidence/'keyframe_states.npz',allow_pickle=False) as saved:
            state=saved['qpos'][1].copy()
        data=mujoco.MjData(model);data.qpos[:]=state;data.ctrl[:]=state[:model.nu];mujoco.mj_forward(model,data)
        for _ in range(200):mujoco.mj_step(model,data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():raise RuntimeError('Nonfinite saved model state')
        result.update(mujoco=mujoco.__version__,model_nq=model.nq,model_nu=model.nu,
                      saved_model_steps=200,candidate_goals=1991)
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
