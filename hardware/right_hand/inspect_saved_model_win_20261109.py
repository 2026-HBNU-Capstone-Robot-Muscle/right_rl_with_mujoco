"""Inspect an archived model using isolated Windows packages; no devices."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'.runtime/sim-win'))
import numpy as np
import mujoco


def main():
    folder=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/evidence'
    model=mujoco.MjModel.from_binary_path('saved.mjb',assets={'saved.mjb':(folder/'model.mjb').read_bytes()})
    with np.load(folder/'keyframe_states.npz',allow_pickle=False) as data:
        arrays={name:{'shape':list(data[name].shape),'dtype':str(data[name].dtype)} for name in data.files}
        qpos=data['qpos'][1].copy()
    state=mujoco.MjData(model);state.qpos[:]=qpos;state.ctrl[:]=qpos[:model.nu];mujoco.mj_forward(model,state)
    for _ in range(200):mujoco.mj_step(model,state)
    if not np.isfinite(state.qpos).all():raise RuntimeError('Nonfinite state')
    result={'mujoco':mujoco.__version__,'numpy':np.__version__,'nq':model.nq,'nu':model.nu,'ngeom':model.ngeom,
            'joints':[model.joint(i).name for i in range(model.njnt)],
            'geoms':[model.geom(i).name for i in range(model.ngeom)],'saved_arrays':arrays,'finite_steps':200,'devices_opened':False}
    out=ROOT/'right_hand_rotation_20261109/middle_probe_actual_review';out.mkdir(parents=True,exist_ok=True)
    (out/'windows_model_runtime.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
