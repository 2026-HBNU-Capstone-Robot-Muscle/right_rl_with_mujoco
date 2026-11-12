"""Derive offline-only trace instrumentation; existing physics is untouched."""
import hashlib,json
from pathlib import Path
R=Path(__file__).resolve().parent
O=R/'right_hand_rotation_20261105/preintegration'
O.mkdir(parents=True,exist_ok=True)
source=R/'trace_raised_cradle_free_20261104.py'
s=source.read_text(encoding='utf-8')
anchor="os.environ.setdefault('MUJOCO_GL','egl')"
assert s.count(anchor)==1
s=s.replace(anchor,anchor+"\nos.environ['JAX_PLATFORMS']='cpu'")
anchor="            if best_state is not None:np.savez_compressed(out/'closest_state.npz',**best_state);mujoco.mj_saveModel(m,str(out/'closest_model.mjb'))"
assert s.count(anchor)==1
s=s.replace(anchor,anchor+"\n            np.savez_compressed(out/'end_state.npz',**{key:getattr(d,key).copy() for key in ('qpos','qvel','ctrl','act','qacc_warmstart')},time=d.time,up=up,initial_cube_rotation=B.copy(),initial_cube_center=C.copy());mujoco.mj_saveModel(m,str(out/'end_model.mjb'))")
target=R/'trace_observed_forward_end_20261105.py'
if target.exists() and target.read_text(encoding='utf-8')!=s:raise RuntimeError('Refuse to overwrite different trace')
target.write_text(s,encoding='utf-8')
manifest=dict(status='offline_trace_instrumentation_only',source=source.name,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),derived=target.name,derived_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),changes=['CPU selection before JAX import','save complete final free-cube state/model'],physics_guards_unchanged=True,hardware_execution_allowed=False)
(O/'trace_derivation.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
print(json.dumps(manifest))
