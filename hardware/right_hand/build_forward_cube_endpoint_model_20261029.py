from pathlib import Path
root=Path(__file__).resolve().parent;source=(root/'screen_loaded_yaw_convergence_20261029.py').read_text(encoding='utf-8')
a='yawB=None;maxself=';b='cube_initial=d.qpos[-7:].copy();held_cube=None;yawB=None;maxself='
assert source.count(a)==1;source=source.replace(a,b)
a="R=d.xmat[m.body('cube').id].reshape(3,3);turn_angle=angle(R,yawB);";b="R=d.xmat[m.body('cube').id].reshape(3,3);held_cube=d.qpos[-7:].copy();turn_angle=angle(R,yawB);"
assert source.count(a)==1;source=source.replace(a,b)
a='row=dict(simulation_timestep_s=args.dt,';b="row=dict(initial_cube_qpos=cube_initial.tolist(),held_cube_qpos=held_cube.tolist() if held_cube is not None else None,final_cube_qpos=d.qpos[-7:].tolist(),simulation_timestep_s=args.dt,"
assert source.count(a)==1;source=source.replace(a,b)
dest=root/'screen_forward_cube_endpoint_20261029.py'
if dest.exists():raise RuntimeError('Refuse to overwrite endpoint model')
dest.write_text(source,encoding='utf-8')
