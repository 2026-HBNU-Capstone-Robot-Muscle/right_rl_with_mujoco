"""Reuse the finite cube model with the already tested complete reverse path."""
from pathlib import Path
root=Path(__file__).resolve().parent
source=(root/'screen_loaded_yaw_convergence_20261029.py').read_text(encoding='utf-8')
def once(a,b):
    global source
    if source.count(a)!=1:raise RuntimeError('Unexpected source shape: '+a)
    source=source.replace(a,b)
start="    opened=np.load(OPEN_PATH)['open_goal'];z=np.load(GRASP_PATH);grasp=z['goals'][z['phases']=='empty_new_grasp'].copy();grasp[:,7]=np.maximum(grasp[:,7],2048)"
end="    prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());"
i=source.index(start);j=source.index(end)
source=source[:i]+'''    opened=np.load(OPEN_PATH)['open_goal'];assert np.array_equal(data['start'],opened)
    goals=data['goals'].astype(int)
    phase_map={'stage_5_loaded_grasp':'grasp','stage_6_grasp_hold':'grasp_hold','stage_7_horizontal_yaw':'yaw','stage_8_horizontal_hold':'yaw_hold','stage_9_loaded_open':'release','stage_10_remove_hold':'remove_hold'}
    phases=np.asarray([phase_map[str(p)] for p in data['phases']]);grasp=goals[phases=='grasp'];yaw=goals[phases=='yaw']
    lo,hi=limits();values=np.vstack([opened,goals]);assert np.isfinite(values).all() and np.all(values>=lo) and np.all(values<=hi) and max(abs(np.diff(values,axis=0)).ravel())<=2
    np.savez_compressed(out/'loaded_path.npz',start=opened,goals=goals,phases=phases,open_goal=opened,grasp_goal=grasp[-1],yaw_goal=yaw[-1])
'''+source[j:]
once("d.qpos[-7:]=seed['settled_cube_qpos'];d.qvel[:]=0", "d.qpos[-7:]=seed['settled_cube_qpos'];q0=d.qpos[-4:].copy();R0=Rotation.from_quat(q0[[1,2,3,0]]);q1=(R0*Rotation.from_rotvec(np.array([0.,-1.,0.])*np.pi/2)).as_quat();d.qpos[-4:]=q1[[3,0,1,2]];d.qvel[:]=0")
once('turn_angle=None;turn_tilt=None','turn_angle=None;turn_tilt=None;whole_turn=None')
once("R=d.xmat[m.body('cube').id].reshape(3,3);turn_angle=angle(R,yawB);", "R=d.xmat[m.body('cube').id].reshape(3,3);whole_turn=angle(R,B);turn_angle=angle(R,yawB);")
once('yaw_before_release_deg=turn_angle,','placed_to_turn_deg=whole_turn,yaw_before_release_deg=turn_angle,')
once("physics_acceptance_bounds_unchanged=True,status=", "physics_acceptance_bounds_unchanged=True,cube_initial_local_green_axis_turn_deg=90,loaded_route='complete_reverse_joint_cycle',status=")
dest=root/'screen_reverse_loaded_cube_20261029.py'
if dest.exists():raise RuntimeError('Refuse to overwrite reverse model')
dest.write_text(source,encoding='utf-8')
print('Prepared reverse cube model; original physics and safety criteria preserved.')
