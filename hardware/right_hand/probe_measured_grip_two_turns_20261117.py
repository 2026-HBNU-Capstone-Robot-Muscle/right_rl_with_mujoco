"""Free-cube dynamics of two small turns from the measured grip's model hypothesis."""
from pathlib import Path
import hashlib,json
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim,sim_to_ticks
from research_rotation_body_hold_20261104 import restore

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261117/user_measured_grip'
MODEL=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
INITIAL=BASE/'model_hold_probe/end.npz'
PATH=BASE/'turn_connection_probe/research_waypoints.npz'


def main():
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    d=restore(m,dict(np.load(INITIAL,allow_pickle=False)))
    cube=m.geom('cube').id;palm=m.body('palm').id
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
    axis=d.xmat[palm].reshape(3,3)[:,2].copy()
    source=np.load(PATH,allow_pickle=False)['goals']
    if len(source)<10:raise RuntimeError('Ten model waypoints required')
    previous=sim_to_ticks(d.ctrl);rows=[];stages=[]
    for i,target in enumerate(source[:10]):
        count=int(np.ceil(np.max(abs(target-previous))/2))
        for k in range(1,count+1):
            rows.append(np.rint(previous+(target-previous)*k/count).astype(int));stages.append('turn1' if i<5 else 'turn2')
        previous=target
        if i in (4,9):
            rows.extend([target.copy()]*34);stages.extend(['hold1' if i==4 else 'hold2']*34)
    rows=np.array(rows,dtype=int)
    first=sim_to_ticks(d.ctrl)
    if np.max(abs(np.diff(np.vstack([first,rows]),axis=0)))>2:raise RuntimeError('Dense step exceeds2ticks')
    report=[];last_phase=None;max_shift=0.;max_tracking=0.;reason=None
    for i,(goal,phase) in enumerate(zip(rows,stages)):
        d.ctrl[:]=ticks_to_sim(goal)
        until=d.time+(.075 if phase.startswith('hold') else .060)
        while d.time<until-1e-10:
            mujoco.mj_step(m,d)
            max_tracking=max(max_tracking,float(np.max(abs(d.ctrl-d.qpos[:16]))))
            max_shift=max(max_shift,float(np.linalg.norm(d.geom_xpos[cube]-C)*1000))
        angle=5 if phase.endswith('1') else 10
        desired=Rotation.from_rotvec(axis*np.radians(angle)).as_matrix()@B
        current=d.geom_xmat[cube].reshape(3,3)
        change=Rotation.from_matrix(current@B.T).as_rotvec()
        target_error=float(np.degrees(np.linalg.norm(Rotation.from_matrix(current@desired.T).as_rotvec())))
        row=dict(index=i,phase=phase,simulation_time=float(d.time),
                 yaw_component_degrees=float(np.degrees(change@axis)),target_error_degrees=target_error,
                 center_shift_mm=float(np.linalg.norm(d.geom_xpos[cube]-C)*1000))
        if phase!=last_phase:
            if report:report[-1]['phase_end']=True
            row['phase_start']=True;last_phase=phase
        report.append(row)
        if max_shift>30 or not np.isfinite(d.qpos).all():
            reason='model placement retention lost';break
    if report:report[-1]['phase_end']=True
    ends=[r for r in report if r.get('phase_end')]
    passed=reason is None and len(ends)==4 and all(r['target_error_degrees']<3 and r['center_shift_mm']<5 for r in ends if r['phase'].startswith('hold'))
    out=BASE/'two_small_turns_model_probe';out.mkdir(exist_ok=True)
    np.savez_compressed(out/'end.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,
                        act=d.act,qacc_warmstart=d.qacc_warmstart,time=np.array(d.time))
    np.savez_compressed(out/'research_goals.npz',goals=rows,phases=np.array(stages),start_goal=first)
    review=dict(kind='two5degree_model_turns_from_measured_reference_hypothesis',phase_ends=ends,
        model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(PATH.read_bytes()).hexdigest(),
        initial_state_sha256=hashlib.sha256(INITIAL.read_bytes()).hexdigest(),
        model_two_small_turns_passed=passed,stop_reason=reason,maximum_center_shift_mm=max_shift,
        maximum_joint_tracking_rad=max_tracking,goal_count=len(rows),trace=report,
        cube_state_reset_during_path=False,external_cube_forces=False,
        ninety_degree_turn_verified=False,physical_cube_pose_measured=False,
        physical_motion_executed=False,executable_candidate=False)
    (out/'review.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in review.items() if k!='trace'}))


if __name__=='__main__':main()
