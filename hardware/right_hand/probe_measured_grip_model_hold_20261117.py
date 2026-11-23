"""Two-second dynamics diagnostic of a fitted placement, not the measured cube pose."""
from pathlib import Path
import hashlib,json
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261117/user_measured_grip'
MODEL=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
FIT=BASE/'cube_placement_hypothesis/review.json'
VIEW=ROOT/'measured_grip_view_20261117_175648_673729/result.json'


def main():
    fit=json.loads(FIT.read_bytes());case=fit['cases'][0]
    observation=json.loads(VIEW.read_bytes())
    if not fit['static_hypothesis_found']:raise RuntimeError('No static hypothesis')
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    d=mujoco.MjData(m)
    d.qpos[:16]=ticks_to_sim(observation['after']['actual_ticks'])
    d.ctrl[:]=ticks_to_sim(observation['after']['goal_ticks'])
    mujoco.mj_forward(m,d)
    palm,cube=m.body('palm').id,m.geom('cube').id
    P,R=d.xpos[palm].copy(),d.xmat[palm].reshape(3,3).copy()
    r,p=case['roll_pitch_in_palm_degrees']
    B=R@Rotation.from_euler('xyz',[r,p,case['yaw_in_palm_degrees']],degrees=True).as_matrix()
    C=P+R@(np.array(case['center_in_palm_mm'])/1000)
    q=Rotation.from_matrix(B).as_quat();d.qpos[-7:-4]=C;d.qpos[-4:]=np.r_[q[3],q[:3]]
    d.qvel[:]=0;mujoco.mj_forward(m,d)
    out=BASE/'model_hold_probe';out.mkdir(exist_ok=True)
    np.savez_compressed(out/'initial.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,
                        act=d.act,qacc_warmstart=d.qacc_warmstart,time=np.array(d.time))
    trace=[];max_shift=max_angle=0.;reason=None
    steps=int(np.ceil(2./m.opt.timestep))
    for i in range(steps):
        mujoco.mj_step(m,d)
        shift=float(np.linalg.norm(d.geom_xpos[cube]-C)*1000)
        angle=float(np.degrees(np.linalg.norm(Rotation.from_matrix(d.geom_xmat[cube].reshape(3,3)@B.T).as_rotvec())))
        max_shift=max(max_shift,shift);max_angle=max(max_angle,angle)
        if i%max(1,int(.075/m.opt.timestep))==0:
            contacts=[]
            for ci,c in enumerate(d.contact):
                a,b=int(c.geom1),int(c.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a
                f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f)
                contacts.append(dict(name=m.geom(other).name,distance_mm=float(c.dist*1000),normal_force_n=float(f[0])))
            trace.append(dict(time=float(d.time),center_shift_mm=shift,angle_change_degrees=angle,
                              contacts=contacts,maximum_tracking_rad=float(np.max(abs(d.ctrl-d.qpos[:16])))))
        if shift>15 or angle>25 or not np.isfinite(d.qpos).all():
            reason='Placement hypothesis did not retain its initial pose';break
    np.savez_compressed(out/'end.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,
                        act=d.act,qacc_warmstart=d.qacc_warmstart,time=np.array(d.time))
    report=dict(kind='fitted_placement_two_second_model_diagnostic',
        fit_sha256=hashlib.sha256(FIT.read_bytes()).hexdigest(),model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        maximum_center_change_mm=max_shift,maximum_orientation_change_degrees=max_angle,
        simulated_seconds=float(d.time),stop_reason=reason,trace=trace,
        measured_cube_pose=False,actual_physical_grip_invalidated=False,
        torque_cap_nm_is_not_a_450ma_conversion=True,physical_motion_executed=False,
        dynamics_roughly_stationary=reason is None and max_shift<2 and max_angle<5,
        executable_candidate=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='trace'}))
    print('last_contacts',trace[-1]['contacts'])


if __name__=='__main__':main()
