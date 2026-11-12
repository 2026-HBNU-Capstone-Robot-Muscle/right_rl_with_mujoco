"""Static contact-force feasibility at the selected common grip, offline only.

Uses the actual simulated contact points and friction cones. The inferred
position targets must retain native limits, the original actuator force
bounds and the .18rad tracking bound. This does not move the cube or validate
a trajectory, and produces no executable hardware command.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json
import numpy as np,mujoco
from scipy.optimize import minimize,Bounds,LinearConstraint
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from native_pyramidal_point_forces_20261106 import point_rays

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--cap',type=float,choices=(.15,.23));ap.add_argument('--native-cone',action='store_true');a=ap.parse_args()
    folder=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3';source=folder/'end_state.npz';m=mujoco.MjModel.from_binary_path(str(folder/'model.mjb'));z=np.load(source);d=restore(m,z)
    hand,_,_=inspect_for(m);cube=m.geom('cube').id;body=m.body('cube').id;C=d.qpos[-7:-4];up=z['up'];_,_,low,high=native(m);K=m.actuator_gainprm[:,0]
    assert np.all(K>0) and np.allclose(m.actuator_biasprm[:,1],-K) and np.allclose(m.actuator_gear[:,0],1.)
    contacts=[];blocks=[];joints=[];seed=[];groups={}
    for ci,c in enumerate(d.contact):
        x,y=int(c.geom1),int(c.geom2)
        if cube not in (x,y):continue
        other=y if x==cube else x
        if int(m.geom_bodyid[other]) not in hand:continue
        force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
        if force[0]<1e-5:continue
        frame=c.frame.reshape(3,3);normal=frame[0]*(1 if x==cube else -1);basis=np.column_stack([-normal,frame[1],frame[2]])
        cube_force=frame.T@force[:3]*(-1 if x==cube else 1);coordinates=basis.T@cube_force
        jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,c.pos,int(m.geom_bodyid[other]));r=c.pos-C
        torque=np.column_stack([np.cross(r,basis[:,j]) for j in range(3)])
        blocks.append(np.vstack([basis,torque]));joints.append(jp[:,:16].T@basis);seed.extend(coordinates)
        name=m.geom(other).name;groups.setdefault(name,[]).append(len(contacts))
        contacts.append(dict(geom=name,point_world_m=c.pos.tolist(),outward_normal=normal.tolist(),friction=float(c.friction[0]),friction_y=float(c.friction[1]),actual_normal_N=float(force[0]),actual_contact_force_on_cube_N=cube_force.tolist()))
    if len(contacts)<3:raise ValueError('at least three actual contact points required')
    W=np.column_stack(blocks);J=np.column_stack(joints);seed=np.asarray(seed);bias=d.qfrc_bias[:16]-d.qfrc_passive[:16]
    # Reconstruct measured actuator balance to expose any sign/point error.
    inferred=d.qpos[:16]+(bias+J@seed)/K
    reconstruction_error_ticks=float(max(abs(motor(inferred)-motor(d.ctrl))))
    measured_wrench=W@seed;gravity=-m.body_mass[body]*m.opt.gravity
    limited=np.asarray(m.actuator_forcelimited,bool)
    force_low=np.where(limited,m.actuator_forcerange[:,0],-np.inf);force_high=np.where(limited,m.actuator_forcerange[:,1],np.inf)
    if a.cap is not None:force_low=np.maximum(force_low,-a.cap);force_high=np.minimum(force_high,a.cap)
    bounds_low=np.maximum(np.maximum(K*(low-d.qpos[:16]),-.18*K),force_low)-bias
    bounds_high=np.minimum(np.minimum(K*(high-d.qpos[:16]),.18*K),force_high)-bias
    group_matrix=np.zeros((len(groups),len(seed)))
    for row,(name,indices) in zip(group_matrix,groups.items()):
        for i in indices:row[3*i]=1.
    constraints=[LinearConstraint(J,bounds_low,bounds_high),LinearConstraint(group_matrix,0,3.5)]
    native_matrix=None
    if a.native_cone:
        if int(m.opt.cone)!=0:raise ValueError('this audit expects native pyramidal contacts')
        native_matrix=np.zeros((4*len(contacts),len(seed)))
        for i,c in enumerate(contacts):
            local=np.array([[-c['friction'],sx,sy*c['friction']/c['friction_y']] for sx,sy in ((1,1),(1,-1),(-1,1),(-1,-1))]);native_matrix[4*i:4*i+4,3*i:3*i+3]=local
            assert np.max(local@point_rays(c['friction'],c['friction_y']))<1e-12
    def cones(x):return -native_matrix@x if native_matrix is not None else np.array([c['friction']*x[3*i]-np.linalg.norm(x[3*i+1:3*i+3]) for i,c in enumerate(contacts)])
    lower=np.tile([0.,-3.5,-3.5],len(contacts));upper=np.full(len(seed),3.5);feasible_seed=seed.copy()
    for i,c in enumerate(contacts):
        limit=c['friction']*feasible_seed[3*i];length=abs(feasible_seed[3*i+1])+abs(feasible_seed[3*i+2])*c['friction']/c['friction_y'] if a.native_cone else np.linalg.norm(feasible_seed[3*i+1:3*i+3])
        if length>limit:feasible_seed[3*i+1:3*i+3]*=limit/length
    cases=[];side=np.cross(up,np.array([1.,0,0]));side/=np.linalg.norm(side)
    targets=[('gravity_hold',gravity,np.zeros(3)),('lift_accel_0.2m_s2',gravity+up*.02,np.zeros(3)),('yaw_plus_0.003Nm',gravity,up*.003),('yaw_minus_0.003Nm',gravity,-up*.003),('yaw_plus_0.01Nm',gravity,up*.01),('yaw_minus_0.01Nm',gravity,-up*.01),('upright_correction_plus',gravity,side*.003),('upright_correction_minus',gravity,-side*.003)]
    scale=np.r_[np.ones(3),np.full(3,30.)]
    for name,F,T in targets:
        target=np.r_[F,T];equality=LinearConstraint(W,target,target)
        def cost(x):return float(np.sum((x-feasible_seed)**2)*.5)
        friction_constraint=LinearConstraint(native_matrix,-np.inf,0) if a.native_cone else {'type':'ineq','fun':cones}
        fit=minimize(cost,feasible_seed,method='SLSQP',bounds=Bounds(lower,upper),constraints=constraints+[equality,friction_constraint],options=dict(maxiter=600,ftol=1e-11))
        x=fit.x;residual=W@x-target;torque=J@x;cone_min=float(min(cones(x)));violation=max(0,float(max(bounds_low-torque)),float(max(torque-bounds_high)),float(max(group_matrix@x-3.5)))
        match=bool(fit.success and max(abs(residual[:3]))<1e-5 and max(abs(residual[3:]))<1e-6 and cone_min>=-1e-7 and violation<1e-7)
        # A force allocation is still a static hypothesis even if feasible.
        cases.append(dict(task=name,static_force_allocation_feasible=match,optimizer_success=bool(fit.success),optimizer_message=fit.message,force_error_N=residual[:3].tolist(),moment_error_Nm=residual[3:].tolist(),minimum_friction_cone_slack_N=cone_min,maximum_native_or_force_bound_violation=violation,normal_N_by_geom={n:float(sum(x[3*i] for i in indices)) for n,indices in groups.items()},maximum_inferred_joint_torque_Nm=float(max(abs(bias+torque))),inferred_goal_ticks=motor(d.qpos[:16]+(bias+torque)/K).tolist(),trajectory_validated=False))
    report=dict(status='static_wrench_feasibility_not_a_motion_proof',source_state=str(source.relative_to(R)),source_state_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_model_sha256=hashlib.sha256((folder/'model.mjb').read_bytes()).hexdigest(),actual_contact_points=contacts,actual_measured_wrench_on_cube=measured_wrench.tolist(),gravity_compensation_force_N=gravity.tolist(),actuator_balance_reconstruction_error_ticks=reconstruction_error_ticks,maximum_source_joint_speed_rad_s=float(max(abs(d.qvel[:16]))),static_balance_is_approximate_if_source_still_moving=True,cases=cases,normal_force_cap_per_contact_geom_hypothesis_N=3.5,model_actuator_force_limit_enabled=limited.tolist(),real450mA_is_not_a_calibrated_joint_torque_bound_here=True,native_goal_tracking_and_actuator_force_bounds_unchanged=True,no_cube_pose_changed_or_integrated=True,no_executable_command_or_transport_adapter=True,hardware_execution_allowed=False,devices_opened=False)
    report['hypothetical_additional_joint_torque_cap_Nm']=a.cap
    report['force_allocation_uses_same_native_pyramidal_cone']=a.native_cone
    out=ROOT/'contact_wrench_audit'
    if a.cap is not None:out=out/f'cap{a.cap:g}{"_native_cone" if a.native_cone else ""}'
    out.mkdir(parents=True,exist_ok=True);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(contact_points=len(contacts),groups=list(groups),reconstruction_error_ticks=reconstruction_error_ticks,measured_wrench=measured_wrench.tolist(),cases=[dict(task=c['task'],feasible=c['static_force_allocation_feasible'],message=c['optimizer_message']) for c in cases],hardware_execution_allowed=False)),flush=True)
if __name__=='__main__':main()
