"""Create a research variant using inverse-static position feedforward.

No force control or gain/current increase is applied to hardware. Finite free
Cube dynamics must independently validate any proposed target sequence.
"""
from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'propose_moving_center_inverse_20261029.py').read_text(encoding='utf-8')
anchor="    hand={m.body('palm').id}"
helper="""    def static_command(raw,rotation,force_N):
        d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:]=[0,0,10,1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d)
        assert np.array_equal(m.actuator_trnid[:,0],np.arange(16)) and np.all(m.actuator_gear[:,0]==1)
        torque=d.qfrc_bias[:16].copy()
        for _,(body,point,cl,_) in anchors.items():
            world=d.xpos[body]+d.xmat[body].reshape(3,3)@point;jac=np.zeros((3,m.nv))
            mujoco.mj_jac(m,d,jac,None,world,body);inward=-np.sign(cl[0])*rotation[:,0]
            torque+=jac[:,:16].T@(inward*force_N)
        q=ticks_to_sim(raw)+torque/m.actuator_gainprm[:,0];values=np.empty(16);values[IDX]=q/TICK+2048
        result=goal.copy();result[ACTIVE]=np.rint(values[ACTIVE]).astype(int)
        result[13]=int(round(raw[13]+offset[13]));return result
"""
changes=[
("OUT=ROOT/'right_hand_rotation_20261029/observed_inverse_moving_center'","OUT=ROOT/'right_hand_rotation_20261030/inverse_static_feedforward'"),
(anchor,helper+anchor),
("OUT.mkdir(exist_ok=True)","OUT.mkdir(parents=True,exist_ok=True)"),
("for height_mm in (0.,3.,6.):", "for force_N in (.5,.9,1.3):\n      height_mm=0."),
("name=f'height{height_mm:g}_normal{nw:g}'", "name=f'force{force_N:g}'"),
("previous=actual.copy();previous[13]-=128;previous_command=goal.copy();previous_command[13]-=128;center_shift=np.zeros(2)","previous=actual.copy();previous_command=goal.copy();center_shift=np.zeros(2)"),
("sequence=list(interp(goal,previous_command));frames=[];failed=None", "sequence=[];frames=[];failed=None"),
("for angle in (-5.,-10.,-15.,-20.,-25.,-30.,-36.3371815813):", "for angle in (0.,-5.,-10.,-15.,-20.,-25.,-30.,-36.3371815813):"),
("command=np.rint(raw+offset).astype(int);center_shift=sol.x[8:].copy()", "command=static_command(raw,R,force_N);center_shift=sol.x[8:].copy()"),
("normal_weight=nw,command", "normal_weight=nw,hypothetical_contact_force_N=force_N,command"),
("sequence.extend(interp(previous_command,command));previous=raw;previous_command=command", "sequence.extend(interp(previous_command,command));previous=raw;previous_command=command\n            if angle==0.:\n                ring=previous.copy();ring[13]-=128;ring_command=previous_command.copy();ring_command[13]-=128\n                sequence.extend(interp(previous_command,ring_command));previous=ring;previous_command=ring_command"),
]
for a,b in changes:
 if s.count(a)!=1:raise RuntimeError((a,s.count(a)))
 s=s.replace(a,b)
(R/'propose_inverse_static_feedforward_20261030.py').write_text(s,encoding='utf-8')
print('Created offline inverse-static feedforward proposer; execution not approved.')
