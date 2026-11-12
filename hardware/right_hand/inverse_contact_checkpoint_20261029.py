"""Read-only diagnostic hooks for a finite model replay, never a device API."""
import json
import numpy as np
import mujoco
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK

def save_checkpoint(model,data,folder,phase,sample):
    contacts=[];cube=model.geom('cube').id;cube_body=model.body('cube').id
    for i,c in enumerate(data.contact):
        a,b=int(c.geom1),int(c.geom2)
        if cube not in (a,b):continue
        other=b if a==cube else a
        f=np.zeros(6);mujoco.mj_contactForce(model,data,i,f)
        world=c.frame.reshape(3,3).T@f[:3]*(1 if b==cube else -1)
        contacts.append(dict(other=model.geom(other).name,distance_mm=float(c.dist)*1000,
            position=np.asarray(c.pos).tolist(),normal_force_N=float(f[0]),
            world_force_on_cube_N=world.tolist(),friction=np.asarray(c.friction).tolist(),
            cube_local_contact=(data.xmat[cube_body].reshape(3,3).T@(c.pos-data.xpos[cube_body])).tolist(),
            finger_body_local_contact=(data.xmat[int(model.geom_bodyid[other])].reshape(3,3).T@
                (c.pos-data.xpos[int(model.geom_bodyid[other])])).tolist()))
    ids=np.asarray(SIM_TO_MOTOR_IDS)-1;actual=np.empty(16);goal=np.empty(16)
    actual[ids]=data.qpos[:16]/TICK+2048;goal[ids]=data.ctrl/TICK+2048
    spec=mujoco.mjtState.mjSTATE_FULLPHYSICS
    state=np.empty(mujoco.mj_stateSize(model,spec));mujoco.mj_getState(model,data,state,spec)
    path=folder/(phase+'_checkpoint.npz')
    np.savez_compressed(path,state=state,ctrl=data.ctrl.copy(),qpos=data.qpos.copy(),qvel=data.qvel.copy(),
        warmstart=data.qacc_warmstart.copy(),state_spec=int(spec))
    mujoco.mj_saveModel(model,str(folder/'checkpoint_model.mjb'),None)
    row=dict(phase=phase,sample=sample,cube_qpos=data.qpos[-7:].tolist(),hand_actual_ticks=actual.tolist(),
        hand_goal_ticks=goal.tolist(),cube_contacts=contacts,
        actuator_torque_Nm=data.actuator_force.tolist(),devices_opened=False)
    (folder/(phase+'_contacts.json')).write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
    print('CHECKPOINT',json.dumps(row),flush=True)
