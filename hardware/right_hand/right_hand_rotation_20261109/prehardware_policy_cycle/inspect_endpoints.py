"""Inspect actual model grip/upper-space evidence at replayed endpoints."""
import json
import numpy as np
from native_cycle import OUT,ROOT,assets,restore,Guard,save_json,angle
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
import mujoco


def inspect(model,data,up,B0):
    guard=Guard(model);guard.check(data)
    cube=model.geom('cube').id;B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube]
    contacts=[]
    for ci,c in enumerate(data.contact):
        a,b=int(c.geom1),int(c.geom2)
        if cube not in (a,b):continue
        other=b if a==cube else a;force=np.zeros(6);mujoco.mj_contactForce(model,data,ci,force)
        contacts.append({'geom':model.geom(other).name,'normal_force_N_model_only':float(force[0]),
                         'position_world':c.pos.tolist(),'height_above_cube_center_mm':float(-(c.pos-C)@B[:,1]*1000)})
    clearance=gaps(model,data,C,B,precompute(model,guard.hand),True)
    return {'yaw_deg':angle(B,B0,up),'target90_error_deg':orientation_error_deg(B,upright_yaw_target(B0,up,90)),
            'up_tilt_deg':face_up_error_deg(B,5,up),'center':C.tolist(),
            'upper_sweep_gap_mm':min(v for _,v in clearance),'closest_upper_geoms':sorted(clearance,key=lambda x:x[1])[:5],
            'contacts':contacts,'guard':guard.last,'physical_hold_load_not_verified':True}


def main():
    result={}
    for folder,kind in (('baseline_archived_zero','archived'),('baseline_active_saved_sac','active')):
        m,initial,_,_,_=assets(kind);z=dict(np.load(OUT/folder/'end_state.npz',allow_pickle=False));d=restore(m,z)
        result[folder]=inspect(m,d,z['up'],z['initial_cube_rotation'])
    pool=dict(np.load(OUT/'baseline_archived_zero/state_pool.npz',allow_pickle=False));m,initial,_,_,_=assets('archived')
    rows=[]
    for at in (0,927,1500,2000,2628,3003,3169,3986,4736,5636,5968,6166):
        pos=int(np.argmin(np.abs(pool['indices']-at)))
        z={key:pool[key][pos] for key in ('qpos','qvel','ctrl','act','qacc_warmstart','time')}
        d=restore(m,z);v=inspect(m,d,initial['up'],initial['initial_cube_rotation'])
        rows.append({'nearest_goal':int(pool['indices'][pos]),'requested':at,**v})
    result['nominal_phase_states']=rows
    save_json(OUT/'endpoint_review.json',result)
    print(json.dumps({name:value for name,value in result.items() if name!='nominal_phase_states'}),flush=True)
    print(json.dumps([{'goal':r['nearest_goal'],'yaw':r['yaw_deg'],'gap':r['upper_sweep_gap_mm']} for r in rows]),flush=True)


if __name__=='__main__':main()
