"""Summarize saved leveling attempts and the shapes still blocking the top."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore,native,motor
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from hand_motor_mapping import SIM_TO_MOTOR_IDS,SIM_JOINT_NAMES

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--match',default='*');args=ap.parse_args()
    raw=json.loads((ROOT/'near_quarter_full_replay/review.json').read_bytes());m=mujoco.MjModel.from_binary_path(str(R/raw['source_model']));hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    geometry=precompute(m,hand);cube=m.geom('cube').id
    for p in sorted((ROOT/'countergrip_level').glob(args.match+'/review.json')):
        r=json.loads(p.read_bytes());v=r.get('recorded_goal_replay');source=p.parent/('verified_end_state.npz' if v else 'end_state.npz');z=np.load(source);d=restore(m,z);C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);cl=sorted(gaps(m,d,C,B,geometry,True),key=lambda x:x[1]);contact_points=[]
        for ci,c in enumerate(d.contact):
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a;ff=np.zeros(6);mujoco.mj_contactForce(m,d,ci,ff);jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,c.pos,int(m.geom_bodyid[other]));J=jp[:,np.r_[np.arange(8),np.arange(12,16)]];normal=c.frame[:3]*(1 if a==cube else -1);hu=z['up']-normal*(normal@z['up']);contact_points.append(dict(geom=m.geom(other).name,normal_N=float(ff[0]),height_up_mm=float((c.pos-C)@z['up']*1000),height_cube_mm=float(-((c.pos-C)@B)[1]*1000),jacobian_singular_values=np.linalg.svd(J,compute_uv=False).tolist(),tangential_height_motion_mm_per_rad_by_motor={str(SIM_TO_MOTOR_IDS[i]):float(hu@jp[:,i]*1000) for i in range(16) if abs(hu@jp[:,i])>1e-7}))
        lo,hi,_,_=native(m);goals=motor(d.ctrl);margins=sorted((dict(motor=i+1,lower_ticks=float(goals[i]-lo[i]),upper_ticks=float(hi[i]-goals[i])) for i in range(16)),key=lambda t:min(t['lower_ticks'],t['upper_ticks']))[:6]
        row=dict(case=p.parent.name,source_state=source.relative_to(R).as_posix(),yaw=r['final_yaw_deg'],reasons=r['reasons'],guard=r['guard_stop'],closest_geoms=cl[:8],closest_goal_bounds=margins,joint_actual_rad={SIM_JOINT_NAMES[i]:float(d.qpos[i]) for i in range(16)},goal_ticks_by_ID={str(SIM_TO_MOTOR_IDS[i]):int(round(d.ctrl[i]*4096/(2*np.pi)+2048)) for i in range(16)},tail=None if v is None else v['settled_tail'],contacts=None if v is None else v['final_contact_N'],contact_points=contact_points,devices_opened=False,hardware_execution_allowed=False);(p.parent/'workspace_diagnostic.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k not in ('joint_actual_rad','goal_ticks_by_ID','tail')} | {'tail_gap':None if v is None else v['settled_tail']['minimum_separating_plane_gap_1ms_mm'],'contact_fraction':None if v is None else v['settled_tail']['opposed_lower_body_contact_fraction']}),flush=True)
if __name__=='__main__':main()
