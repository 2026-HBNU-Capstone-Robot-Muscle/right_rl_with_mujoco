"""Free-cube endpoint sensitivity, offline only; no motor/camera dependencies.

Each perturbation starts from the same saved dynamic state. These local probes
are search filters, not evidence for a complete placement-to-grip trajectory.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,evaluate,motor,inspect_for,native,angle
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
from cube_contact_material_policy_20261029 import apply_material
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

R=Path(__file__).resolve().parent
ROOT=R/'right_hand_rotation_20261105/canonical_body_grip'
CP=R/'right_hand_rotation_20261105/preintegration/reference_end'
SEED=ROOT/'easy_suffix/supported_level_near_reference/end_g1_f0.3.npz'
OUT=ROOT/'dynamic_search'

def ramp(start, goal, hold=67):
    count=max(1,int(np.ceil(max(abs(goal-start))/2)))
    moving=np.rint([start+(goal-start)*i/count for i in range(1,count+1)]).astype(int)
    rows=np.vstack([moving,np.tile(goal,(hold,1))])
    if max(abs(np.diff(np.vstack([start,rows]),axis=0)).ravel())>2:raise RuntimeError('step limit')
    return rows

def assess(m,z,goal,reference,cache,save,hold=67):
    start=motor(z['ctrl']);lo,hi,_,_=native(m)
    if np.any(goal<lo) or np.any(goal>hi):return dict(admissible_goal=False,goal=goal.tolist())
    rows=ramp(start,goal,hold);began=time.perf_counter();res=evaluate(m,z,rows,save)
    final=np.load(save);d=restore(m,final);cube=m.geom('cube').id;C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
    clearance=gaps(m,d,C,B,cache,True);gap=min(v for name,v in clearance)
    tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1))))
    yaw=angle(B,reference['B'],z['up']);center=float(np.linalg.norm(C-reference['C'])*1000)
    return dict(admissible_goal=True,goal=goal.tolist(),safety_passed=res['safety_passed'],stop=res['stop'],yaw_error_from_reference_deg=yaw,center_shift_from_reference_mm=center,tilt_deg=tilt,upper_sweep_gap_mm=gap,closest_geoms=sorted(clearance,key=lambda v:v[1])[:4],hold_drift_deg=res['hold_drift_deg'],hold_shift_mm=res['hold_shift_mm'],lower_body_contact_fraction=res['lower_body_opposed_contact_fraction'],worst=res['worst'],final_contact_N=res['final_contact_N'],goals=len(rows),elapsed_seconds=time.perf_counter()-began,
        filter_passed=bool(res['safety_passed'] and abs(yaw)<5 and tilt<5 and gap>.3 and res['hold_drift_deg']<3 and res['hold_shift_mm']<2 and res['lower_body_opposed_contact_fraction']>.8),
        dynamic=res)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--group',choices=('thumb','middle','support'),required=True);ap.add_argument('--ticks',type=int,default=16);ap.add_argument('--seed',type=Path,default=SEED);ap.add_argument('--tag');a=ap.parse_args()
    if not 1<=a.ticks<=96:raise RuntimeError('bounded sensitivity search')
    out=OUT/(a.tag or f'probe_{a.group}_{a.ticks}');out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(a.seed);rz=np.load(CP/'end_state.npz');rd=restore(m,rz);cube=m.geom('cube').id;reference={'C':rd.geom_xpos[cube].copy(),'B':rd.geom_xmat[cube].reshape(3,3).copy()}
    hand,_,_=inspect_for(m);apply_material(m,.3,hand,cube);cache=precompute(m,hand);start=motor(z['ctrl']);cases=[]
    ids={'thumb':(1,2,3,4),'middle':(9,10,11,12),'support':(8,)}[a.group]
    for mid in ids:
      for delta in (-a.ticks,a.ticks):
        goal=start.copy();goal[mid-1]+=delta;save=out/f'id{mid}_delta{delta:+d}.npz';row=assess(m,z,goal,reference,cache,save)
        row.update(motor_id=mid,delta_ticks=delta);cases.append(row)
        report=dict(status='free_cube_local_endpoint_probe_only',seed_source=str(a.seed.relative_to(R)),seed_sha256=hashlib.sha256(a.seed.read_bytes()).hexdigest(),cases=cases,checkpoint_is_nominal_only=True,full_prefix_not_replayed=True,geometry_and_force_and_native_guards_unchanged=True,devices_opened=False,hardware_execution_allowed=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k not in ('dynamic','goal','closest_geoms')}),flush=True)
if __name__=='__main__':main()
