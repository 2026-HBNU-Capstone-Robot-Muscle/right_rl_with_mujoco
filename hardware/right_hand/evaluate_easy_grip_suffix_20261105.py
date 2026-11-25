"""Free-cube continuation to a proposed common grip; no pose reset or devices."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore,evaluate,motor,inspect_for,native
from cube_contact_material_policy_20261029 import apply_material
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
R=Path(__file__).resolve().parent;CP=R/'right_hand_rotation_20261105/preintegration/reference_end';O=R/'right_hand_rotation_20261105/canonical_body_grip/easy_suffix'
ENDPOINT=R/'right_hand_rotation_20261105/canonical_body_grip/easy_endpoint_search/margin3_native/endpoint_level_25.npz'

def main():
    global O
    ap=argparse.ArgumentParser();ap.add_argument('--nominal-only',action='store_true');ap.add_argument('--index-raise',type=int,default=0);ap.add_argument('--endpoint',type=Path,default=ENDPOINT);ap.add_argument('--prefix',type=Path);ap.add_argument('--order',choices=('simultaneous','middle_first','thumb_first'),default='simultaneous');ap.add_argument('--tag');a=ap.parse_args()
    if not 0<=a.index_raise<=160:raise RuntimeError('Research support lift outside bounded search')
    if a.tag:O=O/a.tag
    O.mkdir(parents=True,exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(CP/'end_state.npz');ep=np.load(a.endpoint);start=motor(z['ctrl']);goal=ep['goal'].astype(int);goal[7]+=a.index_raise;count=int(np.ceil(max(abs(goal-start))/2));moving=np.rint([start+(goal-start)*i/count for i in range(1,count+1)]).astype(int);rows=np.vstack([moving,np.tile(goal,(134,1))]);lo,hi,nlo,nhi=native(m);idx=np.array(SIM_TO_MOTOR_IDS)-1;q=(goal[idx]-2048)*TICK-(z['ctrl']-z['qpos'][:16]);excess=max(0.,float(max(nlo-q)),float(max(q-nhi)))
    if a.prefix or a.order!='simultaneous':
        pieces=[];previous=start.copy()
        if a.prefix:
            prefix=np.load(a.prefix);assert np.array_equal(prefix['start'],start);pieces.extend(prefix['goals']);previous=prefix['goals'][-1].copy()
        targets=[]
        if a.order!='simultaneous':
            intermediate=previous.copy();active=np.arange(8,12) if a.order=='middle_first' else np.arange(4);intermediate[active]=goal[active];targets.append(intermediate)
        targets.append(goal)
        for target in targets:
            count=max(1,int(np.ceil(max(abs(target-previous))/2)));pieces.extend(np.rint(previous+(target-previous)*i/count).astype(int) for i in range(1,count+1));previous=target
        moving=np.array(pieces);rows=np.vstack([moving,np.tile(goal,(134,1))])
    if excess>1e-6 or np.any(rows<lo) or np.any(rows>hi) or max(abs(np.diff(np.vstack([start,rows]),axis=0)).ravel())>2:raise RuntimeError('Endpoint actual/command native range or step rejected')
    np.savez_compressed(O/'suffix.npz',start=start,goals=rows,phases=np.r_[['postturn_body_regrasp']*len(moving),['postturn_body_hold']*134])
    old=np.load(R/'right_hand_rotation_20261104/forward_hold/candidate.npz');assert np.array_equal(old['goals'][-1],start)
    np.savez_compressed(O/'candidate.npz',start=old['start'],goals=np.vstack([old['goals'],rows]),phases=np.r_[old['phases'],['postturn_body_regrasp']*len(moving),['postturn_body_hold']*134],forward_end_sample=len(old['goals']))
    hand,support,check=inspect_for(m);geoms=precompute(m,hand);gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();cases=[]
    for scale in ((1.,) if a.nominal_only else (1.,.5,1.5)):
      for friction in ((.3,) if a.nominal_only else (.3,.2,.5)):
        m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;apply_material(m,friction,hand,m.geom('cube').id);path=O/f'end_g{scale:g}_f{friction:g}.npz';began=time.perf_counter();dynamic=evaluate(m,z,rows,path);final=np.load(path);d=restore(m,final);cube=m.geom('cube').id;C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);cl=gaps(m,d,C,B,geoms,True);margin=min(v for name,v in cl);center_error=float(np.linalg.norm(C-ep['cube_center'])*1000);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1))))
        row=dict(gain_scale=scale,friction=friction,dynamic=dynamic,minimum_final_full_sweep_gap_mm=margin,closest_geoms=sorted(cl,key=lambda v:v[1])[:6],target_center_error_mm=center_error,final_up_tilt_deg=tilt,
            suffix_task_passed=bool(dynamic['safety_passed'] and margin>.3 and center_error<5 and tilt<5 and abs(dynamic['continuation_yaw_deg'])<5 and dynamic['hold_drift_deg']<3 and dynamic['hold_shift_mm']<2 and dynamic['lower_body_opposed_contact_fraction']>.8),elapsed_seconds=time.perf_counter()-began)
        cases.append(row);report=dict(status='free_cube_common_grip_suffix_research',endpoint_source=str(a.endpoint),prefix_source=str(a.prefix) if a.prefix else None,order=a.order,index_root_raise_ticks=a.index_raise,goals=len(rows),moving_goals=len(moving),fixed_hold_goals=134,drift_and_contact_measurement_last_seconds=4.05,maximum_allowed_regrasp_yaw_change_deg=5,cases=cases,all_passed=all(c['suffix_task_passed'] for c in cases),checkpoint_is_nominal_only=True,full_prefix_not_yet_replayed=True,cube_state_not_reset_after_checkpoint=True,pinned_cube_endpoint_not_used_as_dynamic_initial_state=True,physical_grip_not_verified=True,hardware_execution_allowed=False,devices_opened=False)
        (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k!='dynamic'}|{'dynamic':{k:v for k,v in dynamic.items() if k!='trace'}}),flush=True)
if __name__=='__main__':main()
