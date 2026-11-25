"""Offline coordinated tip motion from a free-cube grip checkpoint."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,propose,evaluate,motor,inspect_for,angle
from dynamic_common_grip_search_20261105 import CP,SEED,ROOT,R
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from cube_contact_material_policy_20261029 import apply_material
from bounded_contact_transport_proposal_20261105 import propose_bounded

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--turn',type=float,required=True);ap.add_argument('--normal',type=float,default=10);a=ap.parse_args()
    if not -25<=a.turn<=25:raise RuntimeError('bounded offline correction')
    out=ROOT/f'coordinated_refinement/turn{a.turn:g}_normal{a.normal:g}';out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(SEED);ref=restore(m,np.load(CP/'end_state.npz'));cube=m.geom('cube').id;B0=ref.geom_xmat[cube].reshape(3,3).copy();C0=ref.geom_xpos[cube].copy();hand,_,_=inspect_for(m);apply_material(m,.3,hand,cube);cache=precompute(m,hand);began=time.perf_counter()
    rows,ik,error=propose_bounded(m,z,a.turn,a.normal)
    report=dict(status='coordinated_free_cube_contact_refinement_research',parameters=vars(a),geometry_stop=error,IK=ik,source=str(SEED.relative_to(R)),source_sha256=hashlib.sha256(SEED.read_bytes()).hexdigest(),hardware_execution_allowed=False,devices_opened=False,physical_grasp_verified=False,full_prefix_not_yet_replayed=True,cube_state_not_reset_in_dynamic_suffix=True)
    if rows is not None:
        np.savez_compressed(out/'suffix.npz',start=motor(z['ctrl']),goals=rows)
        dynamic=evaluate(m,z,rows,out/'end_state.npz');d=restore(m,np.load(out/'end_state.npz'));B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube];cl=gaps(m,d,C,B,cache,True);gap=min(v for name,v in cl);yaw=angle(B,B0,z['up']);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1))))
        report.update(dynamic=dynamic,minimum_full_sweep_gap_mm=gap,yaw_error_from_reference_deg=yaw,tilt_deg=tilt,center_shift_from_reference_mm=float(np.linalg.norm(C-C0)*1000),closest_geoms=sorted(cl,key=lambda r:r[1])[:6],suffix_filter_passed=bool(dynamic['safety_passed'] and abs(yaw)<5 and tilt<5 and gap>.3 and dynamic['hold_drift_deg']<3 and dynamic['hold_shift_mm']<2 and dynamic['lower_body_opposed_contact_fraction']>.8))
        old=np.load(ROOT/'easy_suffix/supported_level_near_reference/candidate.npz');assert np.array_equal(old['goals'][-1],motor(z['ctrl']));np.savez_compressed(out/'candidate.npz',start=old['start'],goals=np.vstack([old['goals'],rows]),phases=np.r_[old['phases'],['postturn_contact_correct']*(len(rows)-134),['postturn_body_hold_corrected']*134])
    report['elapsed_seconds']=time.perf_counter()-began;(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k not in ('IK','dynamic')}|{'dynamic':{k:v for k,v in report.get('dynamic',{}).items() if k!='trace'}}),flush=True)
if __name__=='__main__':main()
