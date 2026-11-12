"""Retarget the prior lower IF/TH grip to the current cube, then free replay."""
from pathlib import Path
import sys,json,time,hashlib
ROOT=Path(__file__).resolve().parent
if sys.platform=='win32':sys.path.insert(0,str(ROOT/'.runtime/sim-win'))
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from research_probe_index_handoff_20261109 import ramp

CP=ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction_linux'
CURRENT=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
PRIOR=ROOT/'right_hand_rotation_20261105/canonical_body_grip/continuous_handoff/end_state.npz'
PRIOR_MODEL=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
OUT=ROOT/'right_hand_rotation_20261109/retargeted_index_thumb'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True)
    original=json.loads((CP/'review.json').read_bytes())
    if not original['same_original_path_reconstructed']:raise RuntimeError('Original Linux state not reconstructed')
    model=mujoco.MjModel.from_binary_path('current.mjb',assets={'current.mjb':CURRENT.read_bytes()})
    old_model=mujoco.MjModel.from_binary_path('prior.mjb',assets={'prior.mjb':PRIOR_MODEL.read_bytes()})
    z=np.load(CP/'checkpoint_927.npz',allow_pickle=False);prior=np.load(PRIOR,allow_pickle=False)
    d=restore(model,z);p=restore(model,z);old=restore(old_model,prior)
    cube=model.geom('cube').id;oldcube=old_model.geom('cube').id
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();oldC=old.geom_xpos[oldcube].copy();oldB=old.geom_xmat[oldcube].reshape(3,3).copy()
    up=z['up'];hand,_,check=inspect_for(model);cache=precompute(model,hand)
    qprior=old.qpos[:16].copy();pd=old.ctrl-qprior;lowraw,highraw,_,_=native(model);ids=np.array(SIM_TO_MOTOR_IDS)-1
    low=(np.ceil(lowraw[ids])+8-2048)*TICK-pd;high=(np.floor(highraw[ids])-8-2048)*TICK-pd
    specs=[]
    for name in ('if_tip','th_tip'):
        oldg=old_model.geom(name).id;g=model.geom(name).id
        contact=min((c for c in old.contact if {int(c.geom1),int(c.geom2)}=={oldcube,oldg}),key=lambda c:c.dist)
        G=old.geom_xmat[oldg].reshape(3,3);n=contact.frame[:3]*(1 if int(contact.geom1)==oldcube else -1)
        specs.append((g,G.T@(contact.pos-old.geom_xpos[oldg]),G.T@n,oldB.T@(contact.pos-oldC),oldB.T@n))
    rotations=[Rotation.from_rotvec(np.array([0.,1.,0.])*np.radians(deg)).as_matrix() for deg in (0,90,180,270)]
    symmetry=min(rotations,key=lambda S:sum(np.linalg.norm(B@S@item[3]-oldB@item[3]) for item in specs))
    start=motor(d.ctrl);cases=[]
    for height in (0.,5.,10.,15.):
        targetC=C+up*height/1000;quat=Rotation.from_matrix(B).as_quat();target_cube=np.r_[targetC,quat[3],quat[:3]]
        def measure(x):
            p.qpos[:16]=x;p.qpos[-7:]=target_cube;p.qvel[:]=0;p.ctrl[:]=x+pd;mujoco.mj_forward(model,p)
            errors=[];normals=[]
            for g,local,localn,clocal,cn in specs:
                G=p.geom_xmat[g].reshape(3,3);errors.extend((p.geom_xpos[g]+G@local-(targetC+B@symmetry@clocal))*1000)
                normals.extend((G@localn-B@symmetry@cn)*2)
            cl=gaps(model,p,targetC,B,cache,True);su,sp=check(p)
            pen=max((max(0.,-float(c.dist)*1000) for c in p.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.r_[np.asarray(errors)*4,normals,(x-qprior)*.6,[max(0.,.5-value)*8 for _,value in cl],
                         max(sp.values(),default=0.)*1500,max(su.values(),default=0.)*1500,max(0.,pen-.6)*700]
        fit=least_squares(measure,np.clip(qprior,low+1e-8,high-1e-8),bounds=(low,high),max_nfev=140,ftol=1e-7,xtol=1e-7,gtol=1e-7,diff_step=1e-5)
        residual=measure(fit.x);goal=motor(p.ctrl);cl=sorted(gaps(model,p,targetC,B,cache,True),key=lambda row:row[1]);su,sp=check(p)
        error=float(max(np.linalg.norm(residual[:6].reshape(2,3)/4,axis=1)))
        geometry=bool(error<.8 and cl[0][1]>.3 and not sp and max(su.values(),default=0.)<=.15)
        folder=OUT/f'height{height:g}';folder.mkdir(exist_ok=True)
        row={'height_change_mm':height,'static_geometry_only':{'passed':geometry,'max_contact_point_error_mm':error,'upper_gaps_mm':cl,'self_contact_mm':sp,'support_contact_mm':su},
             'goal':goal.tolist(),'optimizer_evaluations':fit.nfev,'free_replays':[]}
        # Even a rejected static fit may reveal safe support-only behavior; it is
        # never promoted without actual lower contact and complete clearance.
        for order in ('together','index_first','index_thumb_first'):
            previous=start.copy();pieces=[]
            for group in ([] if order=='together' else [np.arange(4,8)] if order=='index_first' else [np.arange(4,8),np.arange(4)]):
                via=previous.copy();via[group]=goal[group];pieces.append(ramp(previous,via));previous=via
            pieces.extend([ramp(previous,goal),np.tile(goal,(134,1))]);rows=np.vstack(pieces)
            np.savez_compressed(folder/(order+'_path.npz'),start=start,goals=rows)
            dynamic=run(model,z,rows,1,folder/(order+'_end.npz'),None,False,required_lower_contacts=('if_tip','th_tip'))
            row['free_replays'].append({'order':order,'dynamic':dynamic,'coarse_passed':bool(geometry and dynamic['whole_grip_task_passed'])})
            print(json.dumps({'height':height,'order':order,'static_ok':geometry,'static_error_mm':error,'static_gap_mm':cl[0][1],
                              'safety':dynamic['safety_passed'],'completed':dynamic['completed_goals'],'total':dynamic['total_goals'],
                              'coarse_task':bool(geometry and dynamic['whole_grip_task_passed']),'tail':dynamic['settled_tail']}),flush=True)
        cases.append(row);(folder/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
        report={'status':'prior_common_grip_retargeted_to_natural_cube_research','source_model_sha256':sha(CURRENT),
                'prior_endpoint_sha256':sha(PRIOR),'prior_model_sha256':sha(PRIOR_MODEL),
                'prior_center_offset_world_mm':((oldC-C)*1000).tolist(),'prior_center_height_difference_mm':float((oldC-C)@up*1000),
                'cube_symmetry_used_for_contact_correspondence':symmetry.tolist(),'cube_pose_pinned_only_in_geometry_proposal':True,
                'cube_never_reset_or_fixed_in_dynamic_continuations':True,'cases':cases,
                'hardware_execution_allowed':False,'devices_opened':False,'elapsed_seconds':time.perf_counter()-began}
        (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
