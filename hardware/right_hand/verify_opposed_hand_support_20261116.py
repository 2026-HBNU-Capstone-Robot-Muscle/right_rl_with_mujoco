"""Accept genuine underside hand support instead of requiring one named link.

Actual contacts, original limits, force allocation and free-cube replay remain
required. A palm support is not discarded just because the index base is clear.
"""
from pathlib import Path
import argparse,hashlib,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from research_quasistatic_wrench_path_20261105 import allocate_native
from canonical_grip_dynamics_20261105 import run
from hand_motor_mapping import ticks_to_sim


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--case',choices=('equal14','index6','thumb20','index6_thumb20','palm_offset','palm_offset_index6'),required=True)
    ap.add_argument('--weight',type=float,choices=(.02,.2,2.),default=2.)
    ap.add_argument('--exact',action='store_true')
    ap.add_argument('--tip-minimum',type=float,choices=(.6,1.2,1.6),default=.6)
    a=ap.parse_args()
    parent=ROOT/'right_hand_rotation_20261116/opposed_lower_grip'
    proposal=parent/'index_base_support'/a.case/'static_proposal.npz'
    tag='actual_hand_support_exact' if a.exact else ('actual_hand_support' if a.weight==2 else 'actual_hand_support_weight'+str(a.weight).replace('.','p'))
    if a.tip_minimum!=.6:tag+='_min'+str(a.tip_minimum).replace('.','p')
    out=proposal.parent/tag;out.mkdir(exist_ok=False)
    source=parent/'independent_initial.npz';z=dict(np.load(source,allow_pickle=False));z.update(dict(np.load(proposal,allow_pickle=False)))
    model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()});d=restore(m,z)
    hand,_,check=inspect_for(m);geometry=precompute(m,hand);cube=m.geom('cube').id
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();contact_names=set();supports=[];contacts=[]
    for c in d.contact:
        aa,bb=int(c.geom1),int(c.geom2)
        if cube not in (aa,bb):continue
        other=bb if aa==cube else aa
        if int(m.geom_bodyid[other]) not in hand or c.dist>=.0004:continue
        name=m.geom(other).name;local=(c.pos-C)@B*1000
        normal=c.frame[:3]*(1 if aa==cube else -1)
        row=dict(name=name,gap_mm=float(c.dist*1000),local_mm=local.tolist())
        contacts.append(row)
        if local[1]>-55/6:contact_names.add(name)
        if local[1]>20 and float(normal@B[:,1])>.5:supports.append(row)
    su,sp=check(d);pen=max((max(0.,-c['gap_mm']) for c in contacts),default=0.)
    clear=gaps(m,d,C,B,geometry,True)
    geometry_ok=bool({'if_tip','th_tip'}<=contact_names and supports and min(v for _,v in clear)>.3
        and max(sp.values(),default=0.)<=1e-6 and max(su.values(),default=0.)<=.15 and pen<=1.)
    lo,hi,nlo,nhi=native(m);force_ok=False;allocation=None;hold=None;began=time.perf_counter()
    if geometry_ok:
        if a.exact:
            from opposed_grip_force_balance_20261116 import allocate
            balanced,allocation=allocate(m,d,cube,hand,nlo,nhi,lo,hi,tip_minimum=a.tip_minimum)
        else:
            balanced,allocation=allocate_native(m,d,cube,C,-m.body_mass[m.body('cube').id]*m.opt.gravity,np.zeros(3),
                {'if_tip':2.,'th_tip':2.},hand,True,(nlo,nhi),('if_tip','th_tip'),normal_reference_weight=a.weight)
        if balanced is not None:
            goal=motor(balanced);err=np.array(allocation['exact_wrench_residual'] if a.exact else allocation['desired_wrench_tracking_residual'])
            force_ok=bool(max(abs(err[:3]))<.005 and max(abs(err[3:]))<.0002 and np.all(goal>=lo) and np.all(goal<=hi))
            if force_ok:
                initial=dict(z);initial.update(qpos=d.qpos.copy(),qvel=np.zeros_like(d.qvel),
                    qacc_warmstart=np.zeros_like(d.qacc_warmstart),ctrl=ticks_to_sim(goal))
                np.savez_compressed(out/'independent_initial.npz',**initial)
                np.savez_compressed(out/'posture_candidate.npz',goal_ticks=goal,hardware_execution_allowed=False)
                hold=run(m,initial,np.tile(goal,(134,1)),1,out/'hold_end.npz',out/'trace.npz',True,5,('if_tip','th_tip'))
    report=dict(case=a.case,pressure_objective_weight=None if a.exact else a.weight,exact_wrench_required=a.exact,model_sha256=sha(model),source_sha256=sha(source),proposal_sha256=sha(proposal),
        geometry_passed=geometry_ok,force_passed=force_ok,actual_contacts=contacts,actual_underside_support=supports,
        upper_gap_mm=min(v for _,v in clear),hold=hold,
        allocation=None if allocation is None else {k:v for k,v in allocation.items() if k!='contact_tasks'},
        independent_initialization_is_not_reaching=True,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(case=a.case,geometry=geometry_ok,force=force_ok,supports=supports,
        allocation=report['allocation'],hold=None if hold is None else {k:hold[k] for k in ['whole_grip_task_passed','final_up_tilt_deg','full_fixed_hold','settled_tail','stop']})),flush=True)


if __name__=='__main__':main()
