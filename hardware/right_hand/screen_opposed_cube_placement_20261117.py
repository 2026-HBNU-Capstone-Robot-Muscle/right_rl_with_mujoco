"""Initial-placement sensitivity, not an executed in-hand transport path.

The cube is placed once before simulation. Hand posture and recorded motor
goals remain unchanged. A minimum collision-free initial height is found;
no pose is reset after the replay starts and no hardware file is produced.
"""
from pathlib import Path
import json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from audit_opposed_working_grip_20261117 import MODEL,replay,sha
from research_rotation_body_hold_20261104 import restore,inspect_for

BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
prior=json.loads((BASE/'core_support_chain_10s/review.json').read_bytes())
source=ROOT/prior['source_path'];initial=dict(np.load(source,allow_pickle=False))
candidate=np.load(BASE/'core_support_chain_10s/extended_hold30/research_goals.npz',allow_pickle=False)
target=json.loads((BASE/'feedback_regrasp_normal1_gain1_nearpatch_basesupport_slip16_targetsurface_budget300_forceslide_margin2_paired_early141_latched_seated_freeifx/review.json').read_bytes())
for dx,roll in ((3.,0.),(3.,-3.)):
    out=BASE/f'placement_x{dx:g}_roll{roll:g}_hold30';out.mkdir(exist_ok=False)
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    d0=restore(m,initial);cube=m.geom('cube').id;hand,_,_=inspect_for(m)
    B=d0.geom_xmat[cube].reshape(3,3).copy();C=d0.geom_xpos[cube].copy()
    delta_R=B@Rotation.from_rotvec([0,0,np.radians(roll)]).as_matrix()@B.T
    body_R=Rotation.from_quat(initial['qpos'][-4:][[1,2,3,0]]).as_matrix()
    new_body_R=delta_R@body_R;quat=Rotation.from_matrix(new_body_R).as_quat();newB=delta_R@B
    z={k:np.array(v,copy=True) for k,v in initial.items()};z['qpos'][-4:]=quat[[3,0,1,2]]
    z['qvel'][-6:]=0.;z['qacc_warmstart'][:]=0.
    fit=None
    for lift in np.arange(0.,3.001,.05):
        shift=B[:,0]*dx/1000-newB[:,1]*lift/1000
        z['qpos'][-7:-4]=C+shift-new_body_R@m.geom_pos[cube]
        d=restore(m,z);penetration=max((max(0.,-float(c.dist)*1000) for c in d.contact
            if cube in (int(c.geom1),int(c.geom2)) and int(m.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)]) in hand),default=0.)
        if penetration<=.05:fit=(lift,shift,penetration);break
    report=dict(status='initial_cube_placement_sensitivity',dx_mm=dx,local_roll_deg=roll,
        source_sha256=sha(source),model_sha256=sha(MODEL),initial_cube_placement_changed=True,
        hand_initial_pose_unchanged=True,not_a_dynamic_repositioning_success=True,
        devices_opened=False,hardware_execution_allowed=False,full_quarter_turn_cycle_verified=False)
    if fit is not None:
        lift,shift,penetration=fit;z['initial_cube_center']=np.asarray(initial['initial_cube_center'])+shift
        np.savez_compressed(out/'trial_initial.npz',**z)
        began=time.perf_counter();result=replay(m,z,candidate['goals'],int(candidate['hold_start']),out,
            np.asarray(target['hold_target_center_world_m'])+shift,
            np.asarray(target['hold_target_rotation_world'])@Rotation.from_rotvec([0,0,np.radians(roll)]).as_matrix(),
            required_roles=('mf_tip','th_tip'))
        report.update(initial_lift_mm=lift,initial_hand_penetration_mm=penetration,initial_shift_world_m=shift.tolist(),
            passed=result['passed'],audit=result,elapsed_seconds=time.perf_counter()-began)
        print(json.dumps(dict(dx=dx,roll=roll,lift_mm=lift,passed=result['passed'],stop=result['stop'],
            hold_seconds=result['evaluated_hold_seconds'],shift_error=result['maximum_center_error_mm'])),flush=True)
    else:report.update(passed=False,stop='no_initial_clearance_within_3mm_lift')
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
