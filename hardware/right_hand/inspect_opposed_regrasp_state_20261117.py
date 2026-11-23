"""Read saved model states only: actual contact loads and fingertip gaps."""
from pathlib import Path
import argparse,json,sys
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,inspect_for,motor
from audit_opposed_working_grip_20261117 import MODEL,ROLES
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

base=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
ap=argparse.ArgumentParser();ap.add_argument('--extra-folder');args=ap.parse_args()
states={'working':base/'level_triple_normal1p2_settle1_settled_frame_active_hold400_heightlock0p0/complete_hold_audit/audit_end_state.npz',
        'postturn':base/'next_turn5p0_palmclear_margin8_fingerclear_allpalms/end_state.npz'}
if args.extra_folder:
    extra=base/args.extra_folder/'end_state.npz'
    if not extra.exists():extra=base/args.extra_folder/'audit_end_state.npz'
    states[args.extra_folder]=extra
m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
hand,_,check=inspect_for(m);geom=precompute(m,hand);result={}
for label,path in states.items():
    d=restore(m,np.load(path,allow_pickle=False));cube=m.geom('cube').id
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();contacts=[];distances={}
    for ci,c in enumerate(d.contact):
        aa,bb=int(c.geom1),int(c.geom2)
        if cube not in (aa,bb):continue
        other=bb if aa==cube else aa;force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
        contacts.append(dict(name=m.geom(other).name,gap_mm=float(c.dist*1000),force_N=float(force[0]),
            cube_local_mm=((c.pos-C)@B*1000).tolist(),
            cube_outward_normal=((c.frame[:3]*(1 if aa==cube else -1))@B).tolist()))
    for name in ROLES:
        g=m.geom(name).id;points=np.zeros(6);dist=mujoco.mj_geomDistance(m,d,cube,g,.1,points)
        distances[name]=dict(gap_mm=float(dist*1000),cube_point_mm=((points[:3]-C)@B*1000).tolist(),
            tip_point_mm=((points[3:]-C)@B*1000).tolist())
    result[label]=dict(center=C.tolist(),rotation=B.tolist(),ticks=motor(d.qpos[:16]).tolist(),
        goals=motor(d.ctrl).tolist(),contacts=contacts,tip_gaps=distances,
        upper_clearance=sorted(gaps(m,d,C,B,geom,True),key=lambda x:x[1])[:4])
(base/('regrasp_state_inspection_extra.json' if args.extra_folder else 'regrasp_state_inspection.json')).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps({args.extra_folder:result[args.extra_folder]} if args.extra_folder else result),flush=True)
