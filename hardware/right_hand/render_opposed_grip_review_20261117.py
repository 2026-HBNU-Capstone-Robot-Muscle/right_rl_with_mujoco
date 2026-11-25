"""Source-bound model views; colors are display-only, no device access."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import argparse,hashlib,json,sys
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,inspect_for
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display,png

ap=argparse.ArgumentParser();ap.add_argument('--validated-hold',action='store_true');ap.add_argument('--followup',action='store_true');ap.add_argument('--predictive-hold',action='store_true');args=ap.parse_args()
out=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'/('predictive_hold_evidence' if args.predictive_hold else 'followup_evidence' if args.followup else 'validated_hold_evidence' if args.validated_hold else 'evidence');out.mkdir(exist_ok=False)
model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()})
hand,_,_=inspect_for(m);geometry=precompute(m,hand)
m.vis.global_.offwidth=1000;m.vis.global_.offheight=800;display_physical_cube(m)
for g in range(m.ngeom):
    body=m.body(int(m.geom_bodyid[g])).name
    color=next((v for k,v in {'if_':[.22,.45,.83,1],'mf_':[.25,.65,.4,1],'rf_':[.88,.69,.14,1],'th_':[.83,.3,.3,1]}.items() if body.startswith(k)),None)
    if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
renderer=mujoco.Renderer(m,height=720,width=960);cam=mujoco.MjvCamera();cam.distance=.32
cam.lookat[:]=[.14,-.025,-.015];opt=mujoco.MjvOption();opt.geomgroup[:]=[1,1,1,0,0,0];rows=[]
cases=[('pair_leveled','level_normal1p2_settle1/verified_end_state.npz'),
       ('three_finger_stable','shared_palm_station18/hold_end.npz'),
       ('three_finger_level_failed','level_triple_normal1p2_settle1/end_state.npz')]
if args.validated_hold:
    cases=[('verified_hold30','level_triple_normal1p2_settle1_settled_frame_active_hold400_heightlock0p0/complete_hold_audit/audit_end_state.npz')]
    newest='next_turn5p0_palmclear_margin8_fingerclear_allpalms/end_state.npz'
    if (out.parent/newest).exists():
        cases.append(('next_turn_probe',newest))
if args.followup:
    chosen='core_support_chain_10s/audit_end_state.npz'
    latest=out.parent/'core_support_chain_10s/active_hold30/review.json'
    if latest.exists() and json.loads(latest.read_bytes())['passed']:
        chosen='core_support_chain_10s/active_hold30/audit_end_state.npz'
    cases=[('regrasp_verified',chosen)]
if args.predictive_hold:
    chosen='core_support_chain_10s/predictive_verified20s'
    if not json.loads((out.parent/chosen/'review.json').read_bytes())['passed']:
        raise RuntimeError('Independent20second pass required')
    cases=[('regrasp_verified20s',chosen+'/audit_end_state.npz')]
for label,path in cases:
    source=out.parent/path;d=restore(m,np.load(source,allow_pickle=False));cube=m.geom('cube').id
    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
    clearance=sorted(gaps(m,d,C,B,geometry,True),key=lambda p:p[1])
    row=dict(label=label,source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
             lowest_clearance_mm=clearance[:5],images=[])
    for name,az,el in [('side',35,-38),('top',90,-78)]:
        cam.azimuth=az;cam.elevation=el;renderer.update_scene(d,camera=cam,scene_option=opt)
        cube_display(renderer.scene,C,B,True)
        filename=label+'_'+name+'.png';png(out/filename,renderer.render());row['images'].append(filename)
    rows.append(row)
renderer.close()
(out/'manifest.json').write_text(json.dumps(dict(model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
    cases=rows,model_images_not_physical_photos=True,display_only_finger_colors=True,devices_opened=False),indent=2)+'\n')
print(json.dumps(rows),flush=True)
