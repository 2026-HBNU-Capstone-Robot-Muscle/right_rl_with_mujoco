"""Offline dynamics replay of legacy intent after ID correction, not a live policy."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json, hashlib, argparse
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image, ImageDraw
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_cube_model import set_cube_physics
from sac_forward_geometry import measure, eligible
from hand_motor_mapping import metadata, ticks_to_sim

ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--inward-quantization',action='store_true')
parser.add_argument('--housing-proxies',action='store_true')
args=parser.parse_args()
OUT=ROOT/'full_mapping_pose_review_20261019'
if args.inward_quantization:OUT=OUT/'inward_quantized'
if args.housing_proxies:OUT=OUT/'housing_sensitivity'
OUT.mkdir(parents=True,exist_ok=True)
source=ROOT/'full_mapping_review_20261018/remapped_path_OFFLINE_ONLY.json'
doc=json.loads(source.read_text(encoding='utf-8'))
assert not doc['hardware_execution_allowed']
goals=ticks_to_sim(np.array([doc['start_ticks']]+doc['goals_id_order']))
resets_path=ROOT/'sac_cube55_resets.json'
resets=json.loads(resets_path.read_text(encoding='utf-8'))['candidates']
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg)
if args.housing_proxies:
    from offline_housing_proxy_model import build_with_middle_thumb_hulls
    m,manifest=build_with_middle_thumb_hulls(env)
else:m,manifest=build(env)
set_cube_physics(m,side_m=.055,mass_kg=.100)
# Same environment root-floor control range; do not add the old hypothetical
# rf_dip mechanical upper bound, which was inferred from mislabelled ID16.
m.actuator_ctrlrange[[0,4,8],0]=.65
quantization=None
if args.inward_quantization:
    tick=2*np.pi/4096
    original=goals.copy()
    lower=np.maximum(m.jnt_range[m.actuator_trnid[:,0],0],m.actuator_ctrlrange[:,0])
    upper=np.minimum(m.jnt_range[m.actuator_trnid[:,0],1],m.actuator_ctrlrange[:,1])
    lo=np.ceil((lower+np.pi)/tick).astype(int)
    hi=np.floor((upper+np.pi)/tick).astype(int)
    integer=np.clip(np.rint((goals+np.pi)/tick).astype(int),lo,hi)
    goals=integer*tick-np.pi
    assert np.all(goals>=lower-1e-12) and np.all(goals<=upper+1e-12)
    assert np.max(abs(goals-original))<=tick+1e-12
    from hand_motor_mapping import sim_to_ticks
    raw=sim_to_ticks(goals)
    assert np.max(abs(ticks_to_sim(raw)-goals))<1e-12
    quantization=dict(max_change_ticks=float(np.max(abs(goals-original))/tick),
        changed_entries=int(np.sum(abs(goals-original)>1e-12)),
        lower_ticks_sim_order=lo.tolist(),upper_ticks_sim_order=hi.tolist())
    (OUT/'path_OFFLINE_ONLY.json').write_text(json.dumps(dict(
        format='offline_mapping_review_not_executable_v1',hardware_execution_allowed=False,
        mapping=metadata(),quantization=quantization,start_ticks=raw[0].tolist(),goals_id_order=raw[1:].tolist(),
        warning='Simulation candidate only. Physical preparation, geometry and actuator transfer unvalidated.'),indent=2),encoding='utf-8')
palm=m.body('palm').id;hand={palm}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={m.geom(n).id for n in ('support_glass','support_wood')}
cube=m.geom('cube').id;cube_masks=(int(m.geom_contype[cube]),int(m.geom_conaffinity[cube]))
report=dict(status='offline_dynamic_replay_only',mapping=metadata(),hardware_execution_allowed=False,
    hardware_commands_sent=0,live_sac=False,actor_retrained=False,model_manifest=manifest,
    source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (source,resets_path)},
    artificial_old_id16_joint_limit_removed=True,model_parameters=[],trials=[],
    inward_quantization=quantization,
    caveats=['Ideal model actuator is not calibrated to real PID/current/friction.',
             'Saved simulated reset, not actual measured cube placement.',
             'Physical mapping, model geometry and marker registration require separate validation.',
             'No hypothetical mechanical bound assigned to a new motor by relabelling.'])
for i in range(16):
    j=int(m.actuator_trnid[i,0])
    report['model_parameters'].append(dict(joint=m.joint(j).name,motor_id=metadata()['motor_ids_in_sim_order'][i],
        joint_range=m.jnt_range[j].tolist(),ctrlrange=m.actuator_ctrlrange[i].tolist(),
        ctrllimited=bool(m.actuator_ctrllimited[i]),gain=m.actuator_gainprm[i].tolist(),bias=m.actuator_biasprm[i].tolist(),
        requested_min=float(goals[:,i].min()),requested_max=float(goals[:,i].max()),
        requested_outside_joint_range_samples=int(np.sum((goals[:,i]<m.jnt_range[j,0])|(goals[:,i]>m.jnt_range[j,1]))),
        requested_outside_ctrlrange_samples=int(np.sum((goals[:,i]<m.actuator_ctrlrange[i,0])|(goals[:,i]>m.actuator_ctrlrange[i,1])))))
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
frame_states=[]
for loaded in (False,True):
    m.geom_contype[cube]=cube_masks[0] if loaded else 0
    m.geom_conaffinity[cube]=cube_masks[1] if loaded else 0
    for ri,reset in enumerate(resets):
        d=mujoco.MjData(m);d.qpos[:]=reset['qpos'];d.ctrl[:]=goals[0];mujoco.mj_forward(m,d)
        start=d.qpos[-7:].copy();rows=[];hold=longest=0
        maxself=maxsupport=maxerror=maxactualrange=0.;pairs={}
        for k,goal in enumerate(goals):
            d.ctrl[:]=goal;step_self=step_support=0.
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d)
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    if a in support or b in support:step_support=max(step_support,-float(c.dist)*1000)
                    elif int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
                        overlap=max(0.,-float(c.dist)*1000);step_self=max(step_self,overlap)
                        key=' / '.join(sorted([m.geom(a).name,m.geom(b).name]));pairs[key]=max(pairs.get(key,0),overlap)
            error=float(np.max(np.abs(d.qpos[:16]-goal)))
            tip,axis,err=map(float,measure(start[-4:],d.qpos[-4:],xp=np))
            shift=float(np.linalg.norm(d.qpos[-7:-4]-start[:3]))
            valid=bool(eligible(tip,axis,err,shift,xp=np)) and step_support<=.5
            hold=hold+1 if valid else 0;longest=max(longest,hold)
            jids=m.actuator_trnid[:,0]
            range_error=float(np.max(np.maximum(m.jnt_range[jids,0]-d.qpos[:16],d.qpos[:16]-m.jnt_range[jids,1])))
            maxactualrange=max(maxactualrange,range_error)
            maxself=max(maxself,step_self);maxsupport=max(maxsupport,step_support);maxerror=max(maxerror,error)
            rows.append(dict(step=k,time_s=(k+1)*.05,tracking_rad=error,self_overlap_mm=step_self,
                support_overlap_mm=step_support,tip_deg=float(np.degrees(tip)),target_error_deg=float(np.degrees(err)),
                center_shift_mm=shift*1000,qpos=d.qpos.tolist()))
            if loaded and ri==0 and k%2==0:frame_states.append(d.qpos.copy())
        trial=dict(loaded=loaded,reset=ri,finite=bool(np.isfinite(d.qpos).all()),
            max_tracking_rad=maxerror,max_native_self_overlap_mm=maxself,max_support_overlap_mm=maxsupport,
            max_actual_joint_range_excursion_rad=maxactualrange,longest_target_hold_s=longest*.05,
            final_tip_deg=rows[-1]['tip_deg'] if loaded else None,
            final_target_error_deg=rows[-1]['target_error_deg'] if loaded else None,
            peak_pairs=pairs,rows=rows)
        report['trials'].append(trial)
        print(json.dumps({k:v for k,v in trial.items() if k not in ('rows','peak_pairs')}),flush=True)
(OUT/'dynamic_replay.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
m.geom_contype[cube],m.geom_conaffinity[cube]=cube_masks
# Illustrative, native simulation frames at 10 fps. Actual-time rendering.
frames=[]
# Hide the unrelated target visual cube and floor; retain hand, actual cube,
# and measured support objects to avoid suggesting a second manipulated cube.
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand and g not in support and int(m.geom_bodyid[g])!=int(m.geom_bodyid[cube]):
        m.geom_group[g]=5
with mujoco.Renderer(m,height=600,width=800) as ren:
    for k,q in enumerate(frame_states):
        d=mujoco.MjData(m);d.qpos[:]=q;mujoco.mj_forward(m,d)
        R=d.xmat[palm].reshape(3,3);o=d.xpos[palm]
        view=R@np.array([.4,-1,-.8]);view/=np.linalg.norm(view)
        up=-R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
        cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([0,-.025,-.06]);cam.distance=.42
        ren.update_scene(d,camera=cam,scene_option=opt)
        for eye in ren.scene.camera:eye.pos[:]=cam.lookat+view*.42;eye.forward[:]=-view;eye.up[:]=up
        im=Image.fromarray(ren.render());draw=ImageDraw.Draw(im)
        draw.rectangle((0,0,800,55),fill='black')
        draw.text((10,8),f'OFFLINE fixed-path dynamics | t={k*.10+.05:.2f}s | cube 55mm / 100g',fill='white')
        draw.text((10,29),'NOT live SAC / NOT real hardware success / model calibration pending',fill='white')
        frames.append(im)
frames[0].save(OUT/'simulated_path.gif',save_all=True,append_images=frames[1:],duration=100,loop=0)
sheet=Image.new('RGB',(1600,1800),'white')
for i,k in enumerate((0,10,20,30,40,len(frames)-1)):sheet.paste(frames[k],((i%2)*800,(i//2)*600))
sheet.save(OUT/'simulated_path_keyframes.jpg')
print('Saved offline dynamics and animation',flush=True)
