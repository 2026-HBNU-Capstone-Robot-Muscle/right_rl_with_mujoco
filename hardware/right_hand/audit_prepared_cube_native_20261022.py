"""Native offline replay from saved prepared-hand telemetry; no hardware code."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import itertools
import json
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image, ImageDraw
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim, metadata
from sac_forward_geometry import measure, eligible

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'prepared_cube_review_20261022'


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    review=json.loads((OUT/'placement_review.json').read_bytes())
    source=ROOT/review['source']/'result.json'
    assert sha(source)==review['source_sha256']
    path=ROOT/'full_mapping_pose_review_20261019/inward_quantized/housing_sensitivity/path_OFFLINE_ONLY.json'
    old=json.loads(path.read_bytes());assert not old['hardware_execution_allowed']
    assert old['mapping']['motor_ids_in_sim_order']==metadata()['motor_ids_in_sim_order']
    start=np.array(review['placement_position_ticks'])
    command=np.array(review['placement_goal_ticks'])
    raw=np.array([old['start_ticks']]+old['goals_id_order'])
    # Compose the -64 tick thumb-clearance preparation with the original
    # anatomical intent over one second. This remains an offline candidate.
    blend=np.maximum(0,1-np.arange(len(raw))/20)
    raw=np.rint(raw+(command-raw[0])[None,:]*blend[:,None]).astype(int)
    goals=ticks_to_sim(raw)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env)
    set_cube_physics(m,side_m=.055,mass_kg=.100)
    joint_bounds=m.jnt_range[m.actuator_trnid[:,0]].copy()
    ctrl_bounds=m.actuator_ctrlrange.copy()
    palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand: hand.add(b)
    supports={m.geom(n).id for n in ('support_glass','support_wood')}
    cube=m.geom('cube').id
    cmasks=(int(m.geom_contype[cube]),int(m.geom_conaffinity[cube]))
    def contacts(d):
        self_depth=support_depth=cube_depth=0.
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(0,-float(c.dist)*1000)
            if a==cube or b==cube: cube_depth=max(cube_depth,depth)
            elif a in supports or b in supports: support_depth=max(support_depth,depth)
            elif int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
                self_depth=max(self_depth,depth)
        return self_depth,support_depth,cube_depth
    def state(q,cube_pose):
        d=mujoco.MjData(m);d.qpos[:16]=q;d.qpos[-7:]=cube_pose;mujoco.mj_forward(m,d);return d
    measured_q=ticks_to_sim(start)
    d=state(measured_q,[0,0,2,1,0,0,0])
    signs=np.array(list(itertools.product((-1,1),repeat=3)))
    boxes=[]
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) in hand and int(m.geom_type[g])==6 and (m.geom_contype[g] or m.geom_conaffinity[g]):
            boxes.append(dict(name=m.geom(g).name,body=m.body(int(m.geom_bodyid[g])).name,
                vertices=(d.geom_xpos[g]+(signs*m.geom_size[g])@d.geom_xmat[g].reshape(3,3).T).tolist()))
    geometry=dict(boxes_world_m=boxes,joint_anchors_world_m={m.joint(int(m.actuator_trnid[i,0])).name:
        d.xanchor[int(m.actuator_trnid[i,0])].tolist() for i in range(16)},
        palm_world_position=d.xpos[palm].tolist(),palm_world_rotation=d.xmat[palm].reshape(3,3).tolist())
    (OUT/'measured_geometry.json').write_text(json.dumps(geometry,indent=2))
    static=[]
    for goal in goals:
        x=state(goal,[0,0,2,1,0,0,0]);static.append(contacts(x)[:2])
    ranges=dict(joint_violation_samples=int(np.sum(np.any((goals<joint_bounds[:,0])|(goals>joint_bounds[:,1]),axis=1))),
        control_violation_samples=int(np.sum(np.any((goals<ctrl_bounds[:,0])|(goals>ctrl_bounds[:,1]),axis=1))),
        native_ranges_unchanged=True)
    resets=json.loads((ROOT/'sac_cube55_resets.json').read_bytes())['candidates']
    initposes=[np.array(r['qpos'][-7:]) for r in resets]
    report=dict(status='offline_current_prepared_hand_replay',source_sha256=sha(source),path_sha256=sha(path),
        mapping=metadata(),model_manifest=manifest,actual_placement_ticks=start.tolist(),command_start_ticks=command.tolist(),
        candidate_start_ticks=raw[0].tolist(),candidate_samples=len(raw),maximum_goal_step_ticks=int(np.max(abs(np.diff(raw,axis=0)))),
        ranges=ranges,static_max_self_overlap_mm=float(np.max(np.array(static)[:,0])),
        static_max_support_overlap_mm=float(np.max(np.array(static)[:,1])),trials=[],
        hardware_commands_sent=0,hardware_execution_allowed=False,live_sac=False,
        caveats=['Current motor angles are measured; cube resets below are synthetic, not registered from the photo.',
                 'Native actuator gains/friction do not certify the actual motors.',
                 'Old marker bridge is inconsistent with the observed green-up placement and cannot drive a policy.'])
    base_gain=m.actuator_gainprm.copy();base_bias=m.actuator_biasprm.copy()
    render_states=[]
    for loaded,ri,gain in [(False,0,g) for g in (.5,1,1.5)]+[(True,i,1) for i in range(len(initposes))]:
        m.geom_contype[cube]=cmasks[0] if loaded else 0
        m.geom_conaffinity[cube]=cmasks[1] if loaded else 0
        m.actuator_gainprm[:]=base_gain*gain;m.actuator_biasprm[:]=base_bias*gain
        d=state(measured_q,initposes[ri]);d.ctrl[:]=goals[0]
        initial=contacts(d);origin=d.qpos[-7:].copy()
        maximum=np.zeros(3);track=0;streak=best=0;trace=[]
        # Include a 1s hold before and after the candidate; never teleport hand
        # to the target command or the cube to the result of a previous trial.
        controls=[goals[0]]*20+list(goals)+[goals[-1]]*20
        for k,goal in enumerate(controls):
            d.ctrl[:]=goal
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d);maximum=np.maximum(maximum,contacts(d))
            err=float(np.max(abs(d.qpos[:16]-goal)));track=max(track,err)
            tip,axis,te=map(float,measure(origin[-4:],d.qpos[-4:]))
            shift=float(np.linalg.norm(d.qpos[-7:-4]-origin[:3]))
            ok=loaded and bool(eligible(tip,axis,te,shift))
            streak=streak+1 if ok else 0;best=max(best,streak)
            trace.append(dict(time_s=(k+1)*.05,tip_deg=float(np.degrees(tip)),target_error_deg=float(np.degrees(te)),
                shift_mm=shift*1000,tracking_rad=err,qpos=d.qpos.tolist()))
            if loaded and ri==0 and k%2==0:render_states.append(d.qpos.copy())
        trial=dict(loaded=loaded,synthetic_reset=ri,gain_scale=gain,initial_contact_depths_mm=list(initial),
            max_self_overlap_mm=float(maximum[0]),max_support_overlap_mm=float(maximum[1]),
            max_cube_overlap_mm=float(maximum[2]),max_tracking_rad=track,finite=bool(np.isfinite(d.qpos).all()),
            final_tip_deg=trace[-1]['tip_deg'] if loaded else None,longest_target_hold_s=best*.05,
            final_shift_mm=trace[-1]['shift_mm'] if loaded else None,trace=trace)
        report['trials'].append(trial)
        print(json.dumps({k:v for k,v in trial.items() if k!='trace'}),flush=True)
    m.actuator_gainprm[:]=base_gain;m.actuator_biasprm[:]=base_bias
    m.geom_contype[cube],m.geom_conaffinity[cube]=cmasks
    assert np.array_equal(joint_bounds,m.jnt_range[m.actuator_trnid[:,0]])
    assert np.array_equal(ctrl_bounds,m.actuator_ctrlrange)
    (OUT/'native_replay.json').write_text(json.dumps(report,indent=2))
    opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) not in hand and g not in supports and int(m.geom_bodyid[g])!=int(m.geom_bodyid[cube]):m.geom_group[g]=5
    frames=[]
    with mujoco.Renderer(m,height=480,width=640) as ren:
        for k,q in enumerate(render_states):
            d=mujoco.MjData(m);d.qpos[:]=q;mujoco.mj_forward(m,d)
            R=d.xmat[palm].reshape(3,3);o=d.xpos[palm]
            view=R@np.array([.4,-1,-.8]);view/=np.linalg.norm(view)
            up=-R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
            cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([0,-.025,-.06]);cam.distance=.42
            ren.update_scene(d,camera=cam,scene_option=opt)
            for eye in ren.scene.camera:eye.pos[:]=cam.lookat+view*.42;eye.forward[:]=-view;eye.up[:]=up
            im=Image.fromarray(ren.render());dr=ImageDraw.Draw(im);dr.rectangle((0,0,640,54),fill='black')
            dr.text((8,7),f'SAVED motor start + SYNTHETIC cube reset | t={k*.1+.05:.2f}s',fill='white')
            dr.text((8,28),'Offline fixed path; NOT live SAC / NOT physical success',fill='white');frames.append(im)
    frames[0].save(OUT/'candidate_simulation.gif',save_all=True,append_images=frames[1:],duration=100,loop=0)
    sheet=Image.new('RGB',(1280,1440),'white')
    for i,k in enumerate(np.linspace(0,len(frames)-1,6,dtype=int)):sheet.paste(frames[k],((i%2)*640,(i//2)*480))
    sheet.save(OUT/'candidate_keyframes.jpg')
    print('Saved offline candidate and measured geometry. No motor commands.',flush=True)


if __name__=='__main__': main()
