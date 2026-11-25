"""Review nominal and recorded-lag hand geometry; render a proposed empty pose."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/legacy_stage4_ring_uncurl_other_v1'
def main():
    seed=json.loads((OUT/'seed.json').read_bytes());p=np.load(OUT/'rrt_thumb_pair3_path.npz');goals=np.vstack([p['start'],p['goals']]);res=np.asarray(seed['response_residual_ticks']);lo,hi=limits();assert np.all(goals>=lo) and np.all(goals<=hi);assert np.max(abs(np.diff(goals,axis=0)))<=2
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);scene=OUT/'empty_review_scene.xml';m,manifest=build(env,scene);set_cube_physics(m,.055,.1);d=mujoco.MjData(m);palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};support.update(i for i in range(m.ngeom) if (m.geom(i).name or '').startswith('leap_mount_collision') or m.geom(i).name=='floor');cases=[]
    for scale in (0.,.25,.5,.75,1.):
        sp=sup=0.;worst=None
        for i,g in enumerate(goals):
            d.qpos[:16]=ticks_to_sim(g+scale*res);d.qpos[-7:]=seed['settled_cube_qpos'];d.qvel[:]=0;mujoco.mj_forward(m,d)
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                if ha and hb and dep>sp:sp=dep;worst=dict(sample=i,a=m.geom(a).name,b=m.geom(b).name)
                if (a in support and hb) or (b in support and ha):sup=max(sup,dep)
        cases.append(dict(recorded_response_fraction=scale,max_self_overlap_mm=sp,max_support_overlap_mm=sup,worst_self_pair=worst,passed=bool(sp<=.01 and sup<=.15)))
    report=dict(status='empty_grasp_transition_nominal_to_recorded_response_geometry',samples=len(goals),cases=cases,all_passed=all(c['passed'] for c in cases),obstacles_checked=[m.geom(i).name for i in sorted(support)],native_goal_limits_checked=True,two_tick_steps_checked=True,range_is_not_a_complete_response_uncertainty_envelope=True,current_off_start_not_read=True,current_off_recovery_path_not_screened=True,loaded_cube_path_not_certified=True,physical_motion_not_executed=True,hardware_execution_allowed=False,real_devices_opened=False);(OUT/'empty_path_response_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    # Separate preview XML. Neither execution model nor mesh assets are changed.
    d.qpos[:16]=ticks_to_sim(goals[-1]);mujoco.mj_forward(m,d);P=d.xmat[palm].reshape(3,3).copy();up=-P[:,2];look=d.xpos[palm]+P@np.array([.015,0.,-.04])
    # The saved source XML uses virtual include assets. Render the already
    # compiled model and adjust only the GL cameras; do not reload bare XML.
    preview=OUT/'empty_goal_preview_scene.xml'
    if preview.exists():preview.unlink()
    v=m;vd=d
    for i in range(v.ngeom):
        if int(v.geom_bodyid[i]) not in hand and i not in support:v.geom_group[i]=5
    v.geom_group[v.geom_type==mujoco.mjtGeom.mjGEOM_PLANE]=5;v.vis.global_.offwidth=700;v.vis.global_.offheight=650
    palette={'if':(.15,.4,.85,1),'mf':(.15,.65,.25,1),'rf':(.94,.65,.1,1),'th':(.88,.2,.2,1)}
    for i in range(v.ngeom):
        name=v.body(int(v.geom_bodyid[i])).name
        for prefix,color in palette.items():
            if name.startswith(prefix+'_'):v.geom_rgba[i]=color;v.geom_matid[i]=-1
    mujoco.mj_forward(v,vd);opt=mujoco.MjvOption();renderer=mujoco.Renderer(v,height=650,width=700)
    from PIL import Image,ImageDraw,ImageFont
    images=[]
    for side in (1.,-1.):
        eye=look+.25*up-.23*P[:,0]+side*.19*P[:,1];z=(eye-look)/np.linalg.norm(eye-look);x=np.cross(up,z);x/=np.linalg.norm(x);y=np.cross(z,x)
        renderer.update_scene(vd,scene_option=opt)
        for camera in renderer.scene.camera:camera.pos[:]=eye;camera.forward[:]=-z;camera.up[:]=y
        images.append(Image.fromarray(renderer.render()))
    renderer.close();canvas=Image.new('RGB',(1400,770),'white');canvas.paste(images[0],(0,60));canvas.paste(images[1],(700,60));draw=ImageDraw.Draw(canvas);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22);draw.text((20,12),'MODEL ONLY: proposed empty grasp posture; ring finger opened',fill='black',font=font);draw.text((20,732),'Blue=index   Green=middle   Yellow=ring   Red=thumb.  No hardware motion verified.',fill='black',font=font);canvas.save(OUT/'empty_grasp_goal_views.png');print(json.dumps(report,indent=2),flush=True);print('Saved empty_grasp_goal_views.png',flush=True)
if __name__=='__main__':main()
