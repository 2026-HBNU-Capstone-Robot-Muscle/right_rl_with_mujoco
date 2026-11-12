"""Current/target command-pose preview. No device access or motion permission."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
from mujoco_playground import registry
from fresh_empty_grasp_plan_20261027 import FreshEmptyGraspPlan,REQUEST
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim

def main():
    plan=FreshEmptyGraspPlan(REQUEST);start=plan.source_position();target=plan.plan(start)[-1][1];out=REQUEST.parent/'preview_v3';out.mkdir(exist_ok=True);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,out/'scene.xml');d=mujoco.MjData(m);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    palette={'if':(.15,.4,.85,1),'mf':(.15,.65,.25,1),'rf':(.94,.65,.1,1),'th':(.88,.2,.2,1)}
    for i in range(m.ngeom):
        name=m.geom(i).name or '';body=m.body(int(m.geom_bodyid[i])).name or ''
        if int(m.geom_bodyid[i]) not in hand and not name.startswith(('support_','leap_mount_collision')):m.geom_group[i]=5
        if m.geom_type[i]==mujoco.mjtGeom.mjGEOM_PLANE:m.geom_group[i]=5
        for prefix,color in palette.items():
            if body.startswith(prefix+'_'):m.geom_rgba[i]=color;m.geom_matid[i]=-1
    bounds=[]
    for q in (start,target):
        d.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,d)
        for i in range(m.ngeom):
            if int(m.geom_bodyid[i]) in hand:
                radius=float(m.geom_rbound[i]);bounds.extend([d.geom_xpos[i]-radius,d.geom_xpos[i]+radius])
    look=(np.min(bounds,axis=0)+np.max(bounds,axis=0))/2
    m.vis.global_.offwidth=700;m.vis.global_.offheight=650;renderer=mujoco.Renderer(m,height=650,width=700);opt=mujoco.MjvOption();mujoco.mj_forward(m,d);palm=m.body('palm').id;P=d.xmat[palm].reshape(3,3).copy();up=-P[:,2];eye=look+.3*up-.28*P[:,0]-.24*P[:,1];z=(eye-look)/np.linalg.norm(eye-look);x=np.cross(up,z);x/=np.linalg.norm(x);y=np.cross(z,x);images=[]
    for q in (start,target):
        d.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,d);renderer.update_scene(d,scene_option=opt)
        for camera in renderer.scene.camera:camera.pos[:]=eye;camera.forward[:]=-z;camera.up[:]=y
        images.append(Image.fromarray(renderer.render()))
    renderer.close();canvas=Image.new('RGB',(1400,770),'white');canvas.paste(images[0],(0,60));canvas.paste(images[1],(700,60));font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22);draw=ImageDraw.Draw(canvas);draw.text((20,12),'MODEL: saved current OFF pose',fill='black',font=font);draw.text((720,12),'MODEL: proposed EMPTY target pose',fill='black',font=font);draw.text((20,732),'Blue=index   Green=middle   Yellow=ring   Red=thumb.  Cube stays OUT.',fill='black',font=font);canvas.save(out/'current_and_target.png');(out/'metadata.json').write_text(json.dumps(dict(current_ticks=start.tolist(),target_ticks=target.tolist(),command_pose_preview=True,real_devices_opened=False,physical_rotation_verified=False),indent=2)+'\n',encoding='utf-8');print('Saved',out/'current_and_target.png',flush=True)
if __name__=='__main__':main()
