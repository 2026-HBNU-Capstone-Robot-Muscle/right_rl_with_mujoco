"""Native simulation replay illustration. No device I/O, no AI image edits."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from sac_cube_model import set_cube_physics
from validate_first_cube_rotation_20261022 import projected_yaw

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261022'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('candidate')
    args = p.parse_args()
    path = OUT / args.candidate
    r = json.loads(path.read_bytes())
    cfg = registry.get_default_config('LeapCubeFaceUp'); cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    m, _ = build_with_middle_thumb_hulls(env)
    set_cube_physics(m, side_m=.055, mass_kg=.100)
    m.vis.global_.offwidth = 640; m.vis.global_.offheight = 540
    d = mujoco.MjData(m)
    cube = m.geom('cube').id
    colors = {'if_': [.15,.45,.95,1], 'mf_': [.15,.8,.25,1],
              'rf_': [.95,.75,.1,1], 'th_': [.95,.2,.2,1]}
    for g in range(m.ngeom):
        body = int(m.geom_bodyid[g]); name = m.body(body).name
        if body == int(m.geom_bodyid[cube]):
            m.geom_group[g] = 0 if g == cube else 5
        elif name[:3] in colors:
            m.geom_rgba[g] = colors[name[:3]]
    m.geom_rgba[cube] = [.9,.55,.2,1]
    option = mujoco.MjvOption(); option.geomgroup[2] = 1; option.geomgroup[3:] = 0
    up = -np.asarray(m.opt.gravity); up = up / np.linalg.norm(up)
    first = r['validation'][0]
    initial = np.asarray(first['initial_qpos'])
    trace = [dict(time_s=0., qpos=initial.tolist())] + first['trace']
    d.qpos[:] = initial; mujoco.mj_forward(m,d)
    palm = m.body('palm').id
    palm_rotation = d.xmat[palm].reshape(3,3)
    horizontal = palm_rotation[:,1].copy()
    horizontal -= up * (horizontal @ up); horizontal /= np.linalg.norm(horizontal)
    lateral = np.cross(horizontal,up)
    center = (d.xpos[palm] + initial[-7:-4]) / 2
    cube_matrix = Rotation.from_quat(initial[-4:][[1,2,3,0]]).as_matrix()
    axis = int(np.argmin(np.abs(up @ cube_matrix)))
    frames = []
    with mujoco.Renderer(m,height=540,width=640) as renderer:
        for row in trace:
            q = np.asarray(row['qpos']); d.qpos[:] = q; mujoco.mj_forward(m,d)
            canvas = Image.new('RGB',(1280,610),'white')
            yaw = projected_yaw(initial[-4:],q[-4:],up)
            for col,(name,view) in enumerate((('TOP',up),('ANGLED',up*.65-horizontal*.7+lateral*.3))):
                view = view / np.linalg.norm(view)
                screen_up = horizontal.copy(); screen_up -= view * (screen_up @ view)
                screen_up /= np.linalg.norm(screen_up)
                camera = mujoco.MjvCamera(); camera.lookat[:] = center; camera.distance = .36
                renderer.update_scene(d,camera=camera,scene_option=option)
                for eye in renderer.scene.camera:
                    eye.pos[:] = center + view * .36
                    eye.forward[:] = -view; eye.up[:] = screen_up
                matrix = Rotation.from_quat(q[-4:][[1,2,3,0]]).as_matrix()
                start = q[-7:-4] + up*.032
                end = start + matrix[:,axis]*.035
                scene = renderer.scene
                geom = scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_ARROW,np.zeros(3),np.zeros(3),
                                   np.eye(3).ravel(),np.array([0.,.95,.95,1.]))
                mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_ARROW,.002,start,end)
                scene.ngeom += 1
                frame = Image.fromarray(renderer.render())
                canvas.paste(frame,(col*640,50))
                draw = ImageDraw.Draw(canvas)
                draw.text((col*640+10,10),f'{name} | t={row["time_s"]:.2f}s | cube yaw {yaw:+.1f} deg',fill='black')
                draw.text((col*640+10,27),'OFFLINE SIMULATION / CEM trajectory / NOT SAC / NOT physical success',fill='black')
            ImageDraw.Draw(canvas).text((10,592),
                'Cyan arrow follows cube. Thumb red / index blue / middle green / ring yellow. Cube placement is provisional.',fill='black')
            frames.append(canvas)
    gif = path.with_name(path.stem+'_replay.gif')
    frames[0].save(gif,save_all=True,append_images=frames[1:],duration=100,loop=0)
    compare = Image.new('RGB',(1280,1220),'white')
    compare.paste(frames[0],(0,0)); compare.paste(frames[-1],(0,610))
    comparison = path.with_name(path.stem+'_before_after.jpg')
    compare.save(comparison,quality=92)
    print(gif); print(comparison)


if __name__ == '__main__': main()
