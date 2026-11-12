"""Render an already recorded upper-layer hypothesis; never simulate a pose."""
import os
os.environ['JAX_PLATFORMS']='cpu'
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib,shutil,subprocess
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from render_canonical_grip_evidence_20261105 import display_physical_cube,extra,png

def main():
    base=ROOT/'articulated_upper_layer';case=base/'bearing0.003_direction+1';report=json.loads((case/'review.json').read_bytes());model=base/'model.mjb'
    if not report['load_filter_passed'] or hashlib.sha256(model.read_bytes()).hexdigest()!=report['model_sha256']:raise ValueError('complete matching source required')
    source=case/'trajectory.npz';data=np.load(source);out=base/'evidence';out.mkdir(exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(model));m.vis.global_.offwidth=1000;m.vis.global_.offheight=800
    display_physical_cube(m)
    for g in range(m.ngeom):
        name=m.body(int(m.geom_bodyid[g])).name;colors={'if_':[.22,.45,.83,1],'mf_':[.25,.65,.4,1],'rf_':[.88,.69,.14,1],'th_':[.83,.3,.3,1]};color=next((v for k,v in colors.items() if name.startswith(k)),None)
        if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
    for name,color in (('cube',[.85,.85,.83,1]),('upper_layer_collision',[.2,.72,.36,1])):
        g=m.geom(name).id;m.geom_matid[g]=-1;m.geom_rgba[g]=color;m.geom_group[g]=2
    renderer=mujoco.Renderer(m,height=720,width=960);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.azimuth=35;cam.elevation=-38;cam.distance=.32;cam.lookat[:]=[.14,-.025,-.015]
    opt=mujoco.MjvOption();opt.geomgroup[:]=[1,1,1,0,0,0];d=mujoco.MjData(m);body=m.body('upper_layer').id
    ffmpeg=shutil.which('ffmpeg')
    if not ffmpeg:raise RuntimeError('ffmpeg required')
    video=out/'upper_layer_plus90_1x.mp4';fps=1/.15
    process=subprocess.Popen([ffmpeg,'-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','960x720','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(video)],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    try:
        for index,q in enumerate(data['qpos']):
            d.qpos[:]=q;d.qvel[:]=0;mujoco.mj_forward(m,d);renderer.update_scene(d,camera=cam,scene_option=opt)
            # A red visual dot makes the upper square's 90deg turn visible.
            B=d.xmat[body].reshape(3,3);C=d.xpos[body]
            extra(renderer.scene,mujoco.mjtGeom.mjGEOM_SPHERE,[.0025,0,0],C+B@np.array([.018,-.010,.018]),np.eye(3),[.95,.1,.08,1])
            rgb=renderer.render();process.stdin.write(rgb.tobytes())
            if index==0:png(out/'before.png',rgb)
            if index==len(data['qpos'])-1:png(out/'after.png',rgb)
        process.stdin.close();err=process.stderr.read().decode(errors='replace');code=process.wait()
        if code:raise RuntimeError(err[-1500:])
    finally:
        if process.poll() is None:process.terminate();process.wait()
        renderer.close()
    manifest=dict(status='saved_articulated_upper_layer_simulation_rendered',source_review=str((case/'review.json').relative_to(R)),source_review_sha256=hashlib.sha256((case/'review.json').read_bytes()).hexdigest(),source_trajectory_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),model_sha256=report['model_sha256'],video=dict(file=video.name,frames=len(data['qpos']),fps=fps,seconds=len(data['qpos'])/fps,sha256=hashlib.sha256(video.read_bytes()).hexdigest()),physical_video=False,red_dot_is_display_only=True,actual_left_hand_model_absent=True,not_right_hand_whole_cube90_demonstration=True,devices_opened=False,hardware_execution_allowed=False)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    (out/'index.html').write_text('''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>윗단 회전 하중의 별도 모델 시험</title><style>body{font:16px/1.7 system-ui;max-width:1000px;margin:30px auto;padding:0 20px;color:#24313c}video,img{width:100%}.note{background:#fff1ce;padding:15px}.pair{display:flex;gap:12px}.pair img{width:48%}</style><h1>윗단 회전 하중의 별도 모델 시험</h1><p class="note">시뮬레이션 저장 상태다. 오른손의 관절 목표는 고정하고 윗단에 가정한 회전 힘을 주었다. 실제 왼손의 모양과 접촉은 없다. 오른손이 큐브 전체를90도 돌리는 성공 영상과는 별개다.</p><p>초록색 윗단 위 빨간 점은 방향을 구분하는 표시다. 실제 충돌이나 질량을 추가하지 않는다.</p><video controls preload="metadata" src="upper_layer_plus90_1x.mp4"></video><div class="pair"><img src="before.png" alt="윗단 회전 전"><img src="after.png" alt="윗단 회전 후"></div><p>이 한 조건에서는 윗단약90도, 아래 몸통의 최대 회전약0.42도, 최대 이동약0.81mm였다. 마찰 설정과 회전 힘은 실제 큐브에서 측정한 값이 아니다.</p><p><a href="manifest.json">영상 출처</a> / <a href="../validation.json">다른 마찰 가설의 결과</a> / <a href="../../offline_review.html">전체 작업 상태</a></p></html>''',encoding='utf-8')
    print(json.dumps(manifest),flush=True)
if __name__=='__main__':main()
