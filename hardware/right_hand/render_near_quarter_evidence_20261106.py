"""Render source-bound saved free-cube states, never invent a successful pose."""
import os
os.environ['JAX_PLATFORMS']='cpu'
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json,shutil,subprocess
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display,extra,png

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    folder=ROOT/'near_quarter_full_replay';review=folder/'review.json';report=json.loads(review.read_bytes());model=R/report['source_model'];assert sha(model)==report['source_model_sha256']
    source=folder/'trajectory.npz';z=np.load(source);m=mujoco.MjModel.from_binary_path(str(model));m.vis.global_.offwidth=1000;m.vis.global_.offheight=800;display=display_physical_cube(m)
    colors={'if_':[.22,.45,.83,1],'mf_':[.25,.65,.4,1],'rf_':[.88,.69,.14,1],'th_':[.83,.3,.3,1]}
    for g in range(m.ngeom):
        name=m.body(int(m.geom_bodyid[g])).name;color=next((v for k,v in colors.items() if name.startswith(k)),None)
        if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
        if name=='cube':m.geom_matid[g]=-1;m.geom_rgba[g]=[.9,.9,.88,1]
    out=folder/'evidence';out.mkdir(exist_ok=True);renderer=mujoco.Renderer(m,height=720,width=960);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.distance=.32;cam.lookat[:]=[.14,-.025,-.015];cam.azimuth=35;cam.elevation=-38;opt=mujoco.MjvOption();opt.geomgroup[:]=[1,1,1,0,0,0];d=mujoco.MjData(m);cube=m.geom('cube').id
    def frame(q,sweep=False):
        d.qpos[:]=q;d.qvel[:]=0;mujoco.mj_forward(m,d);C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);renderer.update_scene(d,camera=cam,scene_option=opt);cube_display(renderer.scene,C,B,sweep)
        for axis,sign,color in ((0,1,[.85,.16,.13,1]),(0,-1,[1.,.45,.07,1]),(1,1,[.12,.3,.85,1]),(1,-1,[.13,.7,.3,1]),(2,1,[.95,.95,.95,1]),(2,-1,[.95,.82,.08,1])):
            size=np.full(3,.023);size[axis]=.00015;pos=np.zeros(3);pos[axis]=sign*.0277;extra(renderer.scene,mujoco.mjtGeom.mjGEOM_BOX,size,C+B@pos,B,color)
        return renderer.render()
    views=[]
    for name,az,el in (('side',35,-38),('top',90,-78)):
        cam.azimuth=az;cam.elevation=el
        for label,index in (('start',0),('before_extension',int(np.argmin(abs(z['samples']-report['retained_prefix_goals'])))),('end',len(z['qpos'])-1)):
            file=f'{label}_{name}.png';png(out/file,frame(z['qpos'][index],label=='end'));views.append(file)
    cam.azimuth=35;cam.elevation=-38;videos=[];ffmpeg=shutil.which('ffmpeg')
    if not ffmpeg:raise RuntimeError('ffmpeg required')
    odd=np.flatnonzero(z['samples']%2==1)
    for name,selection,fps,speed in (('whole_path_6x',odd[::4],10.,6),('additional_turn_1x',odd[z['samples'][odd]>report['retained_prefix_goals']],1/.15,1)):
        file=out/(name+'.mp4');proc=subprocess.Popen([ffmpeg,'-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','960x720','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(file)],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        try:
            for i in selection:proc.stdin.write(frame(z['qpos'][i],z['samples'][i]>report['total_goals']-134).tobytes())
            proc.stdin.close();error=proc.stderr.read().decode(errors='replace');code=proc.wait()
            if code:raise RuntimeError(error[-1500:])
        finally:
            if proc.poll() is None:proc.terminate();proc.wait()
        videos.append(dict(file=file.name,frames=len(selection),fps=fps,seconds=len(selection)/fps,playback_speed=speed,sha256=sha(file)))
    renderer.close();v=report['dynamic'];manifest=dict(status='saved_near_quarter_full_replay_rendered',source_review=str(review.relative_to(R)),source_review_sha256=sha(review),source_trajectory_sha256=sha(source),source_model_sha256=sha(model),videos=videos,images=views,display=display,face_colors_are_visual_only=True,whole90_and_common_grip_passed=report['quarter_and_common_grip_passed'],physical_video=False,devices_opened=False,hardware_execution_allowed=False)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>추가 회전 경로 검토</title><style>body{font:16px/1.7 system-ui;max-width:1000px;margin:30px auto;padding:0 20px;color:#24313c}.note{background:#fff0cd;padding:15px}video{width:100%}.pair{display:flex;gap:10px}.pair img{width:49%}</style><h1>큐브 추가 회전 경로 검토</h1><p class="note">저장된 자유 큐브 시뮬레이션이다. 마지막 회전량은 '''+f"{v['final_yaw_from_placement_deg']:.2f}"+'''도다. 회전 뒤 공통 파지와 윗단 공간 검사는 아직 통과하지 못했다. 실제 로봇에서의90도 성공 영상이 아니다.</p><p>파랑=검지, 초록=중지, 노랑=약지, 빨강=엄지. 큐브 면 색과 주황 원통은 방향과 윗단 공간을 보이는 표시이며 물리 형상을 바꾸지 않는다.</p><h2>처음 배치부터 전체 경로,6배속</h2><video controls preload="metadata" src="whole_path_6x.mp4"></video><h2>기존 공통 파지 이후 추가 회전, 원래 속도</h2><video controls preload="metadata" src="additional_turn_1x.mp4"></video><p>앞의 기존 경로에서는 약63도까지 돌아가고, 추가 경로가 나머지를 진행한다. 완벽한 모델 자세와 접촉 정보를 이용해 계산한 관절 목표를 저장 후 재생했으며 실물용 제어기는 아니다.</p><h2>추가 회전 전과 후</h2><div class="pair"><img src="before_extension_top.png" alt="추가 회전 전 저장 자세"><img src="end_top.png" alt="추가 회전 후 저장 자세"></div><p><a href="../review.json">실패 항목을 포함한 전체 재생 결과</a> / <a href="manifest.json">영상 출처</a> / <a href="../../offline_review.html">전체 작업 상태</a></p></html>'''
    (out/'index.html').write_text(page,encoding='utf-8');print(json.dumps(dict(videos=videos,yaw=v['final_yaw_from_placement_deg'],quarter_common=report['quarter_and_common_grip_passed'])),flush=True)
if __name__=='__main__':main()
