"""Render saved uninterrupted motion; no interpolation of cube poses or devices."""
import os
os.environ['JAX_PLATFORMS']='cpu'
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json,shutil,subprocess
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display,extra,png

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    source=ROOT/'continuous_handoff';review=json.loads((source/'review.json').read_bytes());model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';m=mujoco.MjModel.from_binary_path(str(model));display_physical_cube(m);m.vis.global_.offwidth=1000;m.vis.global_.offheight=800
    out=source/'evidence';out.mkdir(exist_ok=True);trajectory=source/'trajectory.npz';z=np.load(trajectory);q=z['qpos'];times=z['samples']*float(z['goal_period_s']);end=np.load(source/'end_state.npz')
    assert np.all(np.diff(times)>=0) and abs(times[-1]-462.45)<.001
    for g in range(m.ngeom):
        name=m.body(int(m.geom_bodyid[g])).name;color=next((v for k,v in {'if_':[.22,.45,.83,1],'mf_':[.25,.65,.4,1],'rf_':[.88,.69,.14,1],'th_':[.83,.3,.3,1]}.items() if name.startswith(k)),None)
        if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
        if name=='cube':m.geom_matid[g]=-1;m.geom_rgba[g]=[.9,.9,.88,1]
    renderer=mujoco.Renderer(m,height=720,width=960);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.distance=.32;cam.lookat[:]=[.14,-.025,-.015];cam.azimuth=35;cam.elevation=-38;opt=mujoco.MjvOption();opt.geomgroup[:]=[1,1,1,0,0,0];d=mujoco.MjData(m);cube=m.geom('cube').id
    def frame(pos):
        d.qpos[:]=pos;d.qvel[:]=0;mujoco.mj_forward(m,d);C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);renderer.update_scene(d,camera=cam,scene_option=opt);cube_display(renderer.scene,C,B,True)
        for axis,sign,color in ((0,1,[.85,.16,.13,1]),(0,-1,[1.,.45,.07,1]),(1,1,[.12,.3,.85,1]),(1,-1,[.13,.7,.3,1]),(2,1,[.95,.95,.95,1]),(2,-1,[.95,.82,.08,1])):
            size=np.full(3,.023);size[axis]=.00015;p=np.zeros(3);p[axis]=sign*.0277;extra(renderer.scene,mujoco.mjtGeom.mjGEOM_BOX,size,C+B@p,B,color)
        return renderer.render()
    images=[]
    for label,az,el in (('side',35,-38),('top',90,-78)):
        cam.azimuth=az;cam.elevation=el;file='final_'+label+'.png';png(out/file,frame(end['qpos']));images.append(file)
    cam.azimuth=35;cam.elevation=-38;ffmpeg=shutil.which('ffmpeg');assert ffmpeg;videos=[]
    for name,start,stop,speed,fps in (('whole_path_12x.mp4',0.,float(times[-1]),12.,12.),('last_regrasp_1x.mp4',float(times[-1]-30),float(times[-1]),1.,8.)):
        desired=np.arange(start,stop, speed/fps);indices=np.clip(np.searchsorted(times,desired,side='left'),0,len(times)-1);file=out/name
        proc=subprocess.Popen([ffmpeg,'-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','960x720','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(file)],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        try:
            for index in indices:proc.stdin.write(frame(q[index]).tobytes())
            proc.stdin.close();error=proc.stderr.read().decode(errors='replace');code=proc.wait()
            if code:raise RuntimeError(error[-1000:])
        finally:
            if proc.poll() is None:proc.terminate();proc.wait()
        v=dict(file=name,sha256=sha(file),frames=len(indices),fps=fps,seconds=len(indices)/fps,source_start_s=start,source_end_s=stop,playback_speed=speed,source_sample_indices=indices.tolist(),maximum_source_sampling_error_s=float(max(abs(times[indices]-desired))),source_trajectory_sha256=sha(trajectory));videos.append(v);print(json.dumps({k:v for k,v in v.items() if k!='source_sample_indices'}),flush=True)
    renderer.close();manifest=dict(status='saved_uninterrupted_physics_trajectory_render',source_review=(source/'review.json').relative_to(R).as_posix(),source_review_sha256=sha(source/'review.json'),source_trajectory_sha256=sha(trajectory),source_end_state_sha256=sha(source/'end_state.npz'),source_model_sha256=sha(model),images=images,videos=videos,no_pose_interpolation=True,nominal_simulation_only=True,physical_video=False,devices_opened=False,hardware_execution_allowed=False);(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>전체 회전과 재파지의 연속 시뮬레이션</title><style>body{font:17px/1.8 system-ui;max-width:1000px;margin:30px auto;padding:0 20px;color:#24313c}video{width:100%}.pair{display:flex;gap:10px}.pair img{width:49%}.note{background:#fff0ce;padding:18px}a{color:#145c9e}</style><h1>전체 회전과 재파지의 연속 시뮬레이션</h1><p class="note">기준 모델 한 조건의 계산 결과다. 실제 로봇 영상이 아니다. 처음 배치부터 마지막 파지까지 큐브 자세를 중간에 초기화하지 않았다. 최종 회전은 약85.94도로, 목표90±5도 범위에 들어왔다. 정확히90도 및 실물 성공은 아직 확인하지 않았다.</p><p>파랑=검지, 초록=중지, 노랑=약지, 빨강=엄지. 주황 원통은 다른 손이 윗단을 돌릴 공간이며 오른손 회전과 재파지 중에는 사용해도 된다. 다른 손 접근 전에는 비운다.</p><h2>처음부터 끝까지 12배속</h2><video controls preload="metadata" src="whole_path_12x.mp4"></video><p>6166개 목표, 모델 동작 시간462.45초. 모든1ms 상태에서 원래 동적 기준을 검사했다. 마지막4.05초의 윗단 공간 최소 여유는0.89mm, 윗면 기울기는1.69도다.</p><h2>마지막30초 원래 속도</h2><video controls preload="metadata" src="last_regrasp_1x.mp4"></video><div class="pair"><img src="final_side.png" alt="최종 파지를 옆에서 본 모델"><img src="final_top.png" alt="최종 파지와 윗단 공간을 위에서 본 모델"></div><h2>남은 확인</h2><p>관절 목표를 그대로 유지한 하중 시험은0, ±0.003, +0.01N·m를 통과했다. −0.01N·m에서는 몸통이4.38도 움직여3도 기준에 미달했다. 실제 왼손의 힘과 형상, 배치 오차, 다음 회전 연결, 실물 재현은 아직 검증하지 않았다. 모델의0.15N·m 한도는 실제450mA를 환산한 값이 아니다.</p><p><a href="manifest.json">영상과 상태 출처</a> / <a href="../review.json">연속 경로와 하중 시험 결과</a> / <a href="../../temporal_handoff_review.html">양손 동작 순서</a> / <a href="../../offline_review.html">전체 진행 상황</a></p></html>''';(out/'index.html').write_text(page,encoding='utf-8')
if __name__=='__main__':main()
