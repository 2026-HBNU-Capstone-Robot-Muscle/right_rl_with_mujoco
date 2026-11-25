"""Source-bound partial grasp progress: saved poses only, no devices or motion."""
import os
os.environ['JAX_PLATFORMS']='cpu'
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json,shutil,subprocess
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display,extra,png

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    parent=json.loads((ROOT/'near_quarter_full_replay/review.json').read_bytes());model=R/parent['source_model'];assert sha(model)==parent['source_model_sha256'];m=mujoco.MjModel.from_binary_path(str(model));m.vis.global_.offwidth=1000;m.vis.global_.offheight=800;display_physical_cube(m);out=ROOT/'countergrip_progress';out.mkdir(exist_ok=True)
    for g in range(m.ngeom):
        name=m.body(int(m.geom_bodyid[g])).name;color=next((v for k,v in {'if_':[.22,.45,.83,1],'mf_':[.25,.65,.4,1],'rf_':[.88,.69,.14,1],'th_':[.83,.3,.3,1]}.items() if name.startswith(k)),None)
        if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
        if name=='cube':m.geom_matid[g]=-1;m.geom_rgba[g]=[.9,.9,.88,1]
    renderer=mujoco.Renderer(m,height=720,width=960);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.distance=.32;cam.lookat[:]=[.14,-.025,-.015];opt=mujoco.MjvOption();opt.geomgroup[:]=[1,1,1,0,0,0];d=mujoco.MjData(m);cube=m.geom('cube').id
    def frame(q):
        d.qpos[:]=q;d.qvel[:]=0;mujoco.mj_forward(m,d);C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);renderer.update_scene(d,camera=cam,scene_option=opt);cube_display(renderer.scene,C,B,True)
        for axis,sign,color in ((0,1,[.85,.16,.13,1]),(0,-1,[1.,.45,.07,1]),(1,1,[.12,.3,.85,1]),(1,-1,[.13,.7,.3,1]),(2,1,[.95,.95,.95,1]),(2,-1,[.95,.82,.08,1])):
            size=np.full(3,.023);size[axis]=.00015;p=np.zeros(3);p[axis]=sign*.0277;extra(renderer.scene,mujoco.mjtGeom.mjGEOM_BOX,size,C+B@p,B,color)
        return renderer.render()
    cases=[('upright_grip','seed0_source1_lift2_normal1.5_n850','verified_end_state.npz','verified_trajectory.npz'),('thumb_lowered','seed0_source1_lift0_normal1.5_n900_resume_lower_staged','end_state.npz',None),('both_lowered','seed0_source1_lift0_normal2.5_n900_resume_lower_staged_freebase','end_state.npz','trajectory.npz')];evidence=[];ffmpeg=shutil.which('ffmpeg')
    if not ffmpeg:raise RuntimeError('ffmpeg required')
    for label,tag,state_name,trajectory_name in cases:
        folder=ROOT/'countergrip_level'/tag;review=folder/'review.json';r=json.loads(review.read_bytes());assert r['checkpoint90_and_new_grip_passed'] is False;z=np.load(folder/state_name);images=[]
        for view,az,el in (('side',35,-38),('top',90,-78)):
            cam.azimuth=az;cam.elevation=el;file=f'{label}_{view}.png';png(out/file,frame(z['qpos']));images.append(file)
        record=dict(label=label,source_review=review.relative_to(R).as_posix(),source_review_sha256=sha(review),source_state_sha256=sha(folder/state_name),images=images,yaw_deg=r['final_yaw_deg'],checkpoint_complete=False)
        if trajectory_name:
            path=folder/trajectory_name;data=np.load(path);selection=np.arange(0,len(data['qpos']),4);file=out/(label+'_4x.mp4');cam.azimuth=35;cam.elevation=-38;fps=4/.6;proc=subprocess.Popen([ffmpeg,'-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','960x720','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(file)],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
            try:
                for index in selection:proc.stdin.write(frame(data['qpos'][index]).tobytes())
                proc.stdin.close();error=proc.stderr.read().decode(errors='replace');code=proc.wait()
                if code:raise RuntimeError(error[-1000:])
            finally:
                if proc.poll() is None:proc.terminate();proc.wait()
            record['video']=dict(file=file.name,sha256=sha(file),frames=len(selection),fps=fps,seconds=len(selection)/fps,playback_speed=4,source_trajectory_sha256=sha(path))
        evidence.append(record);print(json.dumps(dict(label=label,yaw=r['final_yaw_deg'],video=record.get('video'))),flush=True)
    renderer.close();manifest=dict(status='partial_countergrip_progress_saved_states_rendered',source_model_sha256=sha(model),evidence=evidence,all_shown_cases_are_incomplete=True,colors_and_upper_cylinder_are_display_only=True,physical_video=False,devices_opened=False,hardware_execution_allowed=False);(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>회전 뒤 재파지 개발 상태</title><style>body{font:16px/1.7 system-ui;max-width:1050px;margin:30px auto;padding:0 20px;color:#24313c}.note{background:#fff0cd;padding:15px}.pair{display:flex;gap:10px}.pair img{width:49%}video{width:100%}</style><h1>회전 뒤 재파지 개발 상태</h1><p class="note">전부 저장된 시뮬레이션 상태다. 아래 세 경로는 아직 목표 전체를 통과하지 못했다. 실제 로봇의 성공 영상이 아니며, 손가락 형상과 관절 한계는 바꾸지 않았다.</p><p>파랑=검지, 초록=중지, 노랑=약지, 빨강=엄지. 주황 원통은 다른 손이 윗단을 돌릴 공간이다.</p><h2>검지와 엄지로 잡고 수평 맞추기</h2><p>약89.19도와 윗면 기울기0.76도까지 맞추고 고정했지만 손가락 끝 외형이 윗단 공간에 남아 있다.</p><video controls preload="metadata" src="upright_grip_4x.mp4"></video><div class="pair"><img src="upright_grip_top.png" alt="수평을 맞춘 모델 파지"><img src="thumb_lowered_top.png" alt="엄지만 낮춘 모델 파지"></div><h2>받침을 옮기며 검지도 낮추기</h2><p>손가락 위치는 낮아지고 접촉힘은 유지되지만, 큐브 방향이 약83.58도와 기울기5.69도로 변해 아직 미통과다. 힘을 유지하는 것과 원하는 방향을 유지하는 것을 별도로 검사하고 있다.</p><video controls preload="metadata" src="both_lowered_4x.mp4"></video><div class="pair"><img src="both_lowered_side.png" alt="받침 이동 뒤 옆에서 본 모델"><img src="both_lowered_top.png" alt="받침 이동 뒤 위에서 본 모델"></div><p><a href="manifest.json">그림과 영상의 출처</a> / <a href="../offline_review.html">전체 결과와 남은 작업</a></p></html>''';(out/'index.html').write_text(page,encoding='utf-8')
if __name__=='__main__':main()
