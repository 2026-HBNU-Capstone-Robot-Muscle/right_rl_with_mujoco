"""Replay the nominal admitted model and render the middle probe, without devices."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,subprocess,shutil,sys,io,contextlib
from pathlib import Path
import numpy as np,mujoco
import screen_natural_grasp_hold_20261030 as source
from render_canonical_grip_evidence_20261105 import display_physical_cube,cube_display,extra,png
from middle_probe_plan_20261106 import R,O,CANDIDATE,validate_candidate,sha
def main():
    out=O/'evidence';out.mkdir(exist_ok=True);a=validate_candidate();holder={};keyframes={};video_q=[];video_steps=[];steps=0;original_build=source.build;original_step=mujoco.mj_step;old_argv=sys.argv
    def captured_build(*args,**kwargs):
        m,manifest=original_build(*args,**kwargs);holder['model']=m;return m,manifest
    def captured_step(m,d,*args,**kwargs):
        nonlocal steps
        original_step(m,d,*args,**kwargs)
        if holder.get('model') is not m:return
        if args or kwargs:raise RuntimeError('Unexpected batched integration in source recorder')
        steps+=1
        for sample in (927,1042,1110):
            if steps==2000+sample*75:keyframes[sample]=d.qpos.copy()
        if 2000+860*75<=steps<2000+1110*75 and (steps-(2000+860*75))%125==0:video_q.append(d.qpos.copy());video_steps.append(steps)
    try:
        source.build=captured_build;mujoco.mj_step=captured_step;sys.argv=['source','--candidate',(O/'evaluation_alias.npz').relative_to(R).as_posix(),'--output-folder',(out/'source_replay').relative_to(R).as_posix(),'--front','-10','--nominal-only','--cube-seed-json','right_hand_rotation_20261030/stable_support_research/natural_recess_hypothesis_seed.json','--seat-up']
        with contextlib.redirect_stdout(io.StringIO()):source.main()
    finally:source.build=original_build;mujoco.mj_step=original_step;sys.argv=old_argv
    m=holder['model'];recorded=json.loads((out/'source_replay/review.json').read_bytes());assert recorded['all_passed'];nominal=json.loads((O/'nominal/review.json').read_bytes())['cases'][0];replay=recorded['cases'][0];comparison={k:abs(replay[k]-nominal[k]) for k in ('final_yaw_from_placement_deg','center_translation_mm','max_tracking_rad','max_cube_mm')};assert max(comparison.values())<1e-9,comparison
    assert steps==2000+len(a['goals'])*75 and len(keyframes)==3 and len(video_q)==150
    mujoco.mj_saveModel(m,str(out/'model.mjb'),None);np.savez_compressed(out/'keyframe_states.npz',goal_samples=[927,1042,1110],qpos=np.stack([keyframes[i] for i in (927,1042,1110)]));np.savez_compressed(out/'video_states.npz',physics_steps=video_steps,qpos=video_q,timestep_s=.001)
    display_physical_cube(m);m.vis.global_.offwidth=1000;m.vis.global_.offheight=800
    for g in range(m.ngeom):
        name=m.body(int(m.geom_bodyid[g])).name;color=next((v for k,v in {'if_':[.22,.45,.83,1],'mf_':[.25,.65,.4,1],'rf_':[.88,.69,.14,1],'th_':[.83,.3,.3,1]}.items() if name.startswith(k)),None)
        if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
        if name=='cube':m.geom_matid[g]=-1;m.geom_rgba[g]=[.9,.9,.88,1]
    renderer=mujoco.Renderer(m,height=720,width=960);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.distance=.32;cam.lookat[:]=[.14,-.025,-.015];cam.azimuth=135;cam.elevation=-28;opt=mujoco.MjvOption();opt.geomgroup[:]=[1,1,1,0,0,0];cube=m.geom('cube').id
    d=mujoco.MjData(m)
    def frame(qpos,sweep=False):
        d.qpos[:]=qpos;d.qvel[:]=0;mujoco.mj_forward(m,d);C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);renderer.update_scene(d,camera=cam,scene_option=opt);cube_display(renderer.scene,C,B,sweep)
        for axis,sign,color in ((0,1,[.85,.16,.13,1]),(0,-1,[1.,.45,.07,1]),(1,1,[.12,.3,.85,1]),(1,-1,[.13,.7,.3,1]),(2,1,[.95,.95,.95,1]),(2,-1,[.95,.82,.08,1])):
            others=[i for i in range(3) if i!=axis]
            for u in (-1,0,1):
              for v in (-1,0,1):
                size=np.full(3,.0083);size[axis]=.00015;p=np.zeros(3);p[axis]=sign*.0277;p[others]=np.array([u,v])*.055/3;extra(renderer.scene,mujoco.mjtGeom.mjGEOM_BOX,size,C+B@p,B,color)
        return renderer.render()
    for name,sample in (('before',927),('middle_folded',1042),('returned',1110)):png(out/(name+'.png'),frame(keyframes[sample],True))
    ffmpeg=shutil.which('ffmpeg');assert ffmpeg;proc=subprocess.Popen([ffmpeg,'-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','960x720','-framerate','8','-i','-','-an','-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(out/'middle_probe_1x.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    try:
        for qpos in video_q:proc.stdin.write(frame(qpos).tobytes())
        proc.stdin.close();error=proc.stderr.read().decode(errors='replace');code=proc.wait()
        if code:raise RuntimeError(error[-1000:])
    finally:
        if proc.poll() is None:proc.terminate();proc.wait()
    renderer.close();manifest=dict(status='nominal_free_cube_middle_probe_render',candidate_sha256=sha(CANDIDATE),model_sha256=sha(out/'model.mjb'),source_script_sha256=sha(Path(__file__)),recorded_directly_from_original_1ms_checker=True,source_review_sha256=sha(out/'source_replay/review.json'),nominal_replay_metric_differences=comparison,recorded_physics_steps=steps,source_start_goal=860,source_end_goal=1110,video_frames=len(video_q),video_fps=8,video_seconds=len(video_q)/8,playback_speed=1,no_cube_pose_resets_during_motion=True,source_grasp_return_open_complete_before_display=True,not_a_physical_video=True,new_common_grip_verified=False,full_upper_layer_access_not_yet_verified=True,devices_opened=False,files={p.name:sha(p) for p in out.glob('*') if p.suffix in ('.png','.mp4','.npz')});(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>중지 공간 확인 동작</title><style>body{font:17px/1.7 system-ui;max-width:1100px;margin:30px auto;padding:0 20px}.note{background:#fff0ce;padding:15px}video{width:100%}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}figure{margin:0}img{width:100%}figcaption{padding:8px}@media(max-width:800px){.grid{grid-template-columns:1fr}}</style><h1>엄지를 고정하고 중지만 접었다 돌아오기</h1><p>초록=중지, 빨강=엄지, 파랑=검지, 노랑=약지.</p><p class="note">실제 로봇 영상이 아닌 기준 모델의 자유 큐브 시뮬레이션이다. 이번 동작은 중지 쪽 공간과 큐브 유지를 관찰한다. 완성된 하부 파지나 윗단 회전은 아직 확인하지 않았다. 정지 그림의 주황 원통은 윗단 회전 공간 표시다.</p><video controls preload="metadata" src="middle_probe_1x.mp4"></video><p>기존 파지 고정 → 중지만 약 3.6초 접기 → 약 5초 고정 → 중지 복귀. 영상은 원래 속도다. 전체 프로그램은 빈손 준비와 큐브 제거를 포함해 약 4분 걸린다.</p><div class="grid"><figure><img src="before.png"><figcaption>기존 파지</figcaption></figure><figure><img src="middle_folded.png"><figcaption>중지를 접고 고정</figcaption></figure><figure><img src="returned.png"><figcaption>기존 파지로 복귀</figcaption></figure></div><p><a href="../operator_guide.md">실행 순서</a> / <a href="manifest.json">출처와 재생 확인</a></p></html>''';(out/'index.html').write_text(page,encoding='utf-8');print(json.dumps({k:v for k,v in manifest.items() if k!='files'}),flush=True)
if __name__=='__main__':main()
