"""Render and explain the conditional free-model clearance suffix."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json
import html
import shutil
import subprocess
import numpy as np
import mujoco
from native_cycle import OUT, assets, restore, save_json, sha
from render_canonical_grip_evidence_20261105 import display_physical_cube, cube_display, extra, png


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'RF_clear_work_grip_evidence'
    folder.mkdir(exist_ok=True)
    joined_path=base/'RF_clear_work_grip_joined_suffix/review.json';joined=json.loads(joined_path.read_text())
    index_path=base/'RF_index_clearance_margin2/trial0/review.json';index=json.loads(index_path.read_text())
    states=[('before','기존 약지 지지 자세',OUT/joined['source']),
        ('index','검지 끝 공간 확보',index_path.parent/'end_state.npz'),
        ('clear','검지·약지 공간 확보',joined_path.parent/'end_state.npz')]
    model,_,_,_,provenance=assets('active_ccd50');display=display_physical_cube(model)
    model.vis.global_.offwidth=1000;model.vis.global_.offheight=800
    palette={'if_':[.22,.45,.83,1.],'mf_':[.25,.65,.4,1.],'rf_':[.88,.69,.14,1.],'th_':[.83,.3,.3,1.]}
    for geom in range(model.ngeom):
        body=model.body(int(model.geom_bodyid[geom])).name or ''
        for prefix,color in palette.items():
            if body.startswith(prefix):model.geom_matid[geom]=-1;model.geom_rgba[geom]=color
        if body=='cube':model.geom_matid[geom]=-1;model.geom_rgba[geom]=[.94,.94,.90,1.]
    renderer=mujoco.Renderer(model,height=600,width=800);camera=mujoco.MjvCamera()
    camera.type=mujoco.mjtCamera.mjCAMERA_FREE;camera.distance=.27;camera.lookat[:]=[.145,-.04,-.013]
    option=mujoco.MjvOption();option.geomgroup[:]=[1,1,1,0,0,0];cube=model.geom('cube').id
    def scene(data,sweep=True):
        renderer.update_scene(data,camera=camera,scene_option=option)
        C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3);cube_display(renderer.scene,C,B,False)
        for axis,sign,color in ((0,1,[.85,.16,.13,1]),(0,-1,[1,.45,.07,1]),(1,1,[.12,.3,.85,1]),(1,-1,[.13,.7,.3,1]),(2,1,[.95,.95,.95,1]),(2,-1,[.95,.82,.08,1])):
            others=[i for i in range(3) if i!=axis]
            for u in (-1,0,1):
                for v in (-1,0,1):
                    size=np.full(3,.0083);size[axis]=.00015;point=np.zeros(3);point[axis]=sign*.0277;point[others]=np.array([u,v])*.055/3
                    extra(renderer.scene,mujoco.mjtGeom.mjGEOM_BOX,size,C+B@point,B,color)
        if sweep:
            matrix=np.column_stack([B[:,0],B[:,2],-B[:,1]])
            extra(renderer.scene,mujoco.mjtGeom.mjGEOM_CYLINDER,[.0275*np.sqrt(2),.055/6,0],C+B@np.array([0.,-.055/3,0.]),matrix,[.85,.2,.65,.20])
    images=[]
    for name,title,path in states:
        data=restore(model,dict(np.load(path,allow_pickle=False)))
        for view,azimuth,elevation in (('front',130,-30),('side',35,-30)):
            camera.azimuth=azimuth;camera.elevation=elevation;scene(data)
            file=f'{name}_{view}.png';png(folder/file,renderer.render())
            images.append(dict(file=file,title=title,view=view,source=path.relative_to(OUT).as_posix(),source_sha256=sha(path)))
    videos=[];ffmpeg=shutil.which('ffmpeg')
    if ffmpeg:
        path=joined_path.parent/'trajectory.npz';trajectory=np.load(path,allow_pickle=False)['qpos'];indices=np.unique(np.r_[np.arange(0,len(trajectory),8),len(trajectory)-1])
        camera.azimuth=35;camera.elevation=-30;data=restore(model,dict(np.load(states[0][2],allow_pickle=False)))
        output=folder/'conditional_suffix_4x.mp4';fps=4/(8*.075)
        process=subprocess.Popen([ffmpeg,'-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','800x600','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(output)],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        try:
            for index_number in indices:
                data.qpos[:]=trajectory[index_number];mujoco.mj_forward(model,data);scene(data);process.stdin.write(renderer.render().tobytes())
            process.stdin.close();error=process.stderr.read().decode(errors='replace');code=process.wait()
            if code:raise RuntimeError(error[-1500:])
        finally:
            if process.poll() is None:process.terminate();process.wait()
        videos.append(dict(file=output.name,source=path.relative_to(OUT).as_posix(),source_sha256=sha(path),frames=len(indices),playback_speed=4,frame_indices=indices.tolist()))
    renderer.close()
    manifest=dict(status='conditional_clear_work_grip_model_visualization',model_provenance=provenance,images=images,videos=videos,
        source_reports_sha256={path.relative_to(OUT).as_posix():sha(path) for path in (joined_path,index_path)},display=display,
        scientific_model_renders_not_photos=True,sweep_is_visual_overlay_only=True,devices_opened=0)
    save_json(folder/'manifest.json',manifest)
    (folder/'source_render_RF_clear_work_grip.py').write_bytes(Path(__file__).read_bytes())
    figures=''.join('<figure><img src="'+item['file']+'"><figcaption>'+html.escape(item['title'])+' · '+('정면' if item['view']=='front' else '측면')+'</figcaption></figure>' for item in images)
    video=''.join('<video controls preload="metadata" src="'+item['file']+'"></video><p>저장한 자유 동역학 경로, 약 4배속. 출발은 별도로 배치한 모델 파지다.</p>' for item in videos)
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>윗단 공간을 비운 파지: 모델 검토</title>
<style>body{font:16px/1.7 system-ui,sans-serif;max-width:1160px;margin:28px auto;padding:0 20px;color:#173d36;background:#f5f9f7}section,figure{background:white;border:1px solid #caddd3;padding:16px;margin:16px 0}h1{font-size:28px}.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:16px}figure{padding:0;margin:0}img,video{width:100%;display:block}figcaption{padding:10px}td,th{padding:9px;border-bottom:1px solid #d5e3dc;text-align:left}table{border-collapse:collapse;width:100%}a{color:#126b61}.note{border-left:5px solid #d59736;padding:14px;background:#fff3d7}@media(max-width:720px){.grid{grid-template-columns:1fr}}</style>
<body><h1>윗단 공간을 비운 파지까지 모델 경로를 연결했다</h1>
<p class="note">초기 배치에서 회전해 도달한 결과는 아니다. 별도로 배치한 약지 지지 파지에서 검지 끝을 비우고 약지를 옮기는 두 구간을 연결했다. 실물 테스트와 다음 90도 회전은 남아 있다.</p>
<p>파랑: 검지 · 초록: 중지 · 노랑: 약지 · 빨강: 엄지. 분홍 원통은 윗단 한 층을 90도 돌릴 때 차지하는 영역이다. 그림과 영상은 모델 결과다.</p>
<section><h2>상태를 초기화하지 않은 두 구간</h2><p>검지 공간 확보 90목표 → 약지 이동과 유지 302목표. 총 392목표 동안 원래 관절 범위·충돌 형상·2틱/75ms 조건을 유지했다.</p>
<table><tr><th>마지막 4.05초</th><th>모델 결과</th></tr>'''+f'''<tr><td>전체 손 지지</td><td>{joined['whole_hand_support_fraction']*100:.0f}%</td></tr><tr><td>윗단 공간 인증</td><td>{joined['certified_samples']}/4050상태, 하한 {joined['upper_certificate_minimum_lower_mm']:.3f}mm</td></tr><tr><td>90도 목표 자세 오차</td><td>{joined['final_global90_error_deg']:.3f}도</td></tr><tr><td>공통 파지 위치 오차</td><td>{joined['common_station_error_mm']:.3f}mm</td></tr><tr><td>유지 중 변화</td><td>{joined['hold_angle_drift_deg']:.3f}도 / {joined['hold_center_drift_mm']:.3f}mm</td></tr>'''+'''</table>
<p>마지막 자세에서는 약지가 큐브에 닿지 않는다. 검지 뿌리·검지 마디·엄지의 모델 접촉으로 지지가 유지됐다. 약지로 세 면을 고정한 결과나 왼손 회전 하중을 버틴 결과로 해석하지 않는다.</p></section>'''+video+'<div class="grid">'+figures+'''</div>
<section><h2>남은 연결</h2><p>기존 초기 배치의 회전 경로는 현재 모델에서 미달이다. 이번에 확보한 작업 파지로 들어오는 경로와 그 파지에서 다음 회전으로 나오는 경로를 검증해야 한다. 완전한 모델 관측과 가정한 마찰·토크 한도 아래의 결과이며 실제 450mA나 카메라 등록을 검증한 것은 아니다.</p><p><a href="../RF_clear_work_grip_joined_suffix/review.json">연결 재검사</a> · <a href="../RF_clearance_free_arrivals/review.json">약지 이동 비교</a> · <a href="../RF_upper_blockers/review.json">기존 공간 침범 근거</a> · <a href="manifest.json">그림·영상 출처</a> · <a href="../whole_hand_preflight_v2/index.html">전체 진행 기록</a></p></section></body></html>'''
    (folder/'index.html').write_text(page,encoding='utf-8')
    print(json.dumps(dict(images=len(images),videos=len(videos),report=folder.relative_to(OUT).as_posix()+'/index.html')),flush=True)


if __name__=='__main__':main()
