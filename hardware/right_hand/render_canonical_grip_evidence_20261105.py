"""Render saved simulated poses/trajectory; no capture or device access."""
import os
os.environ['JAX_PLATFORMS']='cpu'
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,shutil,subprocess,struct,zlib,html,hashlib
from pathlib import Path
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore
from dynamic_common_grip_search_20261105 import CP,ROOT,R,SEED
from cube_task_frame_20261105 import working_rotation

def png(path,rgb):
    def chunk(k,b):return struct.pack('>I',len(b))+k+b+struct.pack('>I',zlib.crc32(k+b)&0xffffffff)
    h,w,_=rgb.shape;raw=b''.join(b'\0'+row.tobytes() for row in rgb);path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))

def extra(scene,kind,size,pos,mat,rgba):
    if scene.ngeom>=scene.maxgeom:raise RuntimeError('visual scene full')
    mujoco.mjv_initGeom(scene.geoms[scene.ngeom],kind,np.asarray(size,float),np.asarray(pos,float),np.asarray(mat,float).reshape(9),np.asarray(rgba,np.float32));scene.ngeom+=1

def display_physical_cube(m):
    """Render the actual55mm box; hide the original70mm visual and goal."""
    physics={k:getattr(m,k).copy() for k in ('geom_size','geom_contype','geom_conaffinity','body_mass','body_inertia')}
    cube=m.geom('cube').id;cube_body=int(m.geom_bodyid[cube]);hidden=[]
    for g in range(m.ngeom):
        body=int(m.geom_bodyid[g])
        if int(m.body_mocapid[body])>=0 or (body==cube_body and g!=cube):m.geom_group[g]=5;hidden.append(g)
    m.geom_group[cube]=2
    for k,v in physics.items():
        if not np.array_equal(getattr(m,k),v):raise RuntimeError('Display changed physical model property '+k)
    return dict(physical_cube_side_mm=(2*m.geom_size[cube]*1000).tolist(),source70mm_cube_visual_hidden=True,mocap_goal_display_hidden=True,hidden_geoms=hidden,physics_properties_unchanged=True)

def cube_display(scene,C,B,sweep=False,top_face_id=5):
    corners=np.array([[x,y,z] for x in (-.0275,.0275) for y in (-.0275,.0275) for z in (-.0275,.0275)])
    pairs=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(corners[i]!=corners[j])==1]
    for i,j in pairs:
        p=C+B@corners[i];q=C+B@corners[j];axis=(q-p)/np.linalg.norm(q-p);ref=np.array([1.,0,0]) if abs(axis[0])<.8 else np.array([0.,1,0]);x=np.cross(ref,axis);x/=np.linalg.norm(x);y=np.cross(axis,x);G=np.column_stack([x,y,axis]);extra(scene,mujoco.mjtGeom.mjGEOM_CAPSULE,[.00025,np.linalg.norm(q-p)/2,0],(p+q)/2,G,[.12,.12,.12,1])
    if sweep:
        W=working_rotation(B,top_face_id);G=W@np.array([[1.,0,0],[0,0,1],[0,-1,0]])
        extra(scene,mujoco.mjtGeom.mjGEOM_CYLINDER,[.0275*np.sqrt(2),.055/6,0],C+W@np.array([0.,-.055/3,0]),G,[1.,.55,.08,.24])

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--folder',type=Path,default=ROOT/'full_placement/front-10');ap.add_argument('--tag',default='evidence_v1');ap.add_argument('--video',action='store_true');a=ap.parse_args();out=ROOT/a.tag;out.mkdir(parents=True,exist_ok=True);folder=a.folder
    m=mujoco.MjModel.from_binary_path(str(folder/'g1_f0.3/model.mjb'));m.vis.global_.offwidth=1000;m.vis.global_.offheight=800
    colors={'if_':[.22,.45,.83,1],'mf_':[.25,.65,.4,1],'rf_':[.88,.69,.14,1],'th_':[.83,.3,.3,1]}
    for g in range(m.ngeom):
        name=m.body(int(m.geom_bodyid[g])).name;color=next((v for k,v in colors.items() if name.startswith(k)),None)
        if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
        if name=='cube':m.geom_matid[g]=-1;m.geom_rgba[g]=[.88,.89,.86,1]
    display=display_physical_cube(m);renderer=mujoco.Renderer(m,height=720,width=960);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.distance=.34;cam.lookat[:]=[.14,-.025,-.015];opt=mujoco.MjvOption();opt.geomgroup[:]=[1,1,1,0,0,0];cube=m.geom('cube').id
    labels=[('forward_reference','기존 회전 종료',CP/'end_state.npz'),('twisted_regrasp','이전 후보: 방향이 틀어짐',SEED),('new_nominal','새 파지: 기준 조건',folder/'g1_f0.3/end_state.npz'),('new_weak','새 파지: 약한 구동과 낮은 마찰',folder/'g0.5_f0.2/end_state.npz')];images=[]
    for name,title,path in labels:
        if not path.exists():continue
        state=np.load(path);d=restore(m,state);top_face=int(state['top_face_id']) if 'top_face_id' in state else 5
        for view,az,el in [('front',135,-28),('side',35,-28),('top',90,-78)]:
            cam.azimuth=az;cam.elevation=el;renderer.update_scene(d,camera=cam,scene_option=opt);cube_display(renderer.scene,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3),True,top_face);filename=f'{name}_{view}.png';png(out/filename,renderer.render());images.append(dict(file=filename,label=title,view=view,source=str(path.relative_to(R)),top_face_id=top_face))
    videos=[];ffmpeg=shutil.which('ffmpeg')
    if a.video and ffmpeg:
        data=np.load(folder/'g1_f0.3/trajectory.npz');samples=data['samples'];states=data['qpos'];video_top=int(data['top_face_id']) if 'top_face_id' in data else 5;d=mujoco.MjData(m);cam.azimuth=35;cam.elevation=-28;cam.distance=.32
        for name,selection,fps,speed in [('regrasp_and_hold_1x',samples>=1417,1/.15,1),('continuous_run_4x',np.ones(len(samples),bool),4/.15,4)]:
            path=out/(name+'.mp4');args=[ffmpeg,'-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','960x720','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(path)];process=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
            n=0
            try:
                for sample,q in zip(samples[selection],states[selection]):
                    d.qpos[:]=q;d.qvel[:]=0;mujoco.mj_forward(m,d);renderer.update_scene(d,camera=cam,scene_option=opt);cube_display(renderer.scene,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3),sample>=int(samples[-1])-134,video_top);process.stdin.write(renderer.render().tobytes());n+=1
                process.stdin.close();error=process.stderr.read().decode(errors='replace');code=process.wait()
                if code:raise RuntimeError('video export failed: '+error[-1500:])
            finally:
                if process.poll() is None:process.terminate();process.wait()
            videos.append(dict(file=path.name,frames=n,playback_speed=speed,source='saved free-cube trajectory',video_seconds=n/fps));print(json.dumps(videos[-1]),flush=True)
    renderer.close();source_review=folder/'review.json';source_data=json.loads(source_review.read_bytes());manifest=dict(status='saved_simulation_evidence_rendered',candidate_sha256=source_data['candidate_sha256'],source_review=str(source_review.relative_to(R)),source_review_sha256=hashlib.sha256(source_review.read_bytes()).hexdigest(),images=images,videos=videos,display_model=display,ffmpeg_found=bool(ffmpeg),colors=dict(index='blue',middle='green',ring='yellow',thumb='red'),orange_cylinder_is_display_only_upper_layer_sweep=True,physics_and_hardware_not_modified=True,not_physical_video=True,articulated_layer_not_simulated=True,physical_exact90_not_verified=True,devices_opened=False)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    cards=''.join('<figure><img src="'+v['file']+'" loading="lazy"><figcaption>'+html.escape(v['label'])+' / '+v['view']+'</figcaption></figure>' for v in images);vid=''.join('<section><h2>'+('재파지와 유지, 원래 속도' if v['playback_speed']==1 else '전체 경로, 4배속')+'</h2><video controls preload="metadata" src="'+v['file']+'"></video></section>' for v in videos)
    text='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>공통 파지 시뮬레이션 검토</title><style>body{font:16px/1.65 system-ui;max-width:1300px;margin:30px auto;padding:0 18px;color:#21303c;background:#f5f7f9}h1{font-size:26px}.note{padding:15px;background:#fff1d3;border-radius:10px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}figure{margin:0;background:white;border-radius:10px;overflow:hidden}img{width:100%;display:block}figcaption{padding:10px}video{width:100%;max-width:960px}section{margin:24px 0}@media(max-width:800px){.grid{grid-template-columns:1fr}}</style><h1>공통 파지 시뮬레이션 검토</h1><p>파랑은 검지, 초록은 중지, 노랑은 약지, 빨강은 엄지다. 주황색 반투명 원통은 윗단을 90도 돌릴 때 필요한 공간을 표시한 것이다.</p><p class="note">실제 로봇 영상이 아닌 저장된 시뮬레이션 상태다. 이 모델의 기존 회전 구간은 약61도다. 저장된 실물 사진 추정은 초기 파지를 포함해 약80도, 파지 후 회전 구간은 약60도이며 정확한90도 완료는 미확인이다. 새 파지의 실물 재현과 실제 다른 손의 회전 하중은 아직 확인하지 않았다. 주황색 표시는 충돌 형상을 바꾸지 않는다.</p>'''+vid+'<div class="grid">'+cards+'</div><p><a href="../offline_review.html">현재 결과와 남은 작업</a> · <a href="manifest.json">그림과 영상의 출처</a></p></html>'
    (out/'index.html').write_text(text,encoding='utf-8');print(json.dumps({k:v for k,v in manifest.items() if k not in ('images','videos')}|{'images':len(images),'videos':len(videos)}),flush=True)
if __name__=='__main__':main()
