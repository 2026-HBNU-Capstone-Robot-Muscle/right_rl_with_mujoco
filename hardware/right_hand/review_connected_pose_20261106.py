"""Summarize camera and OFF reads; render FK without issuing hardware writes."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261106/taught_upper_clearance'
PHOTO=O/'current_camera_220650/latest.jpg';READ=R/'resume_off_readonly_20261106_220704_740850/result.json';BEFORE=R/'resume_off_readonly_20261106_215211_977512/result.json'
def main():
    import mujoco
    from hand_motor_mapping import ticks_to_sim
    from research_rotation_body_hold_20261104 import inspect_for
    from render_canonical_grip_evidence_20261105 import display_physical_cube,png
    before=json.loads(BEFORE.read_bytes());after=json.loads(READ.read_bytes());assert after['status']=='readonly_data_saved' and after['servo_writes']==after['torque_changes']==0 and after['all_torque_off_read'];s=after['snapshots'][-1];delta=np.array(s['position'])-before['snapshots'][-1]['position'];assert max(abs(delta))<=4
    m=mujoco.MjModel.from_binary_path(str(O/'model.mjb'));display_physical_cube(m);m.vis.global_.offwidth=1000;m.vis.global_.offheight=800;d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(s['position']);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=[0,0,2,1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d);su,sp=inspect_for(m)[2](d)
    for g in range(m.ngeom):
        name=m.body(int(m.geom_bodyid[g])).name;color=next((v for k,v in {'if_':[.22,.45,.83,1],'mf_':[.25,.65,.4,1],'rf_':[.88,.69,.14,1],'th_':[.83,.3,.3,1]}.items() if name.startswith(k)),None)
        if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
    out=O/'connected_observation';out.mkdir(exist_ok=True);renderer=mujoco.Renderer(m,height=720,width=960);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.distance=.34;cam.lookat[:]=[.12,-.025,-.02];opt=mujoco.MjvOption();opt.geomgroup[:]=[1,1,1,0,0,0];images=[]
    for name,az,el in [('wrist',-45,-55),('top',90,-85),('side',35,-25),('opposite',135,-35)]:
        cam.azimuth=az;cam.elevation=el;renderer.update_scene(d,camera=cam,scene_option=opt);file=name+'.png';png(out/file,renderer.render());images.append(file)
    renderer.close();report=dict(status='camera_observation_and_nearby_OFF_reference_saved',camera_photo=PHOTO.relative_to(R).as_posix(),encoder_source=READ.relative_to(R).as_posix(),earlier_encoder_source=BEFORE.relative_to(R).as_posix(),maximum_change_from_reviewed_start_ticks=int(max(abs(delta))),per_motor_delta_ticks=delta.tolist(),after_camera_drift_ticks=after['maximum_drift_ticks'],minimum_voltage_V=min(s['voltage'])/10,maximum_temperature_C=max(s['temperature']),all_torque_off=True,servo_writes=0,torque_changes=0,encoder_photo_not_continuously_synchronized=True,model_static_self_pairs_mm=sp,model_static_support_pairs_mm=su,geometry_calibration_verified=False,current_OFF_pose_is_observation_not_new_motion_target=True,images=images)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>현재 빈손 사진과 관절 모델</title><style>body{font:17px/1.7 system-ui;max-width:1100px;margin:30px auto;padding:0 20px}img{width:100%}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}figure{margin:0}figcaption{padding:8px}.note{background:#fff0ce;padding:15px}</style><h1>현재 빈손 사진과 관절 모델</h1><p>파랑=검지, 초록=중지, 노랑=약지, 빨강=엄지. 모델의 카메라 방향은 실제 사진과 정확히 맞춘 보정 결과가 아니다.</p><p class="note">읽기만 수행했고 모터 목표나 토크를 바꾸지 않았다. OFF 상태의 현재 자세를 새 파지 목표로 채택한 것이 아니다.</p><h2>웹캠 원본</h2><img src="../current_camera_220650/latest.jpg"><h2>기록한 관절값으로 만든 모델</h2><div class="grid">'''+''.join('<figure><img src="'+f+'"><figcaption>'+f+'</figcaption></figure>' for f in images)+'''</div><p><a href="review.json">사진과 관절값 출처</a> / <a href="../progress.md">진행 상황</a></p></html>''';(out/'index.html').write_text(page,encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':main()
