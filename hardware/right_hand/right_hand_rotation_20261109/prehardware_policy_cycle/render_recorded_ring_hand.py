"""Static forward kinematics of recorded angles; no cube pose is invented."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,ROOT,assets,restore,Guard,save_json,sha
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS
from render_canonical_grip_evidence_20261105 import png,display_physical_cube


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'user_ring_photo_reference_20261110'
    assert (folder/'review.json').exists()
    source=ROOT/'ring_reference_readonly_20261109_124719_945552/result.json';record=json.loads(source.read_text());rows={r['id']:r for r in record['snapshots'][0]['motors']}
    ticks=np.array([rows[i]['position_ticks'] for i in range(1,17)]);model,initial,_,_,provenance=assets('active_ccd50');data=restore(model,initial)
    data.qpos[:16]=ticks_to_sim(ticks);data.ctrl[:]=data.qpos[:16];mujoco.mj_forward(model,data);guard=Guard(model)
    # Display changes only: the cube placement was not measured with this read.
    display=display_physical_cube(model)
    cube_body=model.body('cube').id
    for gid in range(model.ngeom):
        if int(model.geom_bodyid[gid])==cube_body:model.geom_group[gid]=5
    colors={'if_':[.22,.45,.83,1.],'mf_':[.25,.65,.4,1.],'rf_':[.88,.69,.14,1.],'th_':[.83,.3,.3,1.]}
    for gid in range(model.ngeom):
        body=model.body(int(model.geom_bodyid[gid])).name or ''
        for prefix,color in colors.items():
            if body.startswith(prefix):model.geom_matid[gid]=-1;model.geom_rgba[gid]=color
    model.vis.global_.offwidth=1000;model.vis.global_.offheight=800
    option=mujoco.MjvOption();option.geomgroup[:]=[1,1,1,0,0,0]
    camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE;camera.lookat[:]=[.13,-.04,-.025];camera.distance=.32
    renderer=mujoco.Renderer(model,height=600,width=800);images=[]
    try:
        for label,azimuth,elevation in (('front',130,-30),('side',35,-30)):
            camera.azimuth=azimuth;camera.elevation=elevation;renderer.update_scene(data,camera=camera,scene_option=option)
            filename='recorded_hand_'+label+'.png';png(folder/filename,renderer.render());images.append(dict(file=filename,view=label))
    finally:renderer.close()
    report=dict(status='static_recorded_joint_forward_kinematics_only',source=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),
        images=images,model_provenance=provenance,display=display,sim_motor_ids=list(SIM_TO_MOTOR_IDS),joint_degrees=np.degrees(data.qpos[:16]).tolist(),
        exact_recorded_ring_values_not_clipped=True,model_joint_range_excess_deg=np.degrees(np.maximum(guard.qlo-data.qpos[:16],data.qpos[:16]-guard.qhi).clip(0)).tolist(),
        cube_hidden_for_display_only_due_to_unregistered_pose=True,other_fingers_from_old_read_not_confirmed_current_photo=True,
        no_dynamic_simulation_or_motor_goal=True,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'render_manifest.json',report)
    (folder/'index.html').write_text('''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>약지 기록과 실물 파지 사진</title><style>body{max-width:1100px;margin:30px auto;padding:0 18px;font:16px/1.7 system-ui;color:#173d36}img{max-width:100%;display:block}.grid{display:grid;grid-template-columns:1fr 1fr;gap:15px}.note{padding:15px;background:#fff1d2}@media(max-width:700px){.grid{grid-template-columns:1fr}}</style><body><h1>실제 파지 사진과 저장 관절값</h1><p class="note">사진과 관절값의 동시 기록은 확인되지 않았다. 모델 그림은 옛16관절 읽기 값의 정적 형태이며 중지·엄지가 사진과 같다고 가정하지 않는다. 측정된 큐브 위치가 없어 모델 그림에서는 큐브를 표시하지 않았다.</p><div class="grid"><figure><img src="user_photo.jpg"><figcaption>사용자가 제공한 실제 파지 사진</figcaption></figure><div><img src="recorded_hand_front.png"><img src="recorded_hand_side.png"><p>파랑 검지 · 초록 중지 · 노랑 약지 · 빨강 엄지. 카메라는 사진에 정합한 시점이 아니다.</p></div></div><p>약지 ID13/15/14/16=3018/2961/2829/3053틱을 자르지 않고 정적으로 표시했다. 모델 범위를 넘은 ID14를 새 목표로 보내거나 제한을 변경하지 않았다. 이 그림은 동적 파지·실물 안전성·윗단 공간의 인증이 아니다.</p><p><a href="REVIEW.md">사진 검토</a> · <a href="review.json">사진과 각도 출처</a> · <a href="render_manifest.json">모델 그림 출처</a></p></body></html>''',encoding='utf-8')
    print(json.dumps(dict(images=len(images),folder=folder.relative_to(ROOT).as_posix(),range_excess_deg=report['model_joint_range_excess_deg'])),flush=True)


if __name__=='__main__':main()
