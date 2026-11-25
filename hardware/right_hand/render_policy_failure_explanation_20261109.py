"""Scientific views of saved diagnostic states; never physical photographs."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
from pathlib import Path
import json
import hashlib
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from sac_forward_geometry import target_quaternion
from render_canonical_grip_evidence_20261105 import display_physical_cube, cube_display, extra, png
from hand_motor_mapping import sim_to_ticks

ROOT = Path(__file__).resolve().parent
BASE = ROOT / 'right_hand_rotation_20261109/policy_audit'
OUT = BASE / 'explanation'
SOURCE = ROOT / 'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_binary_path('source.mjb', assets={'source.mjb': SOURCE.read_bytes()})
    display_physical_cube(model)
    model.vis.global_.offwidth = 1000
    model.vis.global_.offheight = 800
    for g in range(model.ngeom):
        body_name = model.body(int(model.geom_bodyid[g])).name
        if model.geom_group[g] < 3 and body_name != 'cube':
            model.geom_group[g] = 0
        for prefix, color in {'if_': [.22,.45,.83,1], 'mf_': [.25,.65,.4,1],
                              'rf_': [.88,.69,.14,1], 'th_': [.83,.3,.3,1]}.items():
            if body_name.startswith(prefix):
                model.geom_matid[g] = -1
                model.geom_rgba[g] = color
    sources = {
        'training_start': (BASE / 'training_start_transfer/sac_cube55_resets_0.npz', 'qpos', None),
        'first_contact': (BASE / 'reused_reference_controller/interpolated_two_ticks_75ms_saved_sac.npz', 'qpos', -1),
        'after_preclear_stop': (BASE / 'reused_reference_preclear_controller/interpolated_two_ticks_75ms_saved_sac.npz', 'qpos', -1),
        'recent_probe_pose': (ROOT / 'right_hand_rotation_20261109/middle_probe_reconstruction_linux/checkpoint_927.npz', 'qpos', None),
    }
    renderer = mujoco.Renderer(model, height=720, width=960)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = .33
    camera.lookat[:] = [.13, -.008, -.005]
    camera.azimuth = 135
    camera.elevation = -28
    option = mujoco.MjvOption()
    option.geomgroup[:] = [1,1,1,0,0,0]
    cube = model.geom('cube').id
    records = []

    def stickers(data):
        center = data.geom_xpos[cube]
        B = data.geom_xmat[cube].reshape(3,3)
        cube_display(renderer.scene, center, B, False)
        for axis, sign, color in ((0,1,[.85,.16,.13,1]),(0,-1,[1.,.45,.07,1]),
                                  (1,1,[.12,.3,.85,1]),(1,-1,[.13,.7,.3,1]),
                                  (2,1,[.95,.95,.95,1]),(2,-1,[.95,.82,.08,1])):
            other = [i for i in range(3) if i != axis]
            for u in (-1,0,1):
                for v in (-1,0,1):
                    size = np.full(3, .0083)
                    size[axis] = .00015
                    p = np.zeros(3)
                    p[axis] = sign * .0277
                    p[other] = np.array([u,v]) * .055/3
                    extra(renderer.scene, mujoco.mjtGeom.mjGEOM_BOX, size, center+B@p, B, color)

    for name, (path, field, index) in sources.items():
        with np.load(path, allow_pickle=False) as z:
            q = z[field] if index is None else z[field][index]
        data = mujoco.MjData(model)
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera, scene_option=option)
        stickers(data)
        target = OUT / (name + '.png')
        png(target, renderer.render())
        records.append({'image': target.name, 'source': str(path.relative_to(ROOT)),
                        'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                        'qpos_source_index': index, 'motor_ticks_from_qpos': sim_to_ticks(q[:16]).tolist()})
        if name == 'training_start':
            desired = q.copy()
            desired[-4:] = target_quaternion(q[-4:])
            data.qpos[:] = desired
            mujoco.mj_forward(model, data)
            camera.lookat[:] = data.geom_xpos[cube]
            camera.distance = .14
            only_cube = mujoco.MjvOption()
            only_cube.geomgroup[:] = [0,0,1,0,0,0]
            renderer.update_scene(data, camera=camera, scene_option=only_cube)
            stickers(data)
            png(OUT / 'cube_target_only.png', renderer.render())
            camera.distance = .33
            camera.lookat[:] = [.13, -.008, -.005]
        if name == 'first_contact':
            contacts = [c for c in data.contact if {model.geom(int(c.geom1)).name, model.geom(int(c.geom2)).name}
                        in ({'palm_collision_4','th_tip'}, {'palm_collision_8','th_tip'})]
            if contacts:
                point = contacts[0].pos.copy()
                camera.lookat[:] = point
                camera.distance = .18
                camera.azimuth = 135
                camera.elevation = -50
                close_option = mujoco.MjvOption()
                close_option.geomgroup[:] = [1,1,0,0,0,0]
                renderer.update_scene(data, camera=camera, scene_option=close_option)
                extra(renderer.scene, mujoco.mjtGeom.mjGEOM_SPHERE, [.0025,0,0], point,
                      np.eye(3), [1.,.03,.03,1.])
                png(OUT / 'thumb_palm_close.png', renderer.render())
                camera.lookat[:] = [.13, -.008, -.005]
                camera.distance = .33
                camera.azimuth = 135
                camera.elevation = -28
    renderer.close()
    manifest = {'scientific_model_renders_not_physical_images': True, 'source_model_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                'states': records, 'goal_image_changes_cube_orientation_for_illustration_only': True,
                'closeup_hides_cube_display_only': True, 'red_dot_marks_model_contact': True,
                'devices_opened': 0}
    (OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>시험한 정책과 시작·정지 자세</title><style>body{font:17px/1.75 system-ui;max-width:1150px;margin:24px auto;padding:0 18px;color:#182931}h1{font-size:28px}h2{font-size:23px}.note{background:#fff0ce;padding:14px;border-radius:8px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}figure{margin:0}img{width:100%;display:block}figcaption{padding:9px}table{border-collapse:collapse;width:100%}td,th{text-align:left;padding:9px;border-bottom:1px solid #ccc}code{overflow-wrap:anywhere}@media(max-width:700px){.grid{grid-template-columns:1fr}}</style>
<h1>어떤 정책으로 어디까지 움직였나</h1><p class="note">모두 저장 상태를 그린 시뮬레이션 그림이다. 실제 로봇이나 웹캠은 이번 시험에서 사용하지 않았다. 빨강=엄지, 파랑=검지, 초록=중지, 노랑=약지.</p>
<p class="note"><strong>2026-11-10 후속 결과:</strong> 이전 첫 회전·유지 통과는 접촉 계산10회 모델의 기록이다. 누락 접촉을 확인해50회 별도 모델로 첫 회전부터 재검증했고 기존 정책은4118목표에서 중단됐다. 같은 명령의50/100회 최종 상태 차이는0이다. 기존 SAC의 보정 학습과 새 초기 배치 검증은 <a href="../../prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/index.html">최신 수렴·정책 검증</a>을 따른다. 전체 회전→유지→다음 회전 및 실물 준비 완료가 아니다. <a href="../../prehardware_policy_cycle/review/index.html">그림과 전체 검증 결과</a>. 아래는 이전 진단의 설명이다.</p>
<h2>최신 정정: 정책 입력의 손바닥 기준점 수정</h2><p class="note">아래 그림을 만든 진단은 학습의 손바닥 센서 기준점 대신 손 본체 원점을 넣고 있었다. 두 점은 약96.48mm 떨어져 있다. 입력을 수정했고 원래 학습 관측 함수와 최대 약1.01e-7 차이로 일치함을 확인했다. 아래의 기존 결과를 정책 성능의 확정 판정에 쓰지 않는다.</p><p>수정 후 같은 저속 비교는 모델8.776초/정책8회 후 추종 오차0.18045rad로 기존 기준에 걸렸다. 그때 시작 대비 약−13.32도였지만 기준 궤적 초기 구간의 중단 상태이며 최종 회전 동작이 완료된 결과가 아니다. 현재 목표는 입력·모델·구동 제약을 맞추고 회전→파지 유지→다음 회전을 연결하는 것이다. 새 학습이나 실물 실행을 완료한 상태는 아니다. <a href="../environment_transfer/observation_provider_verification.json">관측 비교</a> · <a href="../reused_reference_palm_pad_sensitivity_0.2mm_training_sensor/review.json">수정 후 모델 결과</a></p>
<h2>접촉 시험의 정책</h2><p><code>sac_forward_support_reference_v1_20k/numpy_export/actor.npz</code>의 110입력/16출력 SAC 보정 모델이다. 예전 기준 궤적에 최대0.05rad 보정을 더하도록2만 스텝 학습한 모델이며, 원본 저장소의108입력 <code>policy_np.npz</code>와 다르다.</p>
<h2>의도한 이동</h2><div class="grid"><figure><img src="training_start.png"><figcaption>출발: 예전 정책 학습용 시작 자세. 큐브가 손바닥 중앙 부근에 있고 약지가 굽혀져 있다. 현재 실물 약지 읽기값을 복사한 자세가 아니다.</figcaption></figure><figure><img src="cube_target_only.png"><figcaption>목표: 큐브 전체를 앞으로 약90도 넘겨 빨강(+X) 면을 위로 보낸다. 초록(-Y)에서 새 면으로 바꾸는 단일 방향 시험이다. 그림은 목표 큐브 방향만 표시하며 손가락 경로나 도달 상태가 아니다.</figcaption></figure></div>
<h2>처음 걸린 위치: 엄지 끝 ↔ 손바닥</h2><div class="grid"><figure><img src="first_contact.png"><figcaption>정책 목표를2틱씩75ms에 걸쳐 보낸 모델: 약0.485초에서 접촉 판정. 큐브는 아직 거의 회전하지 않았다.</figcaption></figure><figure><img src="thumb_palm_close.png"><figcaption>같은 자세 확대. 접촉을 보기 위해 큐브 표시만 숨겼다. 빨간 점은 엄지 끝 패드와 손바닥의 모델 접촉 위치다.</figcaption></figure></div>
<p>접촉쌍은 <code>th_tip ↔ palm_collision_4 / palm_collision_8</code>이다. 보고된 겹침은 약0.002mm로 매우 작다. 실제 기구에서 위험한 충돌이 있었다는 증거가 아니며, 현재 모델의 자기 접촉 검사를 통과하지 못했다는 결과다. “중지와 약지끼리 부딪혔다”는 설명은 이 시험에 맞지 않는다.</p>
<h2>엄지를 먼저 띄운 비교</h2><div class="grid"><figure><img src="after_preclear_stop.png"><figcaption>엄지 초기 분리 후 정책8회 갱신, 저속 목표193개, 모델 시간 약14.475초까지 진행했다. 실행한 상태에서 자기 접촉 겹침은0이었다.</figcaption></figure><figure><img src="recent_probe_pose.png"><figcaption>비교용: 저번 중지 진단의 모델 파지 자세. 이번 정책 시험 출발 자세와 큐브 위치가 다르다. 실제 웹캠 사진이 아니다.</figcaption></figure></div>
<p>이 비교에서는 큐브가 목표 반대 방향으로 약13도 기울었고, 다음 정책 목표를 제한된 보정 범위 안에서 충돌 없이 만드는 계산에 실패해 다음 목표를 보내기 전에 멈췄다. 그때 실제 로봇이 충돌한 것도, 이 모델 자세를 실제로 실행한 것도 아니다.</p>
<h2>미세 접촉을 구분하면 더 움직이나</h2><p><strong>약0.002mm 모델 겹침만으로 동작 불가라고 판단하는 것은 과도했다.</strong> 손끝 패드–손바닥 접촉만 최대0.2mm로 구분한 후속 모델 비교에서는 충돌 힘 계산을 유지한 채 약9.038초, 정책8회까지 진행했다. 다만 큐브는 목표 반대 약13.33도, 중심 약12.28mm 이동했다. 마지막 엄지–약지 끝의 약0.00057mm 모델 접촉 판정 역시 실제 위험 충돌을 증명하지 않는다. 초기 접촉 정지를 넘긴 사실과 원하는 방향 회전 미달을 구분한다. 실물 제어 허용치를 바꾼 시험은 아니다. <a href="../reused_reference_palm_pad_sensitivity_0.2mm/review.json">후속 비교 수치</a></p>
<p class="note">판단: 옛 학습 정책을 현재 모델에 바로 연결하는 방법과 단순 감속은 검증을 통과하지 못했다. 해결 불가능하다는 결론은 아니다. 현재 모델과 제어 방식에 맞는 정책 보정이 아직 필요한 상태다.</p>
<p><a href="../STATUS.md">전체 근거와 범위</a> · <a href="manifest.json">그림 출처</a></p></html>'''
    # The free camera looks at the model contact point, so its screen projection
    # is the image center. Mark that projection even when the visual mesh hides it.
    overlay = '''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 720"><image href="thumb_palm_close.png" width="960" height="720"/><circle cx="480" cy="360" r="27" fill="none" stroke="#fff" stroke-width="10"/><circle cx="480" cy="360" r="27" fill="none" stroke="#e11419" stroke-width="5"/><path d="M500 339 L615 185 L875 185" fill="none" stroke="white" stroke-width="10"/><path d="M500 339 L615 185 L875 185" fill="none" stroke="#e11419" stroke-width="4"/><rect x="610" y="138" width="285" height="45" rx="8" fill="white"/><text x="624" y="169" font-family="sans-serif" font-size="25" fill="#ae1015">엄지 끝 ↔ 손바닥</text></svg>'''
    (OUT / 'thumb_palm_marked.svg').write_text(overlay, encoding='utf-8')
    page = page.replace('src="thumb_palm_close.png"', 'src="thumb_palm_marked.svg"').replace('빨간 점은 엄지 끝 패드와 손바닥의 모델 접촉 위치다.', '빨간 원은 엄지 끝 패드와 손바닥의 모델 접촉 위치를 화면에 투영한 표시다.').replace('red_dot_marks_model_contact', 'red_circle_marks_model_contact_projection')
    (OUT / 'index.html').write_text(page, encoding='utf-8')
    print(json.dumps({'page': str(OUT / 'index.html'), 'images': 6, 'devices_opened': 0}, ensure_ascii=False))


if __name__ == '__main__':
    main()
