import datetime,hashlib,json
from pathlib import Path
import leap_quarter_support_photo_20261025 as photo
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/rigid_contact_review_20261025'
def load(p):return json.loads(Path(p).read_bytes())
def save(p,r):Path(p).write_text(json.dumps(r,indent=2,ensure_ascii=False),encoding='utf-8')
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    photo.require_mode_review();material=load(OUT/'material_confirmation.json');mode=load(OUT/'photo_mode_review.json')
    coarse=load(OUT/'surface_friction_comparison.json');fine=[load(OUT/f'timestep_{step:.4f}/surface_friction_comparison.json') for step in (.002,.001)]
    assert material['confirmed_contact_material']=='rigid plastic'
    baseline=load(ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json');poster='exhibition_20261023/족자_붙여넣기.txt'
    assert digest(ROOT/poster)==baseline['production_hashes_recorded_without_reissuing_certificates'][poster]
    assert digest(ROOT/'right_hand_rotation_20261023/native_motor_limits.json')=='c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716'
    assert digest(Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'))=='008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'
    now=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat()
    report=dict(timestamp=now,material_confirmation=material,surface_comparison_cases=len(coarse['cases']),coarse_summary=coarse['summary'],
        smaller_timestep_cases=sum(len(r['cases']) for r in fine),smaller_timestep_summaries=[dict(timestep=r['diagnostic_timestep'],summary=r['summary']) for r in fine],
        traction_only_recommendation_supported=False,reason='Traction changes are not consistently beneficial across time steps and positions; contact penetration peaks exceed3mm in several cases. Actual contact surface and initial seating remain unregistered.',
        fingertip_only_friction_change_did_not_improve_coarse_cases=True,numeric_friction_not_inferred_from_rigid_material=True,
        maximum_cube_contact_penetration_mm=max(c['max_cube_overlap_mm'] for r in [coarse]+fine for c in r['cases']),
        physical_friction_identified=False,physical_ninety_degree_rotation_verified=False,hardware_rotation_candidate_ready=False,
        next_user_command='leap_quarter_support_photo.cmd',next_user_command_ready=True,next_user_command_photo_only=True,
        next_user_command_has_empty_preparation_motion=True,next_user_command_has_loaded_motion=False,photo_mode_review=mode,
        review_camera_guide=(OUT/'camera_support_side_guide.html').relative_to(ROOT).as_posix(),
        next_action='One side support photo in the reviewed quarter-turn starting pose; show Cube bottom edge and supporting links. EMPTY first SPACE, PLACE/photo second SPACE, REMOVE Cube before third SPACE/OFF. No loaded rotation.',
        assistant_motor_goal_writes=0,assistant_torque_changes=0,actual_device_acquisition_this_turn=False,
        original_reference_preserved=True,poster_unchanged=True,
        source_references=['https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters','https://v1.leaphand.com/leap_cad'])
    save(OUT/'review.json',report)
    ready_path=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json';ready=load(ready_path)
    ready.update(timestamp=now,material_confirmation=material,supervised_experimental_loaded_trial_ready=False,
        next_user_command=report['next_user_command'],next_user_command_photo_only=True,next_user_command_has_loaded_motion=False,
        quarter_support_photo_mode_review=mode,contact_review=(OUT/'review.json').relative_to(ROOT).as_posix());save(ready_path,ready)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json'):
        p=ROOT/name;r=load(p);r.update(timestamp=now,material_confirmation=material,next_user_command=report['next_user_command'],
            next_user_command_ready=True,next_user_command_empty_only=False,next_user_command_photo_only=True,rotation_in_next_command=False,
            supervised_experimental_loaded_trial_ready=False,physical_ninety_degree_rotation_verified=False,
            rigid_contact_review=report,new_device_free_photo_tests_passed=7,new_agent_motor_goal_writes=0,new_agent_torque_changes=0,
            loaded_trial_readiness=ready,next_action=report['next_action']);save(p,r)
    note='''## CURRENT 2026-10-25: 단단한 플라스틱 확인 / 접촉 계산의 불확실성 검토 / 측면 받침 촬영 준비

- 사용자는 큐브와 닿는 검지와 중지 끝부분이 단단한 플라스틱이라고 확인했다. 실제 마찰계수는 아직 측정하지 않았으며 재질 이름만으로 수치를 확정하지 않는다. ID4 하한 2도 감소는 현재 실험 설정에 반영돼 있다.
- 실제 적재 관절값을 처방한 접촉 비교 20개를 수행했다. 기본 시간 간격에서 손끝 네 곳의 마찰만 높이는 안은 기존 결과와 같았고, 검지와 중지의 평평한 링크 접촉을 바꾸는 안에서 개선이 보였다. 그러나 이 안은 접촉 도형 전체의 마찰을 바꾼 상한 가정이며 실제 패드 두께나 부착 면을 검증한 결과가 아니다.
- 계산 시간 간격을 2ms와 1ms로 줄인 12개 비교에서는 위치에 따른 낙하 여부가 달라졌고 마찰 변경이 항상 개선을 주지도 않았다. 일부 조건에서 큐브와 손 사이의 모델 겹침이 3mm를 넘었다. 이 결과로 고무 부착이 해결책이라고 단정하거나 높은 마찰 수치를 넣어 실물 회전 성공으로 표시하지 않는다. 17:16 실제 실행은 여전히 명령 완주와 회전 미달로 구분해 보존한다.
- 저장 영상에서는 큐브 밑모서리와 실제 받침이 가려져 있어, 초기 접촉 위치를 확인할 측면 사진 한 번이 필요하다. 새 **`leap_quarter_support_photo.cmd`**는 기존에 검토한 빈손 복귀 경로의 앞부분만 실행한다. PLACE 이후에는 관절 목표를 유지하면서 사진을 찍고, 큐브 제거 확인 후 OFF한다. 적재 회전 목표를 보내지 않는다. 별도 검사 7개에서 이 중단 경계, 카메라 실패와 사용자 중단 시 OFF 및 설정 복원을 확인했다. 현재 실제 자세에서의 검사도 실행 때 다시 수행한다. 이 개발 및 검토에서 실제 장치는 열지 않았다.
- 사용자 순서: 웹캠을 낮춰 큐브 밑모서리와 바로 아래 검지와 중지 받침이 함께 보이도록 잡는다. **첫 SPACE는 빈손 준비**다. PLACE에서 이전처럼 초록 위와 주황 손목 방향으로 큐브를 놓고, 화면에서 받침을 확인한 뒤 **둘째 SPACE로 사진만 저장**한다. **큐브를 먼저 뺀 뒤 셋째 SPACE로 OFF**한다. Q와 Esc 및 Ctrl+C는 중단이다. 끝나면 했어만 알리면 저장 사진과 관절값을 직접 확인한다.
- 카메라 방향 안내는 `right_hand_rotation_20261024/rigid_contact_review_20261025/camera_support_side_guide.html`에 있다. 마커 크기 재측정이나 같은 전체 회전의 반복은 요청하지 않았다. 원본 시퀀스, 과거 실물 기록과 전시 문구는 그대로다.

<!-- END RIGID CONTACT AND SUPPORT PHOTO 20261025 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8')
        if old.startswith('## CURRENT 2026-10-25: 단단한 플라스틱 확인'):old=old.split('<!-- END RIGID CONTACT AND SUPPORT PHOTO 20261025 -->',1)[1].lstrip()
        p.write_text(note+old.replace('## CURRENT','## PREVIOUS'),encoding='utf-8')
    (OUT/'README.md').write_text(note,encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('surface_comparison_cases','smaller_timestep_cases','traction_only_recommendation_supported','next_user_command','next_user_command_has_loaded_motion','assistant_motor_goal_writes')},ensure_ascii=False))
if __name__=='__main__':main()
