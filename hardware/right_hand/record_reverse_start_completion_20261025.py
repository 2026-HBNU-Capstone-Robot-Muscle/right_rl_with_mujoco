"""Persist reviewed actual completion and the next photo-only physical task."""
import datetime,hashlib,json,math,subprocess,sys
from pathlib import Path
import numpy as np
from reverse_place_capture_plan_20261025 import ROOT,ReversePlacePlan,SOURCE,SOURCE_SHA,PHOTO_HOLD,INITIAL_SUPPORT_SCOPE
from reverse_place_capture_gate_20261025 import validate_proof

ENTRY=ROOT/'right_hand_rotation_20261024/reverse_empty_entry'
FIXTURE=ENTRY/'placement_fixture_v8'
FRESH='reverse_start_placement_20261025_074317_334798/fresh_off_request.json'

def main():
    p=ReversePlacePlan(FIXTURE/'request.json');proof=validate_proof(p,FIXTURE/'screen.json')
    regressions=json.loads((ENTRY/'placement_recovery_regressions_v8/summary.json').read_bytes())
    assert len(regressions)==3 and all(c['accepted'] for c in regressions)
    tests=(FIXTURE/'test_completion.log').read_text(encoding='utf-8')
    assert 'Ran 15 tests' in tests and tests.rstrip().endswith('OK')
    dry=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'leap_reverse_place_capture_20261025.py')],cwd=ROOT,capture_output=True,text=True,encoding='utf-8',timeout=5)
    assert dry.returncode==0 and 'CHECK ONLY: no camera, motor port, or simulator process opened.' in dry.stdout
    blob=(ROOT/SOURCE).read_bytes();assert hashlib.sha256(blob).hexdigest()==SOURCE_SHA
    r=json.loads(blob);assert r['status']=='reverse_start_empty_complete' and r['all_torque_off_verified'] and r['settings_restored']
    current=json.loads((ROOT/FRESH).read_bytes());read_log=json.loads((ROOT/Path(FRESH).with_name('result.json')).read_bytes())
    assert read_log['off_start_endpoint_stable'] and read_log['write_events']==[] and read_log['all_torque_off_verified']
    drift=int(np.ptp([s['position'] for s in read_log['off_start_observations'][-3:]],axis=0).max())
    assert drift<=2
    original=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json').read_bytes())
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt'
    assert hashlib.sha256(poster.read_bytes()).hexdigest()==original['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    source=Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py')
    assert hashlib.sha256(source.read_bytes()).hexdigest()=='008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'
    samples=r['samples'];error=np.asarray(r['final_on_state']['position'])-r['final_command']
    actual=dict(source=SOURCE,source_sha256=SOURCE_SHA,status=r['status'],all_torque_off_verified=True,settings_restored=True,
        max_tracking_rad=max(max(s['tracking_rad']) for s in samples),peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
        min_voltage_v=min(min(s['voltage_v']) for s in samples),max_temperature_c=max(max(s['temperature_c']) for s in samples),
        final_goal_error_ticks=error.tolist(),maximum_final_goal_error_degrees=float(max(abs(error))*360/4096),
        saved_camera_images=len(list((ROOT/Path(SOURCE).parent/'camera').glob('frame_*.jpg'))),
        immediate_post_off_reference_valid=r['final_off_reference_valid'],stable_later_off_source=FRESH,
        physical_cube_rotation_tested=False,human_hand_visible_not_scored_as_free_space_calibration=True)
    (ENTRY/'actual_043035_review/completion_review.json').write_text(json.dumps(actual,ensure_ascii=False,indent=2),encoding='utf-8')
    rows=p.plan(p.source_position());cp=next(i for i,(phase,q) in enumerate(rows) if phase==PHOTO_HOLD)
    ready=dict(timestamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat(),
        placement_photo_command_ready=True,user_command='leap_reverse_place_capture.cmd',rotation_included=False,
        native_static_cases=len(proof['static_screens']),native_dynamic_cases=len(proof['dynamic_sensitivity']),device_free_tests_passed=15,
        regression_cases=regressions,actual_start_families_tested=2,synthetic_root_sag_cases_tested=2,
        nominal_empty_goals_before_placement=cp,total_constant_inclusive_goals=len(rows),
        nominal_empty_restore_seconds=cp*.05,initial_support_scope=INITIAL_SUPPORT_SCOPE,
        actual_current_PID_and_stops_unchanged=True,assistant_motor_goal_writes=0,assistant_torque_changes=0,
        physical_ninety_degree_rotation_verified=False,development_request_sha256=p.SOURCE_SHA,
        plan_sha256=p.plan_digest(p.source_position()),screen_sha256=hashlib.sha256((FIXTURE/'screen.json').read_bytes()).hexdigest(),
        default_check_exit_code=dry.returncode,default_check_device_access=False,poster_unchanged=True,original_reference_unchanged=True)
    (FIXTURE/'readiness_review.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
    status=json.loads((ROOT/'right_hand_rotation_20261024/current_handoff_20261025.json').read_bytes())
    status.update(timestamp=ready['timestamp'],source_readonly=FRESH,source_readonly_maximum_drift_ticks=drift,
        empty_entry_physically_completed=True,empty_entry_result=SOURCE,empty_entry_metrics=actual,
        next_user_command=ready['user_command'],next_user_command_empty_only=False,next_user_command_photo_only=True,
        next_user_command_ready=True,new_device_free_tests_passed=15,placement_fixture='reverse_empty_entry/placement_fixture_v8',
        placement_static_cases=201,placement_native_cases=9,rotation_in_next_command=False,
        latest_placement_failure='No motion: first ID1 PING timeout, then old route exceeded support-separation model deadline',
        placement_recovery_revision='v8: begin all three roots during thumb-base separation at the existing 2-tick speed',
        placement_recovery_regressions=regressions,
        reverse_loaded_execution_ready=False,physical_ninety_degree_rotation_verified=False,
        next_action='User runs photo-only placement: EMPTY SPACE, PLACE on index/middle flat links with GREEN up and ORANGE toward WRIST, SPACE photo, REMOVE cube then SPACE OFF. Agent reviews files directly; no turn yet.')
    for path in (ROOT/'right_hand_rotation_20261024/current_handoff_20261025.json',ROOT/'right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json'):
        path.write_text(json.dumps(status,ensure_ascii=False,indent=2),encoding='utf-8')
    block=f'''## CURRENT 2026-10-25: 새 시작 자세 실물 완료 / 큐브 배치 사진 확인 대기

- `{Path(SOURCE).parent}`의 빈손 새 시작 자세 이동이 1303개 목표와 최종 정지까지 완료됐다. OFF 확인과 설정 복원이 정상이며 최저 전압 {actual['min_voltage_v']:.1f}V, 최대 전류 {actual['peak_current_ma']:.0f}mA, 최대 추종 {actual['max_tracking_rad']:.3f}rad, 최종 목표 오차 최대 {actual['maximum_final_goal_error_degrees']:.2f}도다. 저장 카메라 이미지는 {actual['saved_camera_images']}장이다. 실제 큐브 회전 시험은 아니다. 사람 손이 본체 근처에 보여 자유 공간 모터 보정 자료로 사용하지 않는다.
- 종료 직후 OFF 읽기는 안정 조건을 충족하지 않아 출발 기준으로 사용하지 않았다. 이후 직접 읽은 `{FRESH}`는 3회 동일한 OFF 값과 오류 없음이다. 읽기만 했으며 새 목표와 토크 쓰기는 없다. 다음 실행도 새 OFF 값을 읽어 다시 검사한다.
- 다음 사용자 작업은 `leap_reverse_place_capture.cmd`다. 첫 터미널 SPACE는 빈손 복귀, PLACE 안내 뒤 검지와 중지의 평평한 링크 윗면에 초록 위와 주황 손목 쪽으로 큐브를 놓고 두 번째 SPACE로 촬영한다. REMOVE에서 먼저 큐브를 빼고 세 번째 SPACE로 OFF한다. 이후 목표는 모두 고정이며 회전이나 SAC 실행이 없다. 카메라는 자동으로 연다. 그림은 `reverse_empty_entry/actual_043035_review/placement_guide.png`다.
- 현재 입력의 짧은 복귀는 촬영 전 {cp}목표와 명목 {cp*.05:.1f}초다. 엄지 뿌리 복귀 동안 세 손가락 뿌리도 기존 2틱 속도 제한 안에서 먼저 올려, OFF 후 추가 처짐 때문에 받침 분리가 늦어지는 것을 해결했다. 전류와 PID, 실제 전압 및 추종 중단 기준, 형상과 접촉 힘 및 분리 기한 판정식은 유지했다. 현재 자세의 201개 정적 조건과 9개 동역학 조건, 이전 실제 자세와 추가 ±16틱 뿌리 처짐 가정의 각각 같은 전체 검사, 장치 없는 검사 15개를 통과했다. 합성 처짐은 실제 측정값으로 표시하지 않는다.
- 초기 받침 접촉 검사는 측정 시작점 주변 ±4틱의 기존 67개 형상과, 목표가 움직이지 않는 0.5초 전원 정착 구간을 구분한다. 그 정착 때 새로 닿는 받침 쌍은 기존 모델 수치 허용 0.15mm 이내여야 하고 이후 범위와 깊이를 넘거나 분리 기한이 지나면 거부한다. 이번 추가 쌍은 보수적인 책 앞끝 -5mm 및 낮은 모델 이득 조건의 약지 뿌리 케이스와 책이며 최대 약 0.00295mm다. 실제 침투량 측정이 아니다. 초기 엄지 케이스 분리는 별도 1mm 모델 상한, 새롭거나 깊어지는 자기 접촉 금지, 분리 후 자기 겹침 0을 유지한다. 실패한 개발 검사는 별도 폴더에 보존했다.
- 역순 전체 회전은 여전히 조건부 시뮬레이션이다. 기준 조건의 약 95.6도와 목표 오차 7.5도 및 4초 유지를 실물 성공으로 표시하지 않는다. 적재 회전의 모델 패드 접촉과 새 실제 큐브 배치를 해결하기 전 회전 명령을 내보내지 않는다. 전시 문구와 원본은 변경하지 않았다.

<!-- END REVERSE PLACEMENT CURRENT 20261025 -->

'''
    end='<!-- END REVERSE PLACEMENT CURRENT 20261025 -->'
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith('## CURRENT 2026-10-25: 새 시작 자세 실물 완료') and end in old:old=old.split(end,1)[1].lstrip()
        path.write_text(block+old.replace('## CURRENT','## PREVIOUS'),encoding='utf-8')
    readme=f'''# 새 시작 자세 도착 후 배치 촬영

새 시작 자세의 실물 이동은 완료됐다. `actual_043035_review/completion_review.json`을 확인한다. 다음 사용자 명령은 `leap_reverse_place_capture.cmd`이며 실제 큐브 회전이나 정책 실행이 없다.

1. 큐브를 손과 화면 밖에 둔다. 카메라는 자동으로 열린다. 터미널 첫 SPACE로 새 OFF 값과 전체 모델 검사 후 짧게 복귀한다.
2. PLACE가 나오면 `actual_043035_review/placement_guide.png`처럼 검지와 중지의 평평한 링크 윗면에 큐브를 걸쳐 둔다. 초록 위, 주황 손목 쪽이다. 손을 떼도 받쳐져야 한다. 손을 뺀 뒤 두 번째 SPACE로 사진만 저장한다. 받쳐지지 않으면 Q로 중단한다.
3. REMOVE에서 실제로 큐브를 먼저 빼고 세 번째 SPACE로 OFF한다. 종료 뒤 했어만 알려주면 저장 파일을 직접 확인한다.

`placement_fixture_v8`은 현재 실제 안정 OFF 기록에 근거한 개발 입력의 전체 201개 정적 조건 및 9개 동역학 조건과 독립 Windows 검증을 통과했다. 장치 없는 검사 15개도 통과했다. 실행 때 개발 입력을 재사용하지 않고 새 안정 OFF 읽기로 전체 검사를 다시 한다. 현재 입력의 촬영 전 이동은 명목 {cp*.05:.2f}초이며 검사 시간과 최종 정지 및 사용자 대기는 별도다.

목표가 움직이지 않는 초기 전원 정착 동안의 받침 접촉과 이후 명령 경로를 구분한다. `initial_support_scope`와 각 동역학 행의 `passive_enable_hold_support`에 최대 깊이, 고정 목표와 구간 길이가 기록돼 있다. 정착 때 새 받침 접촉은 기존 0.15mm 모델 허용 안에서만 등록하며, 이후 새 범위나 깊이 초과 및 기한 뒤 접촉은 계속 거부한다. 형상과 접촉 힘을 제거하거나 실제 보호 한계를 바꾼 것이 아니다. 초기 자기 접촉은 엄지 뿌리를 이전에 도착한 목표 쪽으로 먼저 분리하며 분리 후 자기 겹침을 허용하지 않는다. 모델 접촉을 실제 침투량으로 해석하지 않는다.

개발 실패 기록 `placement_fixture_v1`부터 `v4`는 이동 순서와 초기 접촉 처리 검사에서 통과하지 못했다. `v5`는 분리 목표 개수의 독립 검증에서 거부됐다. `v6`는 이전 실제 입력을 통과했으나 사용자의 07:43 입력에서 받침 분리 기한 초과로 실제 이동 전 거부됐다. `v7`은 약지 뿌리만 일찍 올려 실제 두 자세를 통과했으나 추가 처짐 가정에서 실패했다. `v8`은 세 뿌리를 기존 속도 제한 안에서 더 일찍 올려 두 실제 입력과 합성 ±16틱 처짐 검사까지 통과했다. 현재 명령은 `v8`만 참조한다. 기존 분리 기한 식과 실제 중단 한계는 바꾸지 않았다. 이전 빈손 진입 `fixture_v2`와 원본 및 실패 기록은 보존했다. 전체 90도 적재 회전은 아직 실행 준비가 아니다.
'''
    (ENTRY/'README.md').write_text(readme,encoding='utf-8')
    side=ROOT/'right_hand_rotation_20261024/middle_ring_side_review/README.md'
    old=side.read_text(encoding='utf-8')
    side.write_text(block+old.replace('## CURRENT','## PREVIOUS'),encoding='utf-8')
    print(json.dumps(dict(actual_empty_complete=True,photo_command_ready=True,tests_passed=15,goals_before_place=cp,nominal_restore_seconds=cp*.05,poster_unchanged=True,actual_assistant_motor_writes=0),ensure_ascii=False))

if __name__=='__main__':main()
