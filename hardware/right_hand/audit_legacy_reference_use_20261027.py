"""Publish source traceability for the user's previous rotation, no devices."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261027'


def main():
    archive=ROOT/'sequence_flip_poses.json'
    poses=json.loads(archive.read_bytes())
    source=Path(poses['original_source'])
    assert hashlib.sha256(source.read_bytes()).hexdigest()==poses['original_sha256']
    assert hashlib.sha256(archive.read_bytes()).hexdigest()=='cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b'
    sources=[]
    for name in ('leap_sequence_flip_20261011_051350_713486.json','leap_sequence_flip_20261011_051518_433595.json'):
        path=ROOT/name;record=json.loads(path.read_bytes())
        sources.append(dict(path=name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),status=record.get('status'),samples=len(record.get('samples',[])),phases=list(dict.fromkeys(v['phase'] for v in record.get('samples',[])))))
    seed_path=ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json'
    seed=json.loads(seed_path.read_bytes())
    trace=np.load(ROOT/'right_hand_rotation_20261026/fresh_empty_start_20261026_235258/recovery_plan_v2_ring_first/path.npz')
    opened=trace['open_goal'];original=np.array(poses['poses'][4]['raw_ticks'])
    assert np.array_equal(opened[:4],original[:4])
    assert np.array_equal(opened[8:12],original[8:12])
    robust_path=ROOT/'right_hand_rotation_20261027/reached_pair_contact_yaw/robust_free_cube_review.json'
    robust=json.loads(robust_path.read_bytes());cases=robust['cases'];assert len(cases)==9 and all(c['complete'] for c in cases)
    report=dict(original_source=poses['original_source'],original_source_sha256=poses['original_sha256'],archive=archive.relative_to(ROOT).as_posix(),archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),original_pose_count=len(poses['poses']),actual_historical_runs=sources,
        actual_source_for_current_cube_seed=seed['source'],seed_source_hash=seed['source_sha256'],cube_seed=seed_path.relative_to(ROOT).as_posix(),cube_seed_is_optical_hypothesis_not_registered_truth=True,
        inherited_open_thumb_ids_1_4=opened[:4].tolist(),inherited_open_middle_ids_9_12=opened[8:12].tolist(),all_old_motor_commands_reused_verbatim=False,
        historical_user_observation=dict(green_stayed_up=True,front_orange_changed_to_yellow=True,base_only_held_by_person=True,precise_90_not_independently_measured=True),
        current_model_yaw=dict(review=robust_path.relative_to(ROOT).as_posix(),cases=9,complete_cases=9,final_deg_range=[min(c['final_yaw_deg'] for c in cases),max(c['final_yaw_deg'] for c in cases)],nominal_deg=next(c['final_yaw_deg'] for c in cases if c['gain_scale']==1 and c['friction']==.3)),
        original_files_unchanged=True,hardware_execution_allowed=False,devices_opened=False,motor_commands_sent=0)
    (OUT/'legacy_reference_traceability.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    text='''# 예전 회전 파일을 현재 계획에 반영한 내용

사용자가 전에 돌렸다고 제공한 `C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py`의 7개 자세를 현재 확인된 모터 번호로 복원했다. 원본을 실행하거나 가져오지 않고 배열을 추출했고, 원본과 보관 배열의 SHA256을 다시 확인했다.

10월 11일 05:13과 05:15 실제 실행 두 건의 목표와 관절 읽기, 단계별 사진도 비교했다. 당시 사용자가 관찰한 “초록은 계속 위, 앞면은 주황에서 노랑으로 변경, 사람은 받침만 잡음”을 보존했다. 이 관찰을 새로운 정밀 90도 측정으로 바꾸어 표현하지 않는다.

현재 계산에도 직접 연결돼 있다.

- 열린 자세의 엄지 ID1~4와 중지 ID9~12 값은 원본의 5번째 자세(P4)에서 가져왔다. 약지 분리와 검지 뿌리 여유 등은 현재 실물 구조에 맞게 별도 검토했다.
- 현재 큐브 배치 가정은 05:15 실행의 P4 실제 관절값과 같은 실행의 두 면 사진에서 출발한 `legacy_stage4_seating_v1/seed.json`을 사용했다. 사진 정합과 마찰은 근사이므로 절대 위치 보정 완료로 취급하지 않는다.
- 원본 전체 재생, 실제 관절 궤적 재생, 밀기 뒤 다시 잡는 순서도 비교했다. 일부 예전 모델에서 90도 근처까지 간 뒤 큐브 이탈이 있어, 전체 원본을 그대로 실물에 보내는 방식은 채택하지 않았다.
- 최근 실물에서 도달한 파지 자세를 시작점으로 자유 큐브 회전을 계산했다. 마찰 0.2/0.3/0.5와 손 응답 배율 0.5/1/1.5의 9조건에서 약 47.9~57.3도, 명목 조건에서 54.8도였다. 이 결과는 시뮬레이션이며 실물 수평90도 성공은 아니다.

원본 비교 자료는 `../right_hand_rotation_20261024/original_sequence_review/README.md`, 검토 해시와 직접 상속한 값은 `legacy_reference_traceability.json`에 있다. 새로운 회전 구간은 계속 오프라인 계산하며 사용자에게 같은 파지 시험이나 추가 촬영을 반복 요청하지 않는다.
'''
    (OUT/'legacy_reference_usage.md').write_text(text,encoding='utf-8')
    protected={
      'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
      'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
      'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0'}
    for name,expected in protected.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected,name
    print(json.dumps(dict(original_source_matches=True,old_actual_runs=len(sources),original_poses=7,thumb_and_middle_open_endpoints_inherited=True,cube_seed_source=seed['source'],model_robust_yaw_deg=report['current_model_yaw']['final_deg_range'],protected_hashes_unchanged=True,devices_opened=False),ensure_ascii=False))


if __name__=='__main__':main()
