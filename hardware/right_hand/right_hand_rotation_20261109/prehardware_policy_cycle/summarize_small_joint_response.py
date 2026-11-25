"""A limited model sensitivity answer, separate from physical error claims."""
from pathlib import Path
import json
import hashlib

OUT=Path(__file__).resolve().parent
F=OUT/'next_cycle/handoff_refinement_20261110'
ROOT=OUT.parents[1]


def main():
    path=F/'small_joint_response/review.json';report=json.loads(path.read_text())
    rows=[]
    for sample in report['samples']:
        valid=[r for r in sample['results'] if r['comparison_valid']]
        top=max(valid,key=lambda r:r['cube_angle_change_vs_matched_hold_deg'])
        rows.append(dict(source_goal_index=sample['parent_goal_index'],comparisons=len(valid),
            blocked=sample['blocked_comparisons'],maximum_cube_change_deg=top['cube_angle_change_vs_matched_hold_deg'],
            motor_id=top['motor_id'],target_offset_ticks=top['offset_ticks'],
            actual_joint_change_deg=top['actual_joint_angle_change_vs_matched_hold_deg']))
    original=ROOT/'ring_reference_readonly_20261109_124719_945552/result.json'
    result=dict(probe_report_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),probe_summary=rows,
        target_change_deg=report['perturbation_target_deg'],comparison_seconds=1.275,
        no_general_average_or_physical_joint_to_cube_conversion_claim=True,
        small_residual_radius_not_established_as_primary_failure_cause=True,
        ring_motor_ids_tip_to_base=[13,15,14,16],recorded_ring_ticks=[3018,2961,2829,3053],
        ring_original_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
        ring_was_recorded_torque_off_and_used_as_bounded_search_seed=True,
        ID14_record2829_above_model_upper2730_54_ticks_approx=98.46,
        this_is_not_a_measured_hardware_safety_limit=True,
        devices_opened=0,hardware_execution_allowed=False)
    (F/'small_joint_response/answer_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    table='\n'.join(f"| {r['source_goal_index']} | {r['maximum_cube_change_deg']:.3f}° | ID{r['motor_id']} | {r['actual_joint_change_deg']:.3f}° |" for r in rows)
    text=f'''# 작은 관절 변화와 저장 약지 자세: 2026-11-10

0.57도는 측정된 관절 오차가 아니라 SAC 제안 보정폭±0.01rad다. 이를 큐브 오차로 바로 환산하는 비율을 측정한 자료는 없다.

저장된50회 모델 상태3개에서 한 관절의 목표만±7틱(약±{report['perturbation_target_deg']:.3f}도) 바꾸고 다른15개 관절 지령을 유지했다. 정수 모터 지령이므로6.519틱과 정확히 같지는 않다. 매 주기 최대2틱,75ms와 원래 동적 기준을 유지했다.17주기·1.275초 뒤 같은 상태에서 원래 지령을 유지한 대조군과 비교했다.96조건 모두 동적 기준을 통과했다.

| 원래 경로 상태 | 큐브 추가 자세 변화의 최대값 | 해당 관절 | 그 관절의 실제 추가 변화 |
|---|---:|---|---:|
{table}

세 모델 접촉 상태와 짧은 시간의 변화량이며 실물 평균 오차나 일반 허용 오차가 아니다. 여러 관절 동시 변화, 긴 시간의 미끄럼과 다른 배치도 아직 이 시험에 포함하지 않았다. 접촉하지 않는 관절도 포함하므로 전체 중앙값을 잡는 관절의 보통 반응으로 부르지 않는다. 이 결과만으로 작은 보정폭을 전체 실패의 주원인으로 단정하지 않는다.

약지 네 값은 이미 저장돼 있다. 손끝→손바닥 ID13/15/14/16은3018/2961/2829/3053틱이다.2026-11-09 토크OFF 상태에서 두 번 읽었고 차이는0이었다. 다시 읽을 필요는 없다. `ring_support_probe.py`, `cooperative_support_search.py`, `search_fixed_grip_ring_support.py`에서도 탐색 초깃값으로 사용했다. 모델 순서는 ID16/14/15/13이므로 코드에는3053/2829/2961/3018로 나타난다.

ID14의2829틱은 현재 모델 상한 약2730.54틱보다98.46틱, 약8.65도 높다. 실제 기록과 모델 설정의 차이이며 실제 하드웨어 안전 한도를 새로 측정한 결과는 아니다. 원래 범위를 자동 확대하지 않고 범위 안으로 제한한 탐색 참고값으로 썼다. 기록된 네 값을 그대로 구동해 하중을 받는 검증은 아니다. 앞서 최적화로 찾은 별도 모델 후보도 사용자의 자세를 그대로 재현한 것과 구분한다. 그 후보까지의 큰 이동량을 최소 필수 이동량으로 해석하지 않는다.

[모델 비교 원본](review.json) · [수치 요약과 약지 출처](answer_summary.json) · [기존 정책 판정](../POLICY_REUSE_DECISION.md)
'''
    (F/'small_joint_response/STATUS.md').write_text(text,encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
