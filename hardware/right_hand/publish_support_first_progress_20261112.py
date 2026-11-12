"""Summarize the new task without relabelling old angle-task failures."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261112/support_first_transfer'
def main():
    first=json.loads((OUT/'review.json').read_bytes())
    path=OUT/'thumb_push_5mm/review.json';push=json.loads(path.read_bytes())
    if 'intermediate_transfer_waypoint_passed' in push:
        push.pop('intermediate_transfer_waypoint_passed')
        push.update(sustained_middleward_translation_observed=bool(push['supported_hold_passed'] and push['achieved_toward_middle_mm']>0),
                    waypoint_completion_certified=False,
                    translation_waypoint_error_mm=abs(push['translation_waypoint_mm']-push['achieved_toward_middle_mm']),
                    metadata_clarification='4.117mm observed against5mm probe waypoint; neither exact5mm nor final grasp station certified; dynamics not rerun for this wording correction')
        path.write_text(json.dumps(push,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    upper=OUT/'upper_link_review.json'
    summary={'status':'support_first_user_strategy_partial_offline_validation','receiving_pad_cases':[
        {k:r[k] for k in ('case','intermediate_supported_hold_passed','base_geom_center_height_difference_mm','hold_samples','supported_samples')}
        for r in first['rows']],
        'thumb_translation':{k:push[k] for k in ('supported_hold_passed','achieved_toward_middle_mm','orientation_change_diagnostic_deg','middle_lower_pad_loaded_hold_fraction','waypoint_completion_certified')},
        'upper_link_cases':json.loads(upper.read_bytes())['rows'] if upper.exists() else [],
        'upper_link_comparison_complete':upper.exists(),'final_left_acquisition_alignment_verified':False,
        'physical_success':False,'hardware_execution_file_updated':False,'devices_opened':0,
        'intermediate90_error_not_a_gate_in_this_new_transfer_task':True,
        'historical90degree_failures_preserved':True,'native_safety_and_joint_tracking_guards_unchanged':True}
    summary['continuous_replay_cases']=[]
    for mu in (.3,.2):
        p=OUT/f'continuous_mu{mu:g}/review.json'
        if p.exists():
            r=json.loads(p.read_bytes());summary['continuous_replay_cases'].append({k:r[k] for k in (
                'mu','passed','done','total','all1ms_samples','hold_samples','supported_samples','upper_certified_samples',
                'minimum_upper_mm','hold_shift_mm','hold_drift_deg','tilt_deg','intermediate90_error_diagnostic_deg',
                'actual_state_resets_after_initial_placement','candidate_sha256')})
    if len(summary['continuous_replay_cases'])==2 and all(r['passed'] for r in summary['continuous_replay_cases']):
        summary['status']='continuous_transfer_model_validated_left_alignment_and_next_turn_pending'
    summary['next_rotation_verified']=False
    (OUT/'STATUS.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if summary['continuous_replay_cases']:
        lines=['# 先받침을 만든 뒤 옮겨잡기: 모델 검토','',
            '사용자 제안에 따라 회전 뒤 중지 받침을 검지와 맞춘 다음 엄지로 중지 쪽으로 옮기고, 위마디와 엄지를 작업 파지로 옮기는 경로를 구성했다. 중간90도 오차는 진단값이고 최종 왼손 파지 시의 위치·방향 정렬은 아직 검증하지 않았다. 기존90도 작업의 실패 기록은 보존했다.','',
            '앞1417지령은 이전 실물에서 실행한 첫 회전 경로와 동일하다. 이후 중지 받침 준비266지령, 엄지 이동214지령, 위마디/엄지 작업 파지971지령을 연결해2868지령을 만들었다. 매지령75ms·최대2틱, 기존관절/추종/건강/접촉기준을유지했다. 중간에큐브상태를초기화하지않았다.','',
            '| 마찰 가정 | 연속 검사 | 최종 지지/윗단 여유 | 최소 윗단 여유 | 유지 이동 | 유지 방향 변화 |',
            '| --- | --- | --- | --- | --- | --- |']
        for r in summary['continuous_replay_cases']:
            lines.append(f"| {r['mu']} | {r['done']}/{r['total']}지령·{r['all1ms_samples']}개1ms상태 | {r['supported_samples']}/{r['hold_samples']}·{r['upper_certified_samples']}/{r['hold_samples']} | {r['minimum_upper_mm']:.3f}mm | {r['hold_shift_mm']:.3f}mm | {r['hold_drift_deg']:.3f}도 |")
        lines+=['','두 마찰은 측정값이 아닌 모델 가정이다. 받침 정렬 첫 구간의 높이 차이는 약0.234mm이고, 엄지 이동 구간에서 중지 쪽으로 약4.117mm 이동한 뒤 지지를 유지했다. 이때 방향도 약11.600도 바뀌었다. 연구용5mm 중간 목표의 정확한 도달이나 최종 파지 위치가 인증된 것은 아니다.','',
            '검지 위마디만 또는 검지→중지 위마디만 움직인 비교는 윗단 공간과 기울기가 미달했다. 중지 위마디→정렬한 두 받침과 검지→엄지 순서로 이동한 경로는 새 중간 파지 과제의 지지·공간·안정 조건을 통과했다. 엄지를 먼저 작업 자세로 옮기는 다른 순서도 별도 모델에서 통과했지만, 채택한 연속 경로는 앞의 순서다.','',
            '**남은 범위:** 새 경로의 실제 낙하 방지 효과, 최종 왼손 정렬, 이후 회전 연결, 배치·구동 오차, 다른 손의 실제 하중은 미검증이다. 마지막 모델의90도 오차는마찰0.3에서43.079도/0.2에서49.996도이며,이를실물오차로환산하거나왼손이그오차를보정할수있다고보장하지않는다. 현단계통과를양손전달완료나전체연속회전완료로세지않는다.','',
            '새 실물 실행 파일은 반영하지 않았다. 기존 `leap_cycle_nominal_two_turns.cmd`는 실제 낙하한v3이므로 이번 새 경로를 실행하는 파일로 안내하지 않는다. 장치접근/구동/원격쓰기0.','',
            '근거: [STATUS.json](STATUS.json), [마찰0.3 연속검토](continuous_mu0.3/review.json), [마찰0.2 연속검토](continuous_mu0.2/review.json), [사용자 전략](STRATEGY.md).']
        ready=ROOT/'right_hand_rotation_20261112/support_first_trial/READY.json'
        if ready.exists():
            lines=[('새 경로의 별도 실물 관찰 파일 `leap_cycle_support_first_transfer.cmd`를 준비했다. [실행 안내](../support_first_trial/operator_guide.md)와 [준비 결과](../support_first_trial/READY.json)를 확인한다. 기존v3과는 다른 경로이고 새 실물 결과는 아직 없다. 장치접근/구동/원격쓰기0.' if line.startswith('새 실물 실행 파일은 반영하지 않았다.') else line) for line in lines]
            summary['hardware_execution_file_updated']=True
            summary['operator_entry']='leap_cycle_support_first_transfer.cmd'
            (OUT/'STATUS.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        (OUT/'README.md').write_text('\n'.join(lines).replace('# 先받침','# 받침')+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k!='upper_link_cases'},ensure_ascii=False))
if __name__=='__main__':main()
