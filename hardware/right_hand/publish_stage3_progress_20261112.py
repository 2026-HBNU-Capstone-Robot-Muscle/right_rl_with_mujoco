"""Record a compact honest handoff without opening devices or changing proofs."""
import hashlib
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261112/stage3_preparation'
reports = []
for path in sorted(OUT.glob('*/review.json')):
    r = json.loads(path.read_bytes())
    if 'conditional_bridge_and_hold_passed' in r:
        reports.append({'path': path.relative_to(ROOT).as_posix(), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                        'steps': r['steps'], 'stop': r['stop'], 'best_remaining_angle_deg': r['best_sampled_error_deg'],
                        'relative_turn_and_hold_passed': r['conditional_bridge_and_hold_passed']})
pair = json.loads((OUT/'opposed_contact_v1/review.json').read_bytes())
current = json.loads((ROOT/'cycle_test_readonly_20261112_050645_574736/result.json').read_bytes())
result = {'status': 'next_rotation_preparation_not_hardware_ready', 'relative_teacher_results': reports,
          'opposed_regrasp_model_trials': len(pair['trials']), 'opposed_regrasp_model_passed': sum(t['paired_contact_passed'] for t in pair['trials']),
          'fresh_read_only_source': 'cycle_test_readonly_20261112_050645_574736/result.json',
          'fresh_read_only_errors': current['errors'], 'fresh_read_only_servo_writes': current['servo_writes'],
          'whole_physical_cycle_passed': False, 'new_motion_file_ready': False, 'old_stage2_motion_file_changed': False,
          'native_joint_collision_current_timing_limits_changed': False, 'new_SAC_training_steps': 0,
          'assistant_motor_writes': 0, 'remote_writes': 0}
(OUT/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
lines = ['# 다음 회전 준비', '', '**연속 동작은 아직 미완료이며 새 실물 실행 파일은 준비 완료로 제공하지 않았다.**', '',
         '첫 회전→파지 전환→약10초 손 위 지지 관찰은 이미 완료됐다. 그 사실을 미완료로 되돌리지 않는다.', '',
         '이번에는 실제 실행한 stage2의 지령과 그 지령을 재생한 저장 모델 상태에서 후속 경로를 비교했다. 저장 모델 상태를 실제 큐브의 3차원 상태 복원으로 취급하지 않는다. 기존 경로의 단순 연결은 추종 기준에 걸렸고, 재파지 순서를 바꾼 일부 경로는 손 지지를 유지해도 다음 회전 목표가 미달했다. 마찰·구동 조건에서 차이가 커서 지지만 통과했다고 회전 성공으로 표시하지 않았다.', '',
         f'도달한 모델 큐브 위치에 맞춘 중지·엄지 맞은편 접촉의 동적 재파지 {len(pair["trials"])}개 중 {sum(t["paired_contact_passed"] for t in pair["trials"])}개가 지지·양쪽 하중·국소 유지 검사를 통과했다. 이는 마찰0.2 가정의 모델 결과이며 실제450mA 힘 환산이나 실물 성공이 아니다.', '',
         '이어진 회전 제어 비교는 아래 파일에 실패를 포함해 보존했다. v2는 두 번째 회전 약45도 진행 후 앞으로의 검지 중간 마디–중지 뿌리 간섭과 관절 범위 예측으로 중단됐다. 이 중단을 실행 중 큐브 낙하나 실물 고장으로 설명하지 않는다. v3는 해당 쌍을 계획 비용에 추가한 별도 비교이며, v4는 중지·엄지 하중 접촉을 유지하는 별도 비교다. 각 결과의 passed와 실제 종료 원인을 먼저 확인한다.', '',
         '예측 계산을 줄인 v2 이후 후보는 가상 예측의 충돌 검사를5ms 간격으로 했고 실제 선택 지령이 진전하는 모델 상태는 매1ms 기존 기준으로 검사했다. 이것은 실물 제어가 아니다. 모든 모델 한계값·450mA·기존PID·2틱/75ms와 기존300초 실행 제한은 변경하지 않았다.', '',
         '연결된 원래 어댑터를1Mbps PING16/READ48만 읽었다.16개 모두 두 번 저장됐고 오류0·토크OFF였다. 포트를 닫았고 보조자 모터·토크·설정 쓰기는0이다. 이 읽기는 새 동작 승인이 아니다. 현재 시작각으로 준비 시간을 산정한 current_start_budget.json 역시 안전 검증이 아닌 시간 계산이다.', '',
         '우선순위는 이 후속 회전→유지 연결이다. UI·미들웨어·원격 업로드·새SAC 학습으로 범위를 넓히지 않는다. 같은 첫 회전 시연이나 추가 사진을 사용자에게 반복 요구하지 않는다.', '',
         '## 저장된 후속 제어 결과', '']
for r in reports:
    lines.append(f'- [{r["path"].split("/")[-2]}]({r["path"].split("/")[-2]}/review.json): {r["steps"]}목표, 종료 `{r["stop"]}`, 최소 남은 각도 {r["best_remaining_angle_deg"]:.3f}도, 회전·유지 통과 {r["relative_turn_and_hold_passed"]}.')
(OUT/'STATUS.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print(json.dumps({'new_hardware_ready': False, 'paired_model_grips_passed': result['opposed_regrasp_model_passed'], 'completed_teacher_comparisons':len(reports)}))
