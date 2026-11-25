# 약지 하중 인계 후속: 2026-11-10

<!-- whole_hand_native_result_20261110 -->
모델 검사3개를 실제 실행했다. [최신 실행 결과](whole_hand_preflight_v2/index.html)를 먼저 본다. 이전 승인 사용량 오류로 미실행인 내용은 준비 당시 기록이다.
<!-- whole_hand_native_result_20261110_end -->

최신 정정: ID14 제한은 모델 범위에서 유래하며 기록 자세의 실물 도달 불가를 증명하지 않는다. 정확한 약지 네 값의 정적 비교 스크립트는 작성했지만 자동 승인 검토의 사용량 제한으로 실행되지 않았다. 아래의 “큐브를 놓쳤다”는 최근 정책 평가 표현은 시뮬레이션의 초기 중심 대비80mm 이동 중단을 뜻하며, 손 밖 완전 이탈이나 실물 낙하를 확인한 결과가 아니다. 상세: [약지 기록 자세 재검토 상태](RECORDED_RING_REASSESSMENT_STATUS.md).

사용자의 “전체 동작 준비가 아니면 계속” 요청을 이어 수행했다. **전체 회전→유지→다음 회전→유지의 실물 준비 완료가 아니다.** 원본170개와 과거 결과를 보존한다. 접촉 계산 상한을 높인 별도 모델에서 첫 회전부터 재검증하고 기존 SAC를 보정하는 단계다.

## 접촉 계산 수렴과 정책 보정 — 가장 최근 판정

기존 접촉 계산10회에서는 같은 형상의 약지 끝 약0.020mm 겹침과 검지 끝 약0.0198mm 겹침을 거리 계산과 독립 볼록 형상이 확인했지만 실제 접촉 목록에는 두 손끝이 없었다.50·100·200회에서는 두 접촉을 일관되게 잡았다. 기존 모델도 native CCD는 켜져 있었고, 처음 진단에서 비트 의미를 잘못 붙인 표시는 `CCD_comparison.json`에 정정했다. 형상을 빼거나 접촉 마스크를 바꾸지 않았다.

원본 모델을 보존하고 접촉 계산 상한만10→50으로 바꾼 별도 모델을 만들었다. 모델 배열468개가 동일하며 관절 범위·힘 한도·PID·마찰·형상과 동적 중단 기준은 같다. [MuJoCo 공식 문서의 CCD 반복 설정](https://mujoco.readthedocs.io/en/stable/XMLreference.html#option-ccd-iterations)을 참고했다. 고정 명령4118개를50회와100회로 재생한 최종 qpos/qvel/ctrl/act/warmstart 차이는 전부0이다. 이것은 이 명령열의 수치 비교이며 모든 자세의 수렴이나 실물 정확도를 보장하지 않는다.

**첫 회전부터 다시 실행한 기존 정책은 새 모델에서4118목표 후 큐브80mm 이탈로 중단됐다.** 방향 오차133.922도다. 이전10회 모델의6956목표·약41초 유지 성공은 그대로 보존하지만 수렴 비교를 통과한50회 모델의 성공으로 이전하지 않는다. 따라서 첫 회전·유지까지 포함해 다시 검증해야 하며, 다음 회전이 준비됐다는 판정은 없다.

실제50회 모델 상태에서 기존 SAC actor를 복원한 보정 실험을 수행했다. 이 후속에서 완료한 요청 학습 분량 합계는30,000스텝이며 실제 trainer의 묶음 진행 번호는 각 기록에 별도로 남긴다. 세부: [{"run": "sac_ccd50_repair_10k", "completed": true, "completed_requested_steps": 10000, "trainer_reported_steps": 10000, "horizon": 512, "reset_indices": [3000, 3400, 3600], "positive_valid_step_reward": false, "learning_windows_are_not_full_cycle_success": true}, {"run": "sac_ccd50_recovery_long_10k", "completed": true, "completed_requested_steps": 10000, "trainer_reported_steps": 10000, "horizon": 1200, "reset_indices": [3000, 3400, 3600], "positive_valid_step_reward": true, "learning_windows_are_not_full_cycle_success": true}, {"run": "sac_ccd50_early_recovery_10k", "completed": true, "completed_requested_steps": 10000, "trainer_reported_steps": 10000, "horizon": 1200, "reset_indices": [600, 900, 1200, 1600, 2000, 2600, 3000, 3400], "positive_valid_step_reward": true, "learning_windows_are_not_full_cycle_success": true}]. 초기화가 있는 학습 창의 성능을 연속 회전 성공으로 세지 않는다.

짧은 학습 창은 실제 실패4118목표 전에 끝나는 조건이 있었고 평균 점수가 개선돼도 중간5256/최종10000정책의 독립 초기 배치 평가는 각각4267/4243목표에서 큐브를 놓쳤다. 원래 기록을 보존했다. 이후 모든 창이 실패 지점을 지나도록1200주기로 늘리고, 유효한 주기는 항상 양의 보상이며 실제 중단 시−20을 주도록 바꿨다. 긴 음의 보상을 피하려고 조기 종료가 유리해질 수 있는 구조를 줄이는 변경이며 정책이 실제로 그 편법을 배웠다고 입증한 것은 아니다. 동적 중단 기준과 성공 판정은 같다.

또한 저장 제어 주기 표본에서 첫90도 오차 차이2도 초과를 확인한 구간은 {"step": 1101, "seconds": 82.575, "orientation_error_difference_deg": 4.574820515443438}다. 이는 최초 물리 차이 시점을 확정한 것이 아니라100주기 간격 지표 비교다. 뒷부분만 바꾸는 정책의 한계를 확인하기 위해 실제 초기50회 모델 상태37개를3600주기까지 재현했고 원래 상태와 차이0을 확인했다.600/900/1200/1600/2000/2600/3000/3400에서 시작하는 별도 보정도 비교한다. 각 최종 정책은 초기 배치부터 재평가한다.

독립 초기 배치부터 다시 실행한 결과: [{"path": "next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair10000_active_ccd50/review.json", "steps": 4243, "passed": false, "stop": ["cube_translation_over80mm"], "final": {"error_deg": 135.44416640167054, "tilt_deg": 79.88016299194551, "center_error_mm": 59.99631039877419, "upper_gap_mm": 6.70553862377629}}, {"path": "next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair5256_active_ccd50/review.json", "steps": 4267, "passed": false, "stop": ["cube_translation_over80mm"], "final": {"error_deg": 135.28076129727552, "tilt_deg": 78.82820321857014, "center_error_mm": 60.1740552441034, "upper_gap_mm": 4.833505938699989}}, {"path": "next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair_early10000_active_ccd50/review.json", "steps": 4233, "passed": false, "stop": ["cube_translation_over80mm"], "final": {"error_deg": 134.6858332333056, "tilt_deg": 80.76098691803595, "center_error_mm": 60.34467745982794, "upper_gap_mm": 6.965049614369187}}, {"path": "next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair_long10000_active_ccd50/review.json", "steps": 4313, "passed": false, "stop": ["cube_translation_over80mm"], "final": {"error_deg": 136.3206011258075, "tilt_deg": 79.736092329602, "center_error_mm": 60.20654238511062, "upper_gap_mm": 4.7312280825242015}}].

입력 표본 시점도 따로 점검했다. 세 상태에서 최신 적분 자세와 native 캐시 자세의 차이를 비교했고, 검사한 정책들의 다음 관절 목표는 모두 같았다. 기록은 `observation_timing_audit.json`이다. 세 번의 단일 주기 비교이며 전체 관측 공급기의 동등성이나 실제 카메라 지연 검증으로 확대하지 않는다.

## 기존 정책의 재사용 판정과 보정 범위

완료한 추가 SAC3회·요청 분량3만 스텝의 중간/최종 정책4개를 초기 배치부터 다시 시험했다. 모두 첫 회전·유지 미달이라 실물 실행 정책으로 채택하지 않았다. 총 완료 요청 분량은 이전6만+이번3만=9만이며 실제 trainer 최종 진행 번호도 각3회 모두10000으로 기록됐다. 학습 창 점수 개선을 전체 성공으로 바꾸지 않는다.

이 SAC는 기준 궤적 주위±0.01rad, 약±0.573도/±6.519틱의 목표를 제안하는 잔차 정책이다. 실행 목표의 주기 간격은 여전히 최대2틱이고75ms다. 지령 속도 제한 때문에 실제 지령이 제안 범위 밖에서 잠시 따라올 수 있다는 점과 제안 범위를 구분한다. 별도로 지지가 됐던 초기화 약지 후보와 기존 첫 파지의 약지 네 지령 차이는 약671/−518/−1127/585틱이다. 그 후보를 최소 필수 이동량이라고 입증한 것은 아니지만 현재의 작은 잔차 제안만으로 해당 재파지를 직접 지시하는 구조는 아니다.

원본108차원 정책, 현재110차원 잔차 정책, 증분 재파지 제어를 서로 같은 정책으로 부르지 않는다. 기존 정책을 전부 폐기해야 한다거나 새 학습만이 유일한 해법이라고 결론내리지 않는다. 다음 검토는 수렴 조건의 기준 동작/접촉 모드와 잔차 제어 범위를 함께 맞추는 문제다. 실패한 동작을 증류하지 않는다. [정책 재사용 판정](POLICY_REUSE_DECISION.md)과 [행동 범위 계산](action_authority_audit.json)을 따른다.

## 상태를 초기화하지 않은 연결과 추가 음성 결과

기존10회 모델에서는 초기 배치→실시간 관측 SAC/학습 보정6956목표→저장한 모델 탐색 명령2093목표, 총9049목표를 상태 초기화 없이 재현했다. 여섯 경계의 물리 상태 차이는0이고 동적 기준을 통과했다. 하지만 최종90도 오차14.066도, 위치 오차3.316mm, 마지막 약지·엄지·검지 뿌리 지지 비율0.0988%로 작업 목표는 실패했다. 뒤쪽은 저장 탐색 명령이며 전체 폐루프 학습 정책으로 부르지 않는다.

잠깐 세 지점이 힘을 받는 구간도10.05초 유지에서 살아남지 못했다. 유지 후보48개 비교와 별도 안정 하중 탐색, 모델 중력 보상 세 변형도 성공으로 채택하지 않았다. 검지/엄지를 고정한 약지 후보 탐색2회도 독립 형상 검사와 실제 접촉 조건을 함께 통과하지 못했다. 이런 결과가 모든 재파지의 불가능성을 뜻하지 않는다.



## 새로 확인한 점

- 저장 모델 상태4개의 접촉력을 일반화 구속력과 대조했다. 최대 차이3.33e-16로 좌표계와 부호를 확인했다. 큐브 모델 무게는0.981N이고 대부분을 검지 뿌리가 받친다. 실제 큐브 하중 측정은 아니다.
- 약지 접촉 후보의 끝 자세 자체를 따로 시험했다.81개 모델 압력 지령 비교 중53개가750ms 선별 조건을 통과했다.4개를10.05초 자유 동역학으로 이어 검사했고3개가 마지막4050상태의 약지·엄지·검지 뿌리 힘과5도/2mm/유지 기준을 통과했다. 해당 자세에 직접 초기화한 끝점 검사이며 첫 파지에서의 도달 성공이 아니다. 압력 지령이 실물 전류나 토크 환산값도 아니다.
- 앞서 붙이던seed0은750ms 선별부터 통과한 압력 조합이 없었다. 원래 힘을 유지한 채 경로만 따라가는 문제 외에 목표 끝 자세 자체의 지지력도 확인할 필요가 있었다. 기존 실패 기록을 보존하고 검증된seed1의 끝 자세로 경로를 다시 만들었다.
- 검지 접촉을 끝까지 고정하는 대신 검지 또는 약지 접촉의 기하적 전환을 허용하고 엄지도 포함해 탐색했다. 이것은 탐색 비용/접촉 모드 변경이며 실제 관절·충돌·추종·2틱/75ms/1ms 기준의 완화가 아니다.
- 현재 정수 관측 첫 회전 끝 상태에서 새 경로는1265목표를 전부 진행하고 원래 동적 기준을 통과했다. 큐브를 놓치지는 않았지만 최종90도 자세 오차15.936도, 마지막 약지 지지 비율0.0%로 전환 성공은 아니다. 세부 상태는 `rrt_rf_seed5_integer_equilibrium3/`에 있다.

## 끝 보정

위 실제 경로 끝 상태의qpos/qvel/ctrl/act/warmstart 차이0을 확인하고 보정을 연결했다. 자세를 원래 목표로 맞추는 보정은 방향 오차를 약15도에서8도까지 줄였지만 지지력을 확보하지 못했다. 손끝 거리와 실제 법선력을 구분한다. 작은 움직임의 독립 진단에서 검지–중지 중간 링크 간섭도 확인했고, 해당 분기를 거부한 채 가능한 동시 관절 움직임을 비교한다.

끝 자세 보정 중 경로 진행 번호가 더 증가하지 않는다는 이유로 중단하던 탐색 로직을 고쳤다. 원래 동적 기준은 그대로다. 의도적으로 탐색을 바꾸며 중단한v4/v5는 `offline_search_stop_requested`로 보존했고 실패한 실물 실행으로 세지 않는다. 새 변형은 실제 현재 자세에서 약지·엄지 하중을 먼저 확보한 다음 원래 목표로 서서히 복귀한다. 기하학적 계획 분기는 후보를 제안할 뿐 실제 큐브 상태를 고정하지 않는다.

- [handoff_equilibrium1_feedback_v2](../../next_cycle/handoff_equilibrium1_feedback_v2/review.json): 317목표, 중단`no_safe_two_tick_correction_found`, 임시 약지 지지미달, 방향 오차19.086도, 위치 오차10.404mm.
- [handoff_equilibrium1_integer_start_v2](../../next_cycle/handoff_equilibrium1_integer_start_v2/review.json): 356목표, 중단`no_safe_two_tick_correction_found`, 임시 약지 지지미달, 방향 오차41.252도, 위치 오차20.752mm.
- [handoff_integer_endpoint_close_v3](../../next_cycle/handoff_integer_endpoint_close_v3/review.json): 140목표, 중단`reference_progress_stalled`, 임시 약지 지지미달, 방향 오차10.965도, 위치 오차2.716mm.
- [handoff_integer_endpoint_close_v4](../../next_cycle/handoff_integer_endpoint_close_v4/review.json): 189목표, 중단`offline_search_stop_requested`, 임시 약지 지지미달, 방향 오차8.147도, 위치 오차2.421mm.
- [handoff_integer_endpoint_close_v5](../../next_cycle/handoff_integer_endpoint_close_v5/review.json): 270목표, 중단`offline_search_stop_requested`, 임시 약지 지지미달, 방향 오차9.258도, 위치 오차2.279mm.
- [handoff_integer_load_first_v6](../../next_cycle/handoff_integer_load_first_v6/review.json): 229목표, 중단`offline_search_stop_requested`, 임시 약지 지지미달, 방향 오차12.976도, 위치 오차1.956mm.
- [handoff_integer_load_first_v7](../../next_cycle/handoff_integer_load_first_v7/review.json): 170목표, 중단`no_safe_two_tick_correction_found`, 임시 약지 지지미달, 방향 오차27.141도, 위치 오차12.309mm.
- [handoff_wrench_free_v1](../../next_cycle/handoff_wrench_free_v1/review.json): 157목표, 중단`no_safe_two_tick_correction_found`, 임시 약지 지지미달, 방향 오차26.916도, 위치 오차11.341mm.
- [handoff_wrench_root_fixed_v1](../../next_cycle/handoff_wrench_root_fixed_v1/review.json): 411목표, 중단`no_safe_two_tick_correction_found`, 임시 약지 지지미달, 방향 오차32.561도, 위치 오차15.314mm.

최종 보고서가 아직 없는 계산: 없음. 최종 판정은 각`review.json`과 `summary.json`을 따른다. 초기화한 끝 자세의 유지, 실제 전환 경로, 다음90도와 최종 윗단 공간은 서로 다른 검증이다. 약지 임시 지지 자세에 윗단 공간이 확보됐다고 주장하지 않는다.

[준비 상태](summary.json) · [접촉력 검산](support_audit.json) · [끝 자세 검사](endpoint_screen/review.json) · [작은 움직임 진단](local_move_diagnostics.json) · [기존 첫 회전·유지](../STATUS.md)
