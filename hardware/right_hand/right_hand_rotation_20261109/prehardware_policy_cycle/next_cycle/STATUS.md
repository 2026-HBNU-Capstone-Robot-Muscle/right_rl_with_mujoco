# 실기 전 후속 준비: 2026-11-10

**첫 회전·유지는 접촉 계산10회 모델의 세 조건에서 학습 보정까지 연결했다. 수렴 비교한50회 모델의 성공으로 이전하지 않는다. 다음 회전과 실물 준비 완료는 아니다.** 최신 수치 진단과 추가 SAC 결과는 [접촉 계산·약지 인계 후속](handoff_refinement_20261110/STATUS.md)을 먼저 따른다. 물리 장치·원격 저장소·미들웨어 UI를 사용하거나 변경하지 않았다.

## 처음부터 연결한 첫 회전·유지

초기 배치→기준 궤적과 SAC 잔차6300목표→학습한 증분 보정→위치 지령 유지, 총6556목표를 세 조건에서 새로 실행했다. 손·큐브 상태 초기화는 없고 앞부분의 기존 SAC 경계 상태 차이는0이었다. 끝 보정에는 저장 교사 관절 목표를 재생하지 않았다.

| 모델 조건 | 회전 | 전체90도 자세 오차 | 공통 위치 오차 | 윗단 인증 하한 | 하부 접촉1ms 상태 |
|---|---:|---:|---:|---:|---:|
| nominal | 89.121° | 1.121° | 1.004mm | 0.431mm | 4050/4050 |
| x_plus05 | 89.353° | 0.789° | 0.578mm | 0.313mm | 4050/4050 |
| friction025 | 89.049° | 0.999° | 0.761mm | 0.373mm | 4050/4050 |

접촉·윗단 인증은 마지막4.05초의4050상태다. 양 손끝 법선력 각각0.2N 초과를 하부 접촉으로 센 모델 평가이며 실물 하중 측정이 아니다. 세 조건이 교사 학습 자료에도 포함돼 있으므로 독립적인 일반화 성공률이 아니다.6556목표의 모델 제어 시간은491.7초이며 속도를 최적화하지 않았다.

앞부분은 기준 궤적+SAC 잔차, 끝 보정은 교사 모방 학습, 이후는 원래 위치 서보다. 유지 중 토크를 끄지 않는다. 전체 순수 SAC나 전체 회전 정책 증류 완료로 부르지 않는다. 완료 SAC의 요청 학습 분량 합계는90,000스텝이며 실제 trainer의 묶음 진행 번호는 각 progress.json에 별도로 남긴다. 별도 모방 학습3회의 합계15,000최적화 갱신을 여기에 더하지 않는다.

## 실제 입력 조건의 모델 시험

- [staged_model_friction035_fractional_encoders](../next_cycle/staged_model_friction035_fractional_encoders/review.json): 미달, 3978목표, 자세 오차82.324도, 위치 오차62.020mm. 범위 안 행동 선택0회. 중단: `['cube_translation_over80mm']`.
- [staged_model_nominal_fractional_encoders_recovery_bc_v3_long_encoder_aug](../next_cycle/staged_model_nominal_fractional_encoders_recovery_bc_v3_long_encoder_aug/review.json): 미달, 6329목표, 자세 오차3.085도, 위치 오차0.230mm. 범위 안 행동 선택0회. 중단: `['shadow_contract_rejection', 'Correction proposed an out-of-range goal']`.
- [staged_model_nominal_integer_encoders_admissible_correction_extra_hold400](../next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400/review.json): 통과, 6956목표, 자세 오차1.369도, 위치 오차1.498mm. 범위 안 행동 선택18회. 중단: `[]`.
- [staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_active_ccd50](../next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_active_ccd50/review.json): 미달, 4118목표, 자세 오차133.922도, 위치 오차60.327mm. 범위 안 행동 선택0회. 중단: `['cube_translation_over80mm']`.
- [staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair10000_active_ccd50](../next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair10000_active_ccd50/review.json): 미달, 4243목표, 자세 오차135.444도, 위치 오차59.996mm. 범위 안 행동 선택0회. 중단: `['cube_translation_over80mm']`.
- [staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair5256_active_ccd50](../next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair5256_active_ccd50/review.json): 미달, 4267목표, 자세 오차135.281도, 위치 오차60.174mm. 범위 안 행동 선택0회. 중단: `['cube_translation_over80mm']`.
- [staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair_early10000_active_ccd50](../next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair_early10000_active_ccd50/review.json): 미달, 4233목표, 자세 오차134.686도, 위치 오차60.345mm. 범위 안 행동 선택0회. 중단: `['cube_translation_over80mm']`.
- [staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair_long10000_active_ccd50](../next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_ccd50repair_long10000_active_ccd50/review.json): 미달, 4313목표, 자세 오차136.321도, 위치 오차60.207mm. 범위 안 행동 선택0회. 중단: `['cube_translation_over80mm']`.
- [staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation](../next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/review.json): 통과, 6956목표, 자세 오차1.369도, 위치 오차1.498mm. 범위 안 행동 선택18회. 중단: `[]`.
- [staged_model_nominal_integer_encoders_extra_hold400](../next_cycle/staged_model_nominal_integer_encoders_extra_hold400/review.json): 미달, 6307목표, 자세 오차4.285도, 위치 오차1.250mm. 범위 안 행동 선택0회. 중단: `['shadow_contract_rejection', 'Correction proposed an out-of-range goal']`.
- [staged_model_nominal_integer_encoders_recovery_bc_v3_long_encoder_aug_admissible_correction_extra_hold400](../next_cycle/staged_model_nominal_integer_encoders_recovery_bc_v3_long_encoder_aug_admissible_correction_extra_hold400/review.json): 통과, 6956목표, 자세 오차1.861도, 위치 오차0.749mm. 범위 안 행동 선택6회. 중단: `[]`.
- [staged_model_nominal_integer_encoders_recovery_bc_v3_long_encoder_aug_extra_hold400](../next_cycle/staged_model_nominal_integer_encoders_recovery_bc_v3_long_encoder_aug_extra_hold400/review.json): 미달, 6315목표, 자세 오차3.999도, 위치 오차1.127mm. 범위 안 행동 선택0회. 중단: `['shadow_contract_rejection', 'Correction proposed an out-of-range goal']`.
- [staged_model_y_plus05_fractional_encoders](../next_cycle/staged_model_y_plus05_fractional_encoders/review.json): 미달, 6353목표, 자세 오차1.088도, 위치 오차0.549mm. 범위 안 행동 선택0회. 중단: `['shadow_contract_rejection', 'Correction proposed an out-of-range goal']`.
- [staged_model_y_plus05_fractional_encoders_admissible_correction](../next_cycle/staged_model_y_plus05_fractional_encoders_admissible_correction/review.json): 미달, 6556목표, 자세 오차1.013도, 위치 오차1.019mm. 범위 안 행동 선택6회. 중단: `[]`.
- [staged_model_y_plus05_fractional_encoders_recovery_bc_v3_long_encoder_aug](../next_cycle/staged_model_y_plus05_fractional_encoders_recovery_bc_v3_long_encoder_aug/review.json): 미달, 6305목표, 자세 오차4.323도, 위치 오차0.966mm. 범위 안 행동 선택0회. 중단: `['shadow_contract_rejection', 'Correction proposed an out-of-range goal']`.

첫v2 부품은 소수 틱 모델 관측의 세 조건을 재현했으나 정수 인코더 관측에서는 끝 보정7주기 뒤 범위 밖 목표 제안으로 거부됐다. 실제 범위 밖 목표를 실행한 것이 아니다. 미학습Y+0.5mm도 끝 보정 중 범위 제안을 거부했고, 가정 마찰0.35는 앞 SAC 구간에서 큐브80mm 이탈로 중단됐다. 원래 결과를 보존했다.

긴 피드백 교사 세 궤적768표본은 재생 상태 오차0을 확인했다. 현재 각도와 최신 관절 오차 이력에 같은 반 틱 잡음을 주고 과거 이력에는 별도 잡음을 주는 정수 인코더 증강을 추가해v3 보정 부품을 별도 학습했다. 위 표의 결과가 실제 폐루프 판정이며 모방 정확도로 대체하지 않는다. 검증 표본을 학습에도 쓰므로 훈련 자료 재현 점수를 독립 성능으로 부르지 않는다.

증강 학습만으로 미달은 해결되지 않았다. 원래 범위 안에서 가장 가까운 증분 목표를 선택하는 별도 변형은 정수 인코더 명목 모델에서 초기 배치부터6956목표를 통과했다. v2 부품의 최종 자세 오차1.36867도, 공통 위치 오차1.49802mm, 마지막4050상태 하부 접촉100%, 원통 여유 인증 하한0.57195mm다. 위치 유지 전환 뒤 약41.55초를 이어갔으며 원래 관절/충돌/추종 기준을 통과했다. 범위를 늘리거나 범위 밖 목표를 실행하지 않았다. v3+범위 안 선택도 통과했지만 다른 정책 변형의 동일 명목 조건으로 구분한다. 준비 묶음의 `AdmissibleShadowController`가 이 변형이며 다음 회전 기능은 없다.

## 다음 회전이 막힌 구간

약지가 큐브에 닿는 기구학적 자세를 찾았다. 다른 손가락을 고정하면 간섭이 크지만 검지·중지를 함께 조정한 정적 자세4개와 검지 끝 접촉을 남기는 정적 자세3개는 형상 검사를 통과했다. 손가락 이동 여섯 순서의12개 비교와 RRT 우회 경로를 시험했다. 손가락 겹침 없는 경로도 실제 자유 동역학에서 큐브를 놓쳤다. 기하적 접촉 유지와 실제 지지력 유지가 다르다.

지지력 피드백·여러 관절 동시2틱 후보·150ms 앞 간섭·검지 뿌리 받침 고정도 비교했지만 약지 하중 인계를 끝내지 못했다. 새 학습 보정 뒤 파지에서 다음 절대180도 목표도144목표 후 오차86.263도, 공통 위치 오차10.329mm로 정체됐다. 다음90도 성공은 없다. 모든 재파지의 불가능성을 입증한 결과는 아니다.

## 연결부와 보존한 원본

출력 API 없는 NumPy 단계 연결부를 만들고, 세 조건의19668목표가 저장 모델 평가와 전부 일치함을 확인했다. 오래된 표본·모델/순서가 다른 공간 근거·확인 응답 오류를 거부하며 확인 전에는 제어 단계를 바꾸지 않는다. 지원하지 않는 다음 회전 요청은 유지 지령을 바꾸지 않는다. 이 검사는 소수 틱 모델 관측이며 실물 카메라/인코더 정확도 검증이 아니다.

빠른 절단 단면/방사 지표는 원통 끝면 주변의 전체 최소거리 하한이 아니다. X+0.5mm의 빠른 지표0.141mm와 독립 분리면 인증0.313mm는 다른 계산이다. Y+0.5mm+범위 안 선택에서는 목표 각도·위치를 맞췄지만 독립 인증이 실패했다. 끝 상태 검지 끝의 독립 최근접점 거리 상한0.258743884mm와 정확한 지지 함수의 분리 하한0.258743884mm가 일치해0.3mm 여유 미달을 확인했다. 단면 지표를 큰 공간 여유 증명으로 쓰지 않는다. 연결부v2의 입력 명칭도 인증과 획득용 지표를 구분하도록 고쳤고 앞선v1 묶음은 보존했다. 형상과0.3mm 기준은 그대로다. 기존 SAC 단독 마찰0.25도 독립 인증이 실패했으므로 통과로 바꾸지 않았다.

native 거리0과 최근접점 불일치가 있던 초기 `workspace_rf/`, `workspace_mf/` 진단은 철회 표시를 하고 보존했다. 기존 독립 볼록 형상 검사로 `workspace_*_checked/`를 다시 계산했다. 실패한 긴 고정 유지 자료와 명목 위치2mm 초과 교사는 학습에 채택하지 않았다.

실물450mA/PID/관절 범위/2틱/75ms/전압·전류·추종 중지 기준과 원래170개 결합 원본을 유지했다. 수동 ID14 참고값으로 범위를 확대하지 않았다. 현재 손·카메라 등록과 힘 한도 환산은 미검증이다. 사용자의 재연결·재촬영·제거 확인을 요구하지 않는다.

[그림과 전체 비교](../review/index.html) · [준비 상태](readiness.json) · [단계 연결부 검사](staged_shadow_runtime_verification.json) · [기존 SAC와 교사 재생 기록](../WORK_STATUS.md)
