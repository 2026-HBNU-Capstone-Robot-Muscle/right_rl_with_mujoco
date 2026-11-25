# 첫 회전·유지의 출력 없는 준비 묶음

2026-11-10. 이 폴더에는 모터·시리얼·카메라·ROS 출력 기능이 없다. 다음90도와 양손 통합의 실물 실행 파일이 아니다.

`staged_shadow_runtime.py`의 두 클래스가 같은 정책 자산을 쓴다.

- `StagedShadowController`: 세 학습 조건의 소수 틱 모델 평가6556목표를 그대로 재현하는 비교 기준.
- `AdmissibleShadowController`: 끝 보정의 목표를 원래 관절 범위 안의 가장 가까운 정수 틱으로 선택한다. 현재 모델의 정수 인코더 명목 시험에서 초기 배치부터6956목표와 약41.55초 위치 유지가 통과했다. 같은6956개 지령을 새 구현에서도 독립적으로 생성해 차이0을 확인했다. 이 재현을 독립적인 두 성공 조건으로 세지 않는다.

앞6300목표는 기존 기준 궤적에 SAC 잔차를 더하는 제어다. 이후 별도 교사를 모방한 보정 정책을 사용하고, 획득 뒤에는 고정 위치 지령을 유지한다. 고정 위치 지령 동안 원래 모델의 위치 서보가 작동한다. 토크 OFF나 OPEN은 없다. 끝 보정을 SAC 학습이나 전체 회전 정책 증류로 부르지 않는다. 기본 보정 actor는v2이며 인코더 증강v3는 별도 비교 자산으로 보존했다.

`propose(sample)`는 목표를 계산하고 `commit_shadow(sequence, goal_ticks)`가 같은 목표의 확인을 받아야 제어 단계가 바뀐다. 확인된 목표와 다음 관측의 지령이 다르면 거부한다. `next_rotation_capability()`는 준비false를 반환하고 파지 유지 지령을 바꾸지 않는다.

입력 계약은 실제 장치 연결 완료를 뜻하지 않는다.

- 모터 배열은 ID1~16, 시뮬레이션 순서는 `(8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1)`이다.
- `frame_id=leap_sim_world`, `palm_point=policy_palm_position`을 명시한다. 위치는미터, 쿼터니언은wxyz다. 실물 손 본체 원점을 학습 손바닥 센서점으로 대체하지 않는다.
- 순서가 증가하는 `sequence`, `sample_time`, `pose_time`과 `measured_ticks`, `commanded_ticks`, `palm_position`, `cube_position`, `cube_quaternion`을 제공한다. 기본 연결부는 처음 세 표본 동안 위상을 진행하지 않는다. 모델 확인에는 반복된 초기 관측을 썼으며 실제 세 카메라 표본 검증은 아니다.
- 끝 보정의 `upper_sweep_acquisition`은 `model_sha256`, 같은`sequence`, `method=slab_clipped_radial_screen`, 유한한`screen_mm`을 포함한다. 이 값은 획득 판단용 단면 지표이며 원통 전체의 최소거리 하한이나 실물 여유 인증이 아니다. 실제 모델 판정에는 별도 정확한 원통 지지 함수 인증을 마지막4050개1ms 상태에 적용했다.

원래 관절 제한·최대2틱/75ms·충돌·추종 기준을 유지했다. 정수 관측의 명목 시험에서 최종90도 자세 오차1.36867도, 위치 오차1.49802mm, 마지막4050상태 하부 접촉100%, 윗단 인증 하한0.57195mm였다. 물체 자세와 공간 지표에는 정확한 모델을 사용했으므로 실물 센서 오차나 하중 대응의 검증이 아니다.

학습에 없던Y+0.5mm는 각도와 위치를 맞춰도 원통 여유가 미달했고 가정 마찰0.35는 앞 SAC 구간에서 큐브를 놓쳤다. 약지 하중 인계와 다음90도도 미완료다. 물리 실행 가능 표시는false로 유지한다.

상세 근거: [현재 상태](../STATUS.md), [준비 상태와 각 결과 경로](../readiness.json), [관측·확인 응답 검사](../staged_shadow_runtime_verification.json), [정수 관측의 새 묶음 확인](../staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/review.json).
