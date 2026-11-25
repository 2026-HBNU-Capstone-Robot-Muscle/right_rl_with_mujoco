# 2026-11-11 첫 회전·유지 실물 결과 검토

첫 시행은 사용자가 배치 오류를 알렸으므로 올바른 배치의 동작 평가에서 제외한다. 원본은 보존한다. 두 시행 모두 실제 사용자 실행 결과다.

두 번째 시행에서는 초록 윗면을 유지한 채 주황 앞면이 흰 앞면으로 바뀌는 큐브 전체 회전을 사진과 선정 프레임에서 확인했다. 실제 유지 구간134샘플의 시각 범위는 약10.094초이고, 그 사이 저장77프레임 모두에서 흰면 마커ID0이 검출됐다. 유지 시작·끝의 선택 프레임에서도 큐브가 손 위에 남아 있다. 프로그램 정상 종료와 토크OFF·설정 복구도 확인했다. 이는 첫 회전→짧은 유지의 실물 관찰이며 다음 회전과 윗단 작업은 아직 시험하지 않았다. 관찰 범위는 [visual_review.json](visual_review.json), 실제 종료 관절 참고는 [actual_first_turn_end_reference.json](actual_first_turn_end_reference.json)에 있다.

## 시행 1: forward_hold_observe_20261111_220404_103327

실행 상태: `forward_hold_observation_complete`, 오류: `None`. 토크OFF 확인: True, 설정 복구: True.

고정 목표 유지 134샘플, 실제 샘플 시각 범위 10.156초. 전압 최저 4.8V, 최대 절대 전류 108mA, 최대 추종오차 0.087437rad.

[cube_before_grasp.jpg](../../forward_hold_observe_20261111_220404_103327/cube_before_grasp.jpg)
[cube_after_fixed_hold.jpg](../../forward_hold_observe_20261111_220404_103327/cube_after_fixed_hold.jpg)

- [grasp_hold: 135.95s](../../forward_hold_observe_20261111_220404_103327/camera/frame_01656.jpg) (stage_6_grasp_hold)
- [turn_0: 139.03s](../../forward_hold_observe_20261111_220404_103327/camera/frame_01680.jpg) (stage_7_horizontal_yaw)
- [turn_1: 147.16s](../../forward_hold_observe_20261111_220404_103327/camera/frame_01742.jpg) (stage_7_horizontal_yaw)
- [turn_2: 155.17s](../../forward_hold_observe_20261111_220404_103327/camera/frame_01803.jpg) (stage_7_horizontal_yaw)
- [turn_3: 163.16s](../../forward_hold_observe_20261111_220404_103327/camera/frame_01866.jpg) (stage_7_horizontal_yaw)
- [turn_4: 171.30s](../../forward_hold_observe_20261111_220404_103327/camera/frame_01928.jpg) (stage_7_horizontal_yaw)
- [turn_5: 179.27s](../../forward_hold_observe_20261111_220404_103327/camera/frame_01989.jpg) (stage_7_horizontal_yaw)
- [turn_6: 187.44s](../../forward_hold_observe_20261111_220404_103327/camera/frame_02051.jpg) (stage_8_horizontal_hold)
- [hold_0: 187.44s](../../forward_hold_observe_20261111_220404_103327/camera/frame_02051.jpg) (stage_8_horizontal_hold)
- [hold_1: 190.05s](../../forward_hold_observe_20261111_220404_103327/camera/frame_02071.jpg) (stage_8_horizontal_hold)
- [hold_2: 192.56s](../../forward_hold_observe_20261111_220404_103327/camera/frame_02090.jpg) (stage_8_horizontal_hold)
- [hold_3: 195.09s](../../forward_hold_observe_20261111_220404_103327/camera/frame_02110.jpg) (stage_8_horizontal_hold)
- [hold_4: 197.66s](../../forward_hold_observe_20261111_220404_103327/camera/frame_02130.jpg) (stage_2_fixed_hold)

## 시행 2: forward_hold_observe_20261111_220908_830958

실행 상태: `forward_hold_observation_complete`, 오류: `None`. 토크OFF 확인: True, 설정 복구: True.

고정 목표 유지 134샘플, 실제 샘플 시각 범위 10.094초. 전압 최저 4.7V, 최대 절대 전류 100mA, 최대 추종오차 0.075165rad.

[cube_before_grasp.jpg](../../forward_hold_observe_20261111_220908_830958/cube_before_grasp.jpg)
[cube_after_fixed_hold.jpg](../../forward_hold_observe_20261111_220908_830958/cube_after_fixed_hold.jpg)

- [grasp_hold: 141.45s](../../forward_hold_observe_20261111_220908_830958/camera/frame_01686.jpg) (stage_6_grasp_hold)
- [turn_0: 144.52s](../../forward_hold_observe_20261111_220908_830958/camera/frame_01710.jpg) (stage_7_horizontal_yaw)
- [turn_1: 152.50s](../../forward_hold_observe_20261111_220908_830958/camera/frame_01772.jpg) (stage_7_horizontal_yaw)
- [turn_2: 160.52s](../../forward_hold_observe_20261111_220908_830958/camera/frame_01833.jpg) (stage_7_horizontal_yaw)
- [turn_3: 168.64s](../../forward_hold_observe_20261111_220908_830958/camera/frame_01895.jpg) (stage_7_horizontal_yaw)
- [turn_4: 176.67s](../../forward_hold_observe_20261111_220908_830958/camera/frame_01958.jpg) (stage_7_horizontal_yaw)
- [turn_5: 184.64s](../../forward_hold_observe_20261111_220908_830958/camera/frame_02021.jpg) (stage_7_horizontal_yaw)
- [turn_6: 192.77s](../../forward_hold_observe_20261111_220908_830958/camera/frame_02083.jpg) (stage_8_horizontal_hold)
- [hold_0: 192.77s](../../forward_hold_observe_20261111_220908_830958/camera/frame_02083.jpg) (stage_8_horizontal_hold)
- [hold_1: 195.38s](../../forward_hold_observe_20261111_220908_830958/camera/frame_02103.jpg) (stage_8_horizontal_hold)
- [hold_2: 197.84s](../../forward_hold_observe_20261111_220908_830958/camera/frame_02122.jpg) (stage_8_horizontal_hold)
- [hold_3: 200.34s](../../forward_hold_observe_20261111_220908_830958/camera/frame_02141.jpg) (stage_8_horizontal_hold)
- [hold_4: 202.86s](../../forward_hold_observe_20261111_220908_830958/camera/frame_02160.jpg) (stage_8_horizontal_hold)

사진 기반 각도는 저장된 카메라 내부 파라미터와 큐브55mm·마커10.7mm 가정의 근사값이다. 이번 다면 마커 자세 적합은 수용되지 않았으므로 숫자 각도를 만들지 않는다. 마커 장착·로봇 좌표계 정합 및 실제 힘 측정이 없으므로 정밀90도·파지력·윗단 작업 하중·새 두 회전 연결 성공을 인증하지 않는다. 종료 후OFF 처짐은 구동 중 유지 실패로 세지 않는다.

장치를 새로 열거나 모터에 출력하지 않았다. 원본 사진·로그는 변경하지 않았다.
