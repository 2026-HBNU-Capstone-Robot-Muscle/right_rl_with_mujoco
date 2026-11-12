"""Summarize saved offline comparisons; standard library, no device access."""
import json,hashlib
from pathlib import Path
R=Path(__file__).resolve().parent
O=R/'right_hand_rotation_20261105/canonical_body_grip'
PROTECTED={
'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
'screen_yaw_roundtrip_model_20261030.py':'c9097f13bce11a1cb457e7670b0e916fd1e71c6fce72c175baa874e9e4b72e10',
'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d',
'horizontal_quarter_motion_runner_20261027.py':'5c8487f134264f5ef4eb9f6bdf6a465e5fb6c69945c4a91407e1b6d75cc760cb'}

def main():
    checks={p:hashlib.sha256((R/p).read_bytes()).hexdigest()==h for p,h in PROTECTED.items()}
    if not all(checks.values()):raise RuntimeError('Protected source changed: '+str(checks))
    dynamic=[]
    for p in sorted((O/'easy_suffix').rglob('review.json')):
        z=json.loads(p.read_bytes())
        for c in z.get('cases',[]):
            d=c['dynamic'];dynamic.append(dict(source=str(p.relative_to(R)),gain=c['gain_scale'],friction=c['friction'],original_guard_pass=d['safety_passed'],yaw_change_deg=d['continuation_yaw_deg'],full_layer_sweep_gap_mm=c['minimum_final_full_sweep_gap_mm'],terminal_center_error_mm=c['target_center_error_mm'],tilt_deg=c['final_up_tilt_deg'],last_4_05s_hold_yaw_drift_deg=d['hold_drift_deg'],last_4_05s_hold_shift_mm=d['hold_shift_mm'],lower_contact_fraction=d['lower_body_opposed_contact_fraction'],all_suffix_tasks_pass=bool(c['suffix_task_passed'] and abs(d['continuation_yaw_deg'])<5)))
    for p in sorted((O/'staged_regrasp').glob('*/review.json')):
        z=json.loads(p.read_bytes());d=z.get('dynamic')
        if d:dynamic.append(dict(source=str(p.relative_to(R)),gain=1.,friction='unchanged checkpoint material',original_guard_pass=d['safety_passed'],yaw_change_deg=d['continuation_yaw_deg'],full_layer_sweep_gap_mm=z['minimum_final_sweep_gap_mm'],terminal_center_error_mm=z['terminal_center_error_mm'],tilt_deg=z['final_tilt_deg'],last_4_05s_hold_yaw_drift_deg=d['hold_drift_deg'],last_4_05s_hold_shift_mm=d['hold_shift_mm'],lower_contact_fraction=d['lower_body_opposed_contact_fraction'],all_suffix_tasks_pass=bool(z.get('suffix_task_passed') and abs(d['continuation_yaw_deg'])<5 and z['terminal_center_error_mm']<5)))
    report=dict(status='choose_easiest_feasible_terminal_grip_not_a_prescribed_location',user_priority='Choose the least complicated location that leaves the upper layer free and withstands its actual turn, then return to that terminal pose after reorientation.',initial_place_and_terminal_hold_need_not_match=True,existing_successful_placement_is_retained_as_first_option=True,comparison_scope='Saved free-cube continuations from the nominal checkpoint. Entire placement/reorientation matrix and actual other-hand torque have not been verified for a new grip.',model_registration_to_actual_cube_verified=False,model_baseline_yaw_deg=61.256,dynamic_cases=dynamic,passing_suffix_count=sum(c['all_suffix_tasks_pass'] for c in dynamic),full_layer_sweep_clear_count=sum(c['full_layer_sweep_gap_mm']>.3 for c in dynamic),physical_location_selected=False,upper_layer_reaction_load_verified=False,all_reorientation_to_common_grip_connections_verified=False,hardware_execution_allowed=False,assistant_motor_writes=0,devices_opened=False,protected_sources_unchanged=checks)
    (O/'selection_review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    note='''# 파지 위치 선택 기준

위치를 먼저 정하고 손을 억지로 맞추지 않는다. 윗단을 돌릴 수 있고 하단을 버티는 후보 중에서 가장 간단한 것을 고른다.

1. 오른손 전체가 윗단의 90도 회전 공간을 비운다.
2. 다른 손이 실제 윗단을 돌릴 때 하단과 큐브 전체가 따라 돌아가지 않는다.
3. 필요한 큐브 방향 변경 후 같은 중심, 높이, 기울기로 다시 잡을 수 있다.
4. 통과한 후보 중 추가 관절 동작과 재파지 시간이 적고, 사람이 처음 놓는 위치에 덜 민감한 후보를 고른다.

큐브를 처음 놓는 자리와 다른 손이 작업할 때의 파지 자리는 달라도 된다. 방향을 바꾸는 도중에 큐브가 조금 이동하는 것도 허용한다. 다른 손이 작업할 때의 최종 위치를 맞추는 것이 목표다.

현재는 이미 성공한 배치와 회전을 유지하면서 최소한의 재파지를 추가하는 안을 우선 비교한다. 손바닥, 검지, 검지와 중지 사이 중 어느 한 곳을 미리 확정하지 않는다.

저장된 새 후보의 시뮬레이션 결과는 selection_review.json에 있다. 정지한 최종 자세만 맞는 것과 자유롭게 놓인 큐브가 실제 동작 중 그 자세로 이동하는 것을 구분했다. 윗단 공간이 비더라도 재파지 중 큐브 방향이 틀어지는 후보는 채택하지 않는다.

현재 모델의 기존 회전 재현은 약 61도이며 실물 영상의 약 90도와 차이가 있다. 새 후보의 수치가 실물에서 그대로 재현된다고 보장하지 않는다. 실제 다른 손의 회전 하중과 최종 위치 재현성 확인 전에는 ‘어느 방향에서든 버틴다’고 판정하지 않는다. 새 실물 실행 명령은 만들지 않았다.
'''
    (O/'selection_note.md').write_text(note,encoding='utf-8')
    print(json.dumps(dict(dynamic_cases=len(dynamic),passing_suffix_count=report['passing_suffix_count'],clear_count=report['full_layer_sweep_clear_count'],protected_sources_unchanged=all(checks.values()),new_hardware_command_created=False)))
if __name__=='__main__':main()
