"""Separate middle-finger teaching evidence from whole-hand OFF sag."""
import hashlib,json,sys
from pathlib import Path
R=Path(__file__).resolve().parent;OUT=R/'right_hand_rotation_20261106/manual_grasp'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    sys.stdout.reconfigure(encoding='utf-8');source=R/'manual_grasp_teach_20261106_183247_578628/result.json';r=json.loads(source.read_bytes());assert r['status']=='manual_joint_pose_and_photo_saved' and r['all_observed_torque_off'] and r['servo_writes']==r['torque_changes']==0;folder=R/r['corrected_pose_folder'];pair=json.loads((folder/'result.json').read_bytes());assert pair['status']=='photo_and_readings_saved' and pair['pair_check']['stationary_endpoints'];before=r['last_powered_position_ticks'];goal=r['last_powered_goal_ticks'];off=r['before_manual_OFF_snapshots'][-1]['position'];corrected=r['corrected_position_ticks'];ids=[9,11,10,12]
    rows=[dict(motor_id=i,last_powered_actual_ticks=before[i-1],last_powered_goal_ticks=goal[i-1],initial_OFF_ticks=off[i-1],saved_OFF_ticks=corrected[i-1],change_from_initial_OFF_ticks=corrected[i-1]-off[i-1],change_from_last_powered_ticks=corrected[i-1]-before[i-1]) for i in ids]
    report=dict(status='middle_encoder_and_photo_saved_not_yet_a_powered_target',source=source.relative_to(R).as_posix(),source_sha256=sha(source),photo=(folder/'paired_photo.png').relative_to(R).as_posix(),photo_sha256=sha(folder/'paired_photo.png'),middle_tip_to_root_ids=ids,middle=rows,photo_bracket_maximum_drift_ticks=pair['pair_check']['maximum_drift_ticks'],whole_hand_OFF_pose_not_adopted_as_goal=True,gravity_sag_and_manual_change_not_separated=True,manual_root_correction_not_inferred_from_sag=True,other_finger_angles_not_adopted=True,thumb_lowering_requires_separate_review=True,new_powered_motion_not_created=True,devices_opened=False)
    report.update(user_reported_whole_hand_sag_after_OFF=True,repeat_OFF_teaching_not_requested=True,cube_removal_after_teaching_not_confirmed=True,
        user_clarification='The middle finger was bent enough to let the uppermost cube layer rotate; this is a functional curvature reference, not merely a direction cue.',
        middle_all_four_saved_angles_are_approximate_shape_evidence=True,
        middle_root_angle_not_discarded=True,
        intended_goal='Retain lower-body support while clearing the complete upper-layer rotation sweep.',
        instruction_to_repeat_OFF_teaching=False)
    (OUT/'middle_teach_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    note='''# 중지 기록 확인\n\n사용자는 중지를 단순히 어느 방향으로 움직인 것이 아니라, 맨 윗단이 돌아갈 정도로 굽혔다고 명확히 설명했다. 저장한 네 관절의 굽힘 정도를 윗단 공간 확보를 위한 대략적인 자세 기준으로 사용한다. 뿌리 관절도 이 참고에서 제외하지 않는다.\n\n사용자는 토크 OFF 때 손이 처져 원하는 자세를 유지하기 어려웠다고 보고했다. 이 방식으로 추가 촬영을 요청하지 않는다.\n\n중지의 손끝부터 ID9, 11, 10, 12에 대해 토크 ON 마지막 실제값과 목표, OFF 직후 값, 최종 저장 값을 모두 확보했다. 최종 사진의 앞뒤 관절값 차이는 0틱이다. 중력 처짐과 의도적인 보정은 분리하지 못했으므로 최종 OFF 자세 전체를 새 구동 목표로 사용하지 않는다.\n\n기존 파지를 출발점으로 중지의 기록된 굽힘을 참고하고 엄지를 낮추는 후보를 검토한다. 큐브 하부 지지와 윗단 전체 회전 공간을 함께 확인한다. 다른 손가락의 처진 자세는 채택하지 않는다. 실제 모터를 다시 켜는 경로는 아직 만들지 않았다.\n\n보정 후 큐브를 제거했는지는 아직 확인되지 않았다. 다음 빈손 복귀 전에 확인해야 한다.\n''';(OUT/'latest_observation.md').write_text(note,encoding='utf-8')
    progress=R/'professor_progress_20261015.md';data=progress.read_bytes();end=b'<!-- manual_grasp_teach_20261106_end -->'
    if data.startswith(b'<!-- manual_grasp_teach_20261106_current -->') and end in data:
        header,history=data.split(end,1);text=header.decode('utf-8');old='- 새 보정 동작은 사용자가 아직 실행하지 않았다. 다음에는 `manual_grasp_teach_*`의 결과를 읽고, 보정한 각도와사진을 확인한 뒤 원래 관절/충돌 기준으로 검토한다. 저장 각도를 즉시토크ON으로 실행하지 않는다.'
        new='- 실제 보정 기록 `manual_grasp_teach_20261106_183247_578628`을 확인했다. 중지ID9/11/10/12의ON 실제/목표,OFF 직후,최종 저장값과 사진이 있다. 사진 전후 차이는0틱이다. 사용자가OFF 때 손 전체가 처진다고 보고했으므로 추가OFF 보정 촬영을 요청하지 않고, 중력 처짐을 그대로 새 목표로 쓰지 않는다. 엄지 낮추기와 중지 기록을 별도 검토한다. 큐브 제거는 미확인이다.'
        if old in text:progress.write_bytes(text.replace(old,new).encode('utf-8')+end+history)
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
