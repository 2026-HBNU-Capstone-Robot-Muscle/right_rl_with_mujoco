import json,datetime as dt
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/four_forward_review'
def main():
 now=dt.datetime.now().astimezone().isoformat();status='rotation_and_stable_grasp_review_after_four_attempts'
 for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
  f=R/name;r=json.loads(f.read_bytes());archive=O/'previous_status';archive.mkdir(exist_ok=True);saved=archive/f.name
  if not saved.exists():saved.write_bytes(f.read_bytes())
  r.update(status=status,current_status=status,timestamp=now,latest_actual_run='single_forward_trial_20261030_053131_966030',latest_actual_review=(O/'run_summary.json').relative_to(R).as_posix(),
   next_command=None,next_user_command=None,next_user_command_ready=False,next_user_action_required=False,next_user_action=None,ready_for_user_run=False,ready_for_supervised_operator_run=False,single_forward_trial_ready=False,new_single_forward_trial_not_yet_run=False,
   active_priority='Right hand reorientation followed by a stable body grasp for upper-layer turning; integrate later reorientation/hold cycles.',
   placement_as_sole_cause_not_proven=True,new_forward_rotation_verified=False,stable_grasp_under_layer_turn_torque_verified=False,assistant_motor_or_setting_writes=0)
  f.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 f=R/'right_hand_rotation_20261030/single_forward/handoff.json';r=json.loads(f.read_bytes());(O/'previous_single_forward_handoff.json').write_bytes(f.read_bytes());r.update(status=status,command=None,completed_command='leap_single_forward_trial.cmd',ready_for_supervised_operator_run=False,operator_action_required=False,physical_new_execution_not_yet_run=False,latest_actual_run='single_forward_trial_20261030_053131_966030',followup_review=(O/'run_summary.json').relative_to(R).as_posix());f.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 note=('<!-- grasp_hold_priority_after_four_20261030 -->\n2026-10-30 사용자 네 실행을 확인했다. 05:21은회전408/638에서User stop,05:25는동작전모델복귀3/9거부,05:26은전체회전638과고정40후OPEN592/669에서User stop,05:31은전체2576지령과OPEN/제거/OFF/설정복구까지완료했다. '
       '마지막사진에서빈손도보여큐브제거질문은필요없다. 카메라검정0장이고구동전압최저4.8/4.9V,최대전류92/101/104mA다. '
       '세번째사진은주황앞면에서흰앞면으로바뀌었으나마지막은큐브가앞쪽으로밀려주황앞면이남았다. 배치만의문제라고단정하지않는다. '
       '사용자는방향변경뒤몸통을고정하여위쪽레이어를돌리고다시방향변경/고정하는흐름을요구했다. 현재우선순위는오른손회전과그뒤안정파지다. 이전정회전단독반복명령은다음작업에서내렸다. '
       '고정지령유지만으로실제클램프를인증하지않으며접촉보완을검토한다. 근거: right_hand_rotation_20261030/four_forward_review/run_summary.json\n\n')
 for name in ('professor_progress_20261015.md','sac_current_status.md'):
  f=R/name;s=f.read_text(encoding='utf-8');
  if not s.startswith('<!-- grasp_hold_priority_after_four_20261030 -->'):f.write_text(note+s,encoding='utf-8')
 print('Current priority and completed attempts recorded; no next motion command yet.')
if __name__=='__main__':main()
