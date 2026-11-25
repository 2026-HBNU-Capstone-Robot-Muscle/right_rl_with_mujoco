"""Publish completed observations, preserve original evidence, retire stale next-run pointers."""
import datetime as dt,hashlib,json
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/roundtrip_actual_review';O.mkdir(exist_ok=True)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def put(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def archive(p):
 a=O/'previous_status';a.mkdir(exist_ok=True);q=a/(p.stem+'_'+sha(p)[:12]+p.suffix)
 if not q.exists():q.write_bytes(p.read_bytes())
 return q.relative_to(R).as_posix()
def main():
 first=R/'yaw_roundtrip_trial_20261030_010851_904040';second=R/'yaw_roundtrip_trial_20261030_011416_269699'
 reports=[json.loads((f/'offline_review/physical_and_angle_review.json').read_bytes()) for f in (first,second)]
 transfer=json.loads((second/'offline_review/side3_to_green_rotation_review.json').read_bytes());a=reports[1]
 assert all(x['all_actual_goal_writes_match_ON_rebased_queue'] and x['all_torque_off_verified'] and x['settings_restored'] and not x['cube_present'] for x in reports)
 fw=transfer['placement_to_forward_camera_relative_p05_median_p95_deg']['twist_deg'][1]
 inv=a['comparisons']['forward_to_inverse']['camera_relative']['twist_deg_p05_median_p95'][1]
 drift=a['comparisons']['inverse_to_open']['camera_relative']['twist_deg_p05_median_p95'][1]
 assert 75<fw<95 and -60<inv<-30 and 0<drift<10 and transfer['consistent_calibration_pairs']>=3
 now=dt.datetime.now().astimezone().isoformat();ping=R/'connection_ping16_readonly_20261030_043052_319531/result.json'
 report=dict(status='two_physical_roundtrips_reviewed_inverse90_incomplete',timestamp=now,latest_actual_run=second.name,
     runs=[{k:v for k,v in x.items() if k!='rows'} for x in reports],second_forward_estimate_deg=fw,second_inverse_estimate_deg=inv,second_open_drift_estimate_deg=drift,
     second_forward_calibration=transfer,second_forward_angle_is_camera_relative_estimate=True,metric_robot_registration_verified=False,
     programmed_inverse_explains_subsequent_counter_turn=True,no_placement_step_between_legs=True,
     placement_affects_contact_hypothesis_not_proven_sole_cause=True,first_run_low_rotation_does_not_prove_operator_error=True,
     inverse90_verified=False,repeatability_verified=False,all_directions90_verified=False,full_cube_solve_verified=False,
     latest_readonly_ping=ping.relative_to(R).as_posix(),latest_ping_responding_ids=[],current_motor_state_verified=False,
     user_action_required='Check and enable existing 5V motor power and connection for PING/READ only. Cube outside hand. Leave fingers/base as they are.',
     next_motion_command=None,ready_for_user_motion=False,assistant_motor_or_setting_writes=0,
     videos={f.name:json.loads((f/'offline_review/video_exports.json').read_bytes()) for f in (first,second)},
     evidence_hashes={p.relative_to(R).as_posix():sha(p) for p in (first/'result.json',second/'result.json',second/'offline_review/side3_to_green_rotation_review.json',ping)})
 put(O/'review.json',report)
 handoff=R/'right_hand_rotation_20261030/yaw_roundtrip/handoff.json';h=json.loads(handoff.read_bytes());archive(handoff)
 h.update(status=report['status'],timestamp=now,command=None,completed_command='leap_yaw_roundtrip_trial.cmd',ready_for_supervised_operator_run=False,
     operator_action_required=True,operator_steps=['Check and enable existing 5V supply for PING/READ. No motion program yet.'],entire_composed_cycle_not_yet_physically_tested=False,
     latest_physical_run=second.name,latest_physical_inverse_estimate_deg=inv,latest_physical_forward_estimate_deg=fw,
     all16_OFF_error0_watchdog0_verified=False,current_motor_state_verified=False,current_readonly=ping.relative_to(R).as_posix(),current_readonly_sha256=sha(ping),
     current_pose_cache_checked=False,current_off_difference_ticks=None,current_pose_requires_fresh_read=True,
     physical_observations_complete=True,physical_inverse90_verified=False,roundtrip_observation_review=(O/'review.json').relative_to(R).as_posix())
 put(handoff,h)
 for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
  p=R/name;old=json.loads(p.read_bytes());archive(p);old.update(status=report['status'],current_status=report['status'],timestamp=now,
      next_command=None,next_user_command=None,next_user_command_ready=False,next_user_action_required=True,next_user_action=report['user_action_required'],next_result_prefix=None,
      ready_for_user_run=False,ready_for_supervised_operator_run=False,ready_scope='OBSERVATION_REVIEW_COMPLETE_NEXT_MOTION_UNDER_RESEARCH',
      roundtrip_trial_ready=False,roundtrip_physical_trial_not_yet_run=False,physical_roundtrip_observation_complete=True,physical_roundtrip_verified=False,
      inverse_90_ready=False,all_directions_90_verified=False,physical_inverse90_verified=False,
      latest_actual_run=second.name,latest_actual_review=(O/'review.json').relative_to(R).as_posix(),physical_forward_estimate_deg=fw,physical_inverse_estimate_deg=inv,
      latest_readonly=ping.relative_to(R).as_posix(),latest_start_proof=None,latest_pose_difference_from_review_ticks=None,source_off_drift_ticks=None,
      motors_off_verified=False,current_motor_state_verified=False,current_motor_connection='Adapter opened at1Mbps; all16 PINGs unanswered. No motor state certified by this read.',
      inverse_research_progress='Completed continuous test separates effective forward (~84deg) and incomplete programmed inverse (~42deg). Full existing reference stroke is under offline comparison; no execution handoff.',
      assistant_motor_or_setting_writes=0)
  put(p,old)
 # Korean operator/progress prose; preserve historical sections below.
 text=(f'<!-- actual_roundtrip_review_20261030 -->\n'
       f'2026-10-30 최신 실물 확인: 01:08과01:14 왕복 실행은 두 번 모두 지령 전체, 자동OPEN, 큐브 제거 확인, 토크OFF와 설정 복구까지 완료했다. '
       f'첫 번째는 정회전 약16도에 그쳤고, 두 번째는 배치부터 고정까지 약{fw:.2f}도 정회전, 다음 구간에서 약{abs(inv):.2f}도 역회전했다. '
       f'OPEN 때 추가 방향 변화는 약{drift:.2f}도였다. 사용자가 재배치로 느낀 구간은 프로그램에 포함된 역회전이며 중간 PLACE 단계는 없다. '
       '두 번째 시작의 초록ID5 격자는 가려져, 같은 실행에서 동시에 보인 주황ID3와 초록 격자10프레임으로 축 관계를 구하고 시작12프레임에 적용했다. '
       '각도는 카메라 기준 추정으로 정밀 로봇 좌표 인증이 아니다. 정지 전후 초록-흰면 상대 자세는 평면 자세의 중의성을 위축으로 구분한 뒤 약1.4도 변했다. '
       '정지 영상에서 큰 레이어 엇갈림의 근거는 없으나 전체 동작의 레이어 불변을 인증하지 않는다.\n\n'
       '두 번째 정회전 발췌103.3초와 전체348.3초 영상을 각각 보존했다. 발췌는 역회전 전에서 끝나며 전체 실행이나 큐브 맞추기 완성 영상으로 표시하지 않는다. '
       '두 실행 모두 검정 프레임0, 읽기 실패2회씩 모두 복구됐다. 최저 전압은4.8/4.6V, 최대 전류107/111mA다. '
       '첫 실패와 배치 차이는 접촉 민감성의 단서이며 사용자의 배치 실수만이 원인이라고 단정하지 않는다. 역회전90도와 반복 재현성은 미완료다.\n\n'
       '이전 왕복 실행을 다음 명령으로 표시하던 포인터를 완료 상태로 바꿨다. 04:30 읽기 검사에서는 USB는 열리지만16개 모터 모두 PING에 응답하지 않았다. '
       '현재 사용자 작업은 큐브를 손 밖에 둔 상태에서 기존5V 전원과 연결을 확인해 켜는 것이다. 손가락과 받침은 그대로 두고 동작 프로그램은 실행하지 않는다. '
       '그동안 예전 전체1302지령 회전/역회전의 오프라인 비교를 진행한다. 새 목표 좌표는 없으나 준비를 합치면 기존300초 한도를 넘어 이 비교 후보는 실물 실행에 제공하지 않는다. '
       '근거: right_hand_rotation_20261030/roundtrip_actual_review/review.json\n\n')
 for name in ('professor_progress_20261015.md','sac_current_status.md'):
  p=R/name;archive(p);old=p.read_text(encoding='utf-8-sig');marker='<!-- actual_supported_inverse32_20261029_231345 -->'
  if old.startswith('<!-- continuous_yaw_ready_20261030 -->') and marker in old:old=old[old.index(marker):]
  if not old.startswith('<!-- actual_roundtrip_review_20261030 -->'):p.write_text(text+old,encoding='utf-8')
 full='../../'+second.name+'/offline_review/full_run_realtime.mp4';excerpt='../../'+second.name+'/offline_review/forward_only_excerpt_realtime.mp4'
 photos=[('cube_before_grasp.jpg','배치 직후'),('cube_before_horizontal_turn.jpg','잡은 뒤'),('cube_forward_turn_hold.jpg',f'정회전 후 약{fw:.0f}°'),('cube_inverse_turn_hold.jpg',f'이어서 역회전 약{abs(inv):.0f}°')]
 gallery=''.join(f'<figure><img src="../../{second.name}/{n}" alt="{label}" loading="lazy"><figcaption>{label}</figcaption></figure>' for n,label in photos)
 html=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>실제 왕복 회전 결과</title>
<style>body{{font:18px/1.65 "Malgun Gothic",sans-serif;max-width:1100px;margin:32px auto;padding:0 20px;color:#1b2934}}h1{{font-size:28px}}.status{{padding:18px;background:#e8f2fa;border-left:5px solid #276b9a}}.gallery{{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:18px}}figure{{margin:0}}img,video{{width:100%;border-radius:7px;background:#111}}figcaption{{padding:5px}}.note{{color:#52616d;font-size:16px}}a{{color:#125a8c}}@media(max-width:600px){{.gallery{{grid-template-columns:1fr}}}}</style>
<h1>두 번째 실행은 약 {fw:.0f}° 정회전</h1><p class="status">이어서 약 {abs(inv):.0f}° 역회전했습니다. 다시 비틀리는 것으로 보였던 부분은 프로그램의 역회전 구간입니다. 두 실행은 모두 정상 종료와 토크 OFF를 확인했습니다.</p>
<p>정회전 영상과 역회전을 포함한 전체 영상을 구분해 보존했습니다. 90° 역회전은 아직 완료하지 못했습니다.</p>
<div class="gallery">{gallery}</div>
<h2>정회전 구간 발췌 — 1분 43초</h2><video controls preload="metadata" src="{excerpt}"></video><p class="note">역회전 시작 전에 끝나는 발췌입니다. 실제 속도로 재생하며 전체 실행 영상은 아래에 있습니다.</p>
<h2>두 번째 전체 실행 — 5분 48초</h2><video controls preload="metadata" src="{full}"></video>
<p class="note">배치와 제거를 기다린 시간까지 포함합니다. 원본 사진과 시간 기록을 사용했으며 각도는 카메라 기준 추정입니다. 큐브 전체 맞추기는 아직 완료하지 않았습니다.</p>
<h2>지금 필요한 작업</h2><p>큐브를 손 밖에 둔 채 기존 5V 전원과 USB 연결을 확인하고 켠 뒤 알려주세요. 손가락과 받침은 그대로 두면 됩니다. 다음 경로를 검토하는 동안 현재 각도를 읽겠습니다.</p>
<p><a href="../roundtrip_actual_review/review.json">측정 및 실행 기록</a></p></html>'''
 p=R/'right_hand_rotation_20261030/yaw_roundtrip/operator_guide.html';archive(p);p.write_text(html,encoding='utf-8');(O/'index.html').write_text(html,encoding='utf-8')
 print(json.dumps(dict(status=report['status'],forward_deg=fw,inverse_deg=inv,guide=p.relative_to(R).as_posix(),next_motion_command=None,original_image_changes=0,assistant_motor_writes=0),ensure_ascii=False))
if __name__=='__main__':main()
