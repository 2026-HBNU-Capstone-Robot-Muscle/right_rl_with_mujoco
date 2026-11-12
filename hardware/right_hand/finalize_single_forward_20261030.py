"""Publish a concrete supervised next action after all saved checks pass."""
import json,datetime as dt,re
from pathlib import Path
from html.parser import HTMLParser
import numpy as np
from single_forward_plan_20261030 import SingleForwardPlan,ROOT,OUT,REQUEST,PROOF,LOADED,HAND_PATH,sha,hashes
from single_forward_gate_20261030 import validate_review,run_screen
from observed_book_model_policy import healthy_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS
def put(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
 p=SingleForwardPlan(REQUEST);proof=validate_review(p,PROOF);test=json.loads((OUT/'software_admission_result.json').read_bytes())
 assert test['status']=='passed' and test['tests_run']==15 and test['failures']==test['errors']==0 and test['controller_and_model_hashes']==hashes()
 assert test['source_sha256']==sha(ROOT/'test_single_forward_20261030.py')
 source=ROOT/'fast_off_readonly_20261030_051447_982334/result.json';r=json.loads(source.read_bytes());assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2 and r['servo_writes']==r['torque_changes']==0
 for s in r['snapshots']:healthy_off({k:np.asarray(v) for k,v in s.items()})
 difference=np.asarray(r['snapshots'][-1]['position'])-p.source_position();assert max(abs(difference))<=4,'Fresh prefix review needed before publication'
 live=OUT/'latest_off_request.json';cached=OUT/'latest_off_review.json';put(live,dict(plan_version=p.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),hardware_execution_allowed=False));run_screen(live,cached);validate_review(SingleForwardPlan(live),cached)
 protected={'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b','right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716','exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0','camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d','horizontal_quarter_motion_runner_20261027.py':'5c8487f134264f5ef4eb9f6bdf6a465e5fb6c69945c4a91407e1b6d75cc760cb','screen_material_checked_reverse_cube_20261029.py':'79adde54e2d3a8e45bbd895b0f50b5c297536c0556ff508e39b54f567e3a3d66'}
 for name,digest in protected.items():assert sha(ROOT/name)==digest,name
 assert sha(Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'))=='008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'
 now=dt.datetime.now().astimezone().isoformat();command='leap_single_forward_trial.cmd';guide=OUT/'operator_guide.html';rows=p.plan(p.source_position())
 report=dict(status='same_completed_single_forward_supervised_trial_ready',timestamp=now,command=command,guide=guide.relative_to(ROOT).as_posix(),
    purpose='Reproduce one complete forward reference without the programmed inverse; review approximate quarter-turn and placement repeatability.',
    ready_for_supervised_operator_run=True,operator_action_required=True,proof=PROOF.relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),candidate_sha256=sha(LOADED),hand_path_sha256=sha(HAND_PATH),
    current_readonly=source.relative_to(ROOT).as_posix(),current_off_difference_ticks=difference.tolist(),all16_OFF_error0_watchdog0_verified=True,latest_start_proof=cached.relative_to(ROOT).as_posix(),
    loaded_goals=2032,whole_goals=len(rows),nominal_motion_seconds=len(rows)*.075,loaded_motion_seconds=152.4,motion_limit_seconds=300,
    new_joint_goal_vectors=0,automatic_inverse=False,automatic_open=True,current_ma=450,sample_period_s=.075,goal_step_ticks=2,force_PID_and_health_criteria_unchanged=True,
    cube_cases=27,finite_empty_hand_cases=9,current_prefix_static_cases=201,current_prefix_dynamic_cases=9,software_tests_passed=15,
    modeled_forward=proof['strict_single_forward_cube_models'],physical_prior_forward_estimates_deg=[88.71186086891521,84.44548087670685],
    latest_actual_run='yaw_roundtrip_trial_20261030_011416_269699',latest_actual_forward_estimate_deg=84.44548087670685,latest_actual_inverse_estimate_deg=-42.311878838667035,
    physical_new_execution_not_yet_run=True,physical_exact90_verified=False,physical_repeatability_verified=False,inverse90_ready=False,all_directions90_verified=False,full_cube_solve_verified=False,no_live_SAC=True,
    expected_camera_present=True,latest_camera='contact_view_readonly_20261030_045210_021307/review.json',protected_sources_unchanged=True,assistant_motor_or_setting_writes=0,
    operator_steps=['FIRST terminal SPACE with EMPTY hand and Cube outside camera.','At PLACE put GREEN up and ORANGE wristward/ID41; remove hands; SECOND SPACE.','Grasp, forward turn and automatic OPEN. At REMOVE take Cube out; THIRD SPACE ends OFF.'])
 put(OUT/'handoff.json',report)
 archive=OUT/'previous_status';archive.mkdir(exist_ok=True)
 for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
  f=ROOT/name;old=json.loads(f.read_bytes());saved=archive/(f.stem+'_'+sha(f)[:12]+f.suffix)
  if not saved.exists():saved.write_bytes(f.read_bytes())
  old.update(status=report['status'],current_status=report['status'],timestamp=now,next_command=command,next_user_command=command,next_user_command_ready=True,next_user_action_required=True,
    next_user_action='Single forward reproduction: EMPTY first SPACE; PLACE GREEN up/ORANGE wristward, hands clear and second SPACE; automatic OPEN then remove Cube and third SPACE/OFF.',
    next_result_prefix='single_forward_trial_',ready_for_user_run=True,ready_for_supervised_operator_run=True,ready_scope='SAME_OBSERVED_APPROXIMATE_QUARTER_FORWARD_REPRODUCTION_NOT_INVERSE90',
    trajectory=LOADED.relative_to(ROOT).as_posix(),proof=PROOF.relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),placement_guide=guide.relative_to(ROOT).as_posix(),cube_orientation_at_place='GREEN up; ORANGE toward wrist ID41',
    latest_readonly=report['current_readonly'],latest_start_proof=report['latest_start_proof'],source_off_readonly=json.loads(REQUEST.read_bytes())['source'],latest_pose_difference_from_review_ticks=difference.tolist(),motors_off_verified=True,current_motor_state_verified=True,
    nominal_motion_seconds=report['nominal_motion_seconds'],active_motion_limit_s=300,current_ma=450,sample_period_s=.075,goal_step_ticks=2,current_force_and_health_criteria_unchanged=True,
    loaded_model_cases=27,new_whole_hand_dynamics_cases=9,current_empty_recovery_static_cases=201,current_empty_recovery_dynamic_cases=9,software_checks_passed=15,
    single_forward_handoff=(OUT/'handoff.json').relative_to(ROOT).as_posix(),single_forward_trial_ready=True,new_single_forward_trial_not_yet_run=True,
    roundtrip_trial_ready=False,inverse_90_ready=False,all_directions_90_verified=False,automatic_motion_started=False,assistant_motor_or_setting_writes=0)
  put(f,old)
 reference='yaw_roundtrip_trial_20261030_011416_269699';html=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>정회전 한 동작</title>
<style>body{{font:19px/1.7 "Malgun Gothic",sans-serif;max-width:1050px;margin:30px auto;padding:0 20px;color:#192a35}}h1{{font-size:29px}}.lead{{background:#e8f4ed;border-left:5px solid #37815a;padding:16px}}.photos{{display:grid;grid-template-columns:1fr 1fr;gap:18px}}figure{{margin:0}}img,video{{width:100%;border-radius:7px;background:#111}}figcaption{{padding:5px 0}}li{{padding:7px 0}}.note{{font-size:16px;color:#52616d}}code{{background:#eef1f5;padding:3px 7px}}@media(max-width:650px){{.photos{{grid-template-columns:1fr}}}}</style>
<h1>정회전 뒤 자동으로 손을 열고 종료</h1><p class="lead">손은 현재 모습 그대로 두세요. 첫 SPACE는 빈손입니다. PLACE가 나온 뒤 초록 면을 위로, 주황 면을 손목 ID41 쪽으로 놓습니다.</p>
<p>아래는 두 번째 실행에서 실제로 약84° 돌아갔던 배치와 회전 후 사진입니다. 이번 파일은 이 정회전 한 동작의 재현성을 확인합니다.</p>
<div class="photos"><figure><img src="../../{reference}/cube_before_grasp.jpg" alt="두 번째 실제 실행의 초기 큐브 배치"><figcaption>PLACE에서 참고할 배치</figcaption></figure><figure><img src="../../{reference}/cube_forward_turn_hold.jpg" alt="같은 실행에서 정회전한 큐브"><figcaption>이전 실행의 정회전 후</figcaption></figure></div>
<ol><li><code>leap_single_forward_trial.cmd</code>를 실행합니다. 큐브를 손과 카메라 밖에 두고 터미널에서 첫 <strong>SPACE</strong>를 누릅니다.</li><li><strong>PLACE CUBE</strong>가 나오면 사진처럼 자연스럽게 받쳐지는 위치에 놓습니다. 사람 손을 뺀 뒤 두 번째 <strong>SPACE</strong>를 누릅니다.</li><li>잡기와 정회전 후 손이 자동으로 열립니다. <strong>REMOVE CUBE</strong>에서 큐브를 뺀 다음 세 번째 <strong>SPACE</strong>로 OFF합니다.</li></ol>
<p>손 동작은 약3분15초이며 배치와 제거를 기다리는 시간은 별도입니다. 중단은 Q / Esc / Ctrl+C입니다.</p>
<p class="note">이전 같은 정회전의 영상 추정은 약84~89°입니다. 새 실행의 실제 각도와 반복 재현성은 저장 영상으로 확인합니다. 역회전90°는 후속 검증 과제입니다.</p>
<details><summary>이전 정회전 영상 발췌</summary><video controls preload="metadata" src="../../{reference}/offline_review/forward_only_excerpt_realtime.mp4"></video></details><p><a href="handoff.json">현재 실행 안내 기록</a> · <a href="../roundtrip_actual_review/index.html">왕복 실행 결과와 전체 영상</a></p></html>'''
 guide.write_text(html,encoding='utf-8')
 for f in (ROOT/'right_hand_rotation_20261030/yaw_roundtrip/operator_guide.html',ROOT/'right_hand_rotation_20261030/roundtrip_actual_review/index.html'):
  text=f.read_text(encoding='utf-8');banner='<p class="status"><a href="../single_forward/operator_guide.html">다음 실행: 정회전 한 동작과 자동 OPEN</a></p>'
  if banner not in text:text=text.replace('<h1>',banner+'<h1>',1)
  f.write_text(text,encoding='utf-8')
 marker='<!-- single_forward_ready_20261030 -->\n';paragraph=(f'2026-10-30 다음 실물 작업: `{command}` 한 번. 같은 실제 정회전2032지령과 자동OPEN을 사용하며 새 관절 목표 좌표0개다. '
    f'현재복귀 포함2589지령/명목194.175초, 기존300초와450mA/PID/75ms 및 건강 중단 기준을 사용한다. 초기 배치까지 검사한1ms 큐브27조건, 빈손9조건, 현재복귀기하201/동역학9조건, 거부회귀15개를 통과했다. '
    '새 실행은 빈손 첫SPACE, PLACE에서 초록 위/주황 손목ID41쪽으로 놓고 손을 뺀 뒤 두 번째SPACE, 정회전 뒤 자동OPEN, REMOVE에서 큐브 제거와 세 번째SPACE/OFF다. '
    '05:14 최신3회읽기는16개 모두OFF/오류0/최대0틱변화였고 검토자세4틱 이내 캐시를 확인했다. 웹캠에서 빈손과 정상 화면도 보았다. '
    '이전 실제 정회전은 약88.71도와84.45도 추정, 새실물 반복재현과 정확한90도는 미검증이다. 왕복 실행의 역회전은42.31도였으며 새파일에는 역회전이 없다. '
    '역회전90도는 미완료다. 전체경로 역재생과 접촉 보정 연구 후보는 추가 회전이 부족하거나 기하 조건에 실패해 실물로 제공하지 않았다. '
    '근거: right_hand_rotation_20261030/single_forward/handoff.json\n\n')
 for name in ('professor_progress_20261015.md','sac_current_status.md'):
  f=ROOT/name;old=f.read_text(encoding='utf-8')
  if not old.startswith(marker):f.write_text(marker+paragraph+old,encoding='utf-8')
 class Links(HTMLParser):
  def handle_starttag(self,tag,attrs):
   for k,v in attrs:
    if k in ('href','src') and v and not v.startswith(('https:','http:','#','data:')):assert (guide.parent/v).resolve().exists(),v
 Links().feed(html)
 print(json.dumps({k:v for k,v in report.items() if k not in ('modeled_forward','operator_steps')},ensure_ascii=False),flush=True)
if __name__=='__main__':main()
