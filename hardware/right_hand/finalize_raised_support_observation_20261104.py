"""Review completed user captures and retire the capture-only handoff; no devices."""
import hashlib
import html
import json
from datetime import datetime
from pathlib import Path

R = Path(__file__).resolve().parent
O = R / 'right_hand_rotation_20261104/raised_support_capture/actual_review'
O.mkdir(exist_ok=True)
rows = []
for folder in sorted(R.glob('raised_support_capture_20261104_*')):
    p = folder / 'result.json'
    if not p.exists():
        continue
    d = json.loads(p.read_bytes())
    samples = d.get('samples', [])
    start = next((i for i, s in enumerate(samples) if s.get('phase') == 'stage_5_loaded_grasp'), None)
    post = samples[start:] if start is not None else []
    target = d.get('placement_goal_ticks')
    fixed = bool(post and target and all(s.get('goal') == target for s in post))
    def vals(key):
        return [float(v) for s in samples for v in s.get(key, [])]
    voltage, current, tracking = vals('voltage_v'), vals('current_ma'), vals('tracking_rad')
    row = {
        'folder': folder.name, 'result_sha256': hashlib.sha256(p.read_bytes()).hexdigest(),
        'status': d.get('status'), 'error': d.get('error'),
        'all_torque_off_verified': d.get('all_torque_off_verified'),
        'settings_restored': d.get('settings_restored'),
        'cube_removal_confirmed': d.get('cube_removal_confirmed'),
        'cube_removal_confirmation_source': d.get('cube_removal_confirmation_source'),
        'fixed_goal_samples_after_second_space': len(post),
        'all_recorded_post_placement_goals_identical': fixed,
        'minimum_voltage_v': min(voltage) if voltage else None,
        'peak_absolute_current_ma': max(map(abs, current)) if current else None,
        'maximum_tracking_rad': max(tracking) if tracking else None,
        'camera_frames': d.get('camera_recording', {}).get('frame_count'),
        'camera_blank_frames': d.get('camera_recording', {}).get('image_signal_blank_frames'),
        'final_off_reference_valid': d.get('final_off_reference_valid'),
        'final_off_endpoint_drift_ticks': d.get('final_off_endpoint_drift_ticks'),
        'visual_observation': 'Cube remains visibly seated with green top and orange toward wrist in the selected before/after photos. The underside/support contacts are partially occluded.',
        'placement_registered_to_robot': False, 'physical_grasp_verified': False,
        'physical90_verified': False, 'repeat_capture_requested': False,
    }
    assert row['status'] == 'raised_support_static_observation_complete'
    assert fixed and row['all_torque_off_verified'] and row['settings_restored']
    rows.append(row)
assert len(rows) == 2
report = {
    'reviewed_at': datetime.now().astimezone().isoformat(),
    'status': 'static_support_capture_completed_offline_rotation_design_pending',
    'runs': rows, 'assistant_motor_writes': 0, 'new_motion_issued': False,
    'next_user_motion_command': None,
    'next_work': 'Shared discussion concerns Codex IDE/web continuity, not a new robot strategy. Continue placement/model comparison and offline turn redesign. Existing turn admission remains blocked.',
    'shared_discussion': {
        'url': 'https://chatgpt.com/share/6aea39e5-46b0-83ee-87f1-c7d20bccf7a9',
        'public_get_status': 200, 'conversation_body_available': False,
        'response_contains_only_share_metadata': True,
        'text_requested_from_user': False,
        'user_clarification_received': True,
        'topic': 'Continue VS Code Codex conversations and project work from web or other devices',
        'handoff_document': 'CODEX_WEB_HANDOFF_20261104.md',
    },
}
(O / 'observation_review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
cards = ''.join('<article><h2>' + html.escape(x['folder']) + '</h2><p>左: 배치 직후 / 오른쪽: 정지 관찰 후. 큐브 회전은 실행하지 않았습니다.</p><div class="pair">' + ''.join('<img src="../../../' + x['folder'] + '/' + name + '" alt="' + name + '">' for name in ('cube_on_support.jpg', 'cube_after_static_hold.jpg')) + '</div></article>' for x in rows)
page = '<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>새 받침 배치 촬영 결과</title><style>body{max-width:1200px;margin:24px auto;padding:16px;font:17px/1.7 system-ui}.pair{display:flex;gap:8px}.pair img{width:49%;object-fit:contain}article{margin:24px 0} @media(max-width:700px){.pair{display:block}.pair img{width:100%}}</style><h1>배치 촬영 2회 완료</h1><p>두 실행 모두 정상 종료했고 모터 OFF와 설정 복구를 확인했습니다. 선택된 전후 사진에서는 큐브가 비슷한 위치에 남아 있습니다. 바닥 접촉과 정확한 로봇 좌표는 확인되지 않았습니다. 파지력과 90도 회전은 검증하지 않았습니다.</p><p>추가 촬영은 요청하지 않습니다. 공유 대화의 본문은 현재 접근되지 않아 붙여넣은 내용을 확인한 뒤 다음 회전 설계에 반영합니다.</p>' + cards + '</html>'
(O / 'index.html').write_text(page.replace('左:', '왼쪽:').replace('공유 대화의 본문은 현재 접근되지 않아 붙여넣은 내용을 확인한 뒤 다음 회전 설계에 반영합니다.', '공유 대화는 Codex 웹과 VS Code의 작업 연속성에 관한 질문으로 확인됐습니다. 로봇 회전 설계의 새 지시는 아니며, 이어받기 요약은 CODEX_WEB_HANDOFF_20261104.md에 저장했습니다.'), encoding='utf-8')

paths = [
    'right_hand_rotation_20261104/raised_support_capture/handoff.json',
    'right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json',
    'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json',
    'right_hand_rotation_20261030/four_forward_review/handoff.json',
    'right_hand_rotation_20261030/natural_cradle_grasp/handoff.json',
]
for rel in paths:
    p = R / rel
    d = json.loads(p.read_bytes())
    d.update(status=report['status'], raised_support_capture_completed=True,
             latest_static_support_capture=rows[-1]['folder'],
             latest_static_support_review=str((O / 'observation_review.json').relative_to(R)).replace('\\', '/'),
             repeat_capture_requested=False, user_action_required=False,
             next_user_action_required=False, next_user_motion_command=None,
             rotation_trial_ready=False, loaded_motion_ready=False,
             shared_discussion_text_pending=False,
             codex_web_handoff_document='CODEX_WEB_HANDOFF_20261104.md')
    for k in ('user_command', 'next_motion_command', 'next_user_command', 'next_command'):
        if k in d:
            d[k] = None
    for k in ('ready', 'ready_for_user_run', 'ready_for_supervised_operator_run', 'next_user_command_ready'):
        if k in d:
            d[k] = False
    p.write_text(json.dumps(d, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(report, ensure_ascii=False, indent=2))
