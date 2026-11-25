"""Persist the user's confirmation; no hardware imports or actions."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
if __name__=='__main__':
    relative='cube_grasp_trial_20261022_141534_666300'
    fact=dict(source_trial=relative,source='user_confirmation_after_completed_v6',
        user_statement='그런건 없었는데 그냥 큐브쥐기만 한거아님?',
        human_cube_correction_during_motion_known=True,human_assistance_required=False,
        unassisted_cube_retention_during_approach_user_confirmed=True,
        approach_only=True,rotation_command_sent=False,physical_rotation_verified=False,
        next_objective='A small deliberate cube rotation from this reached grasp, followed by larger requested rotations.',
        current_repeated_grasp_or_new_photo_requested=False)
    (ROOT/'right_hand_rotation_20261022').mkdir(exist_ok=True)
    (ROOT/'right_hand_rotation_20261022/user_confirmed_start.json').write_text(
        json.dumps(fact,ensure_ascii=False,indent=2),encoding='utf-8')
    for path in (ROOT/relative/'completion_review.json',
                 ROOT/'prepared_cube_review_20261022/completed_loaded_approach_20261022.json'):
        d=json.loads(path.read_bytes());d.update({k:fact[k] for k in (
            'human_cube_correction_during_motion_known','human_assistance_required',
            'unassisted_cube_retention_during_approach_user_confirmed')})
        path.write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf-8')
    section='''## CURRENT 2026-10-22: 사용자 개입 없이 쥐기 완료 확인 / 다음은 회전

- 사용자는 접근 중 큐브를 받치거나 바로잡아야 했던 일이 없었다고 확인했다. 완료된 v6는 큐브를 감싸는 접근까지만 실행했고 회전 명령은 없었다. 이 단계가 목표 방향 회전의 성공이라는 뜻은 아니다.
- 다음 작업은 방금 도달한 관절 자세를 시작으로 작은 의도적 큐브 회전을 찾는 시뮬레이션. 기존 합성 큐브 위치 모델은 실제 사진의 엄지 접촉과 일치하지 않는 사례가 있어 접촉·유지 불확실성을 함께 평가한다. 손가락 각도 변화만으로 성공 판정하지 않고 큐브 자체의 방향 변화를 측정한다.
- 새 사진이나 같은 쥐기 시험 반복은 요청하지 않았다. 완료 알림은 계속 했어만으로 충분하다. 필요한 새 실물 작업은 별도 명령과 관찰 목표를 준비한 뒤 안내한다.
- 근거: `right_hand_rotation_20261022/user_confirmed_start.json`, 이번 실행 폴더의 `completion_review.json`.

'''
    for filename in ('professor_progress_20261015.md','sac_current_status.md'):
        p=ROOT/filename;text=p.read_text(encoding='utf-8-sig')
        if not text.startswith('## CURRENT 2026-10-22: 사용자 개입 없이'):
            p.write_text(section+text,encoding='utf-8')
    for filename in ('prepared_cube_review_20261022/current_status.json',
                     'right_hand_start_20261019/objective_and_schedule.json'):
        p=ROOT/filename;d=json.loads(p.read_bytes())
        d.update(status='unassisted_grasp_approach_confirmed_rotation_search',
                 human_assistance_required=False,user_confirmation=fact,
                 next_user_task='현재 새 실행/사진 불필요. 작은 회전 후보 시뮬레이션 진행.')
        if 'current_user_task' in d:d['current_user_task']=d['next_user_task']
        p.write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf-8')
    print('Unassisted approach confirmed; cube rotation remains the next objective.')
