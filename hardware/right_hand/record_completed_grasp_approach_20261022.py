"""Record the user's completed v6 approach and simpler reporting workflow."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from leap_trial_summary import ROOT,read_result

FOLDER=ROOT/'cube_grasp_trial_20261022_141534_666300'
BASE=ROOT/'prepared_cube_review_20261022'


def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':
    source=FOLDER/'result.json';r=json.loads(source.read_bytes());summary=read_result(source)
    if not (summary['approach_sequence_completed'] and summary['all_torque_off_verified']
            and summary['settings_restored'] and summary['camera_status']=='complete'):
        raise RuntimeError('Saved trial did not complete with OFF/restoration and recording')
    phase_counts=dict(Counter(s['phase'] for s in r['samples']))
    report=dict(summary,source=str(source.relative_to(ROOT)),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),plan_version=r['plan_version'],
        phase_sample_counts=phase_counts,
        motion_goal_writes=sum(w['address']==116 for w in r['write_events'])-2,
        camera_frames=r['camera_recording']['frame_count'],
        operator_release_confirmation=r.get('contact_release_operator_confirmed') is True,
        placement_confirmed=r.get('cube_placement_confirmed') is True,
        before_after_photos_reviewed=['cube_before_approach.jpg','cube_after_approach.jpg'],
        cube_visible_on_hand_in_final_photo=True,
        physical_stable_grasp_force_verified=False,
        human_cube_correction_during_motion_known=False,
        physical_rotation_verified=False,hardware_commands_sent_by_assistant=0)
    save(FOLDER/'completion_review.json',report)
    save(BASE/'completed_loaded_approach_20261022.json',report)
    workflow=dict(complete_log_copy_required=False,finish_notification='했어',
        assistant_reads='latest saved result JSON and camera photos after the user notifies completion',
        automatic_background_monitoring=False,
        short_summary_automatically_printed_by='leap_cube_grasp_trial.cmd',
        latest_compact_summary='leap_cube_grasp_summary.json',
        fallback='Only if no new readable result was saved, request the last error lines.',
        stale_success_rejected=True,reporting_tests_passed=3)
    save(BASE/'trial_reporting_workflow.json',workflow)
    task='새 실행 불필요. 방금 동작 중 큐브를 사람 손으로 바로잡거나 받쳐줘야 했는지만 답변. 이후 작은 회전 시험을 준비하며 앞으로 완료 시 했어만 알리면 저장 결과와 사진을 직접 확인.'
    section=f'''## CURRENT 2026-10-22 14:15 v6 큐브 접근 완주 / 긴 로그 복사 불필요

- 사용자 실행 `cube_grasp_trial_20261022_141534_666300`의 파지 접근 경로{report['motion_goal_writes']}개 완료. 약지 분리·큐브 배치·제거 확인 및16개 토크 OFF/설정 복원, 카메라{report['camera_frames']}프레임 저장 확인. 최저{summary['min_voltage_v']:.1f}V, 최대 전류{summary['peak_current_ma']}mA, 최대 추적 오차{summary['max_tracking_rad']:.5f}rad.
- 전/후 자동사진을 직접 검토. 완료 사진에서도 큐브가 손 위, 손가락 사이에 남아 있다. 이것만으로 안정적인 접촉력·동작 중 사람 개입 없음·회전 성공을 인증하지 않음. 이번 실행은 회전 동작이 없는 접근 관찰이다.
- 앞으로 사용자는 긴 로그를 복사하지 않고 실행 후 **했어**라고만 알려도 된다. 그 알림을 받으면 조수가 자동 저장 JSON·사진을 읽는다. 상시 백그라운드 감시가 시작됐다는 뜻은 아니다.
- 같은 cmd의 종료 뒤5줄 결과 요약을 자동 출력하도록 추가. 주 제어 프로그램의 종료 코드를 보존. 요약은 카메라·모터 코드를 import하지 않는 별도 파일 리더이고 제어 경로·gain·한도·검사 코드는 변경하지 않았다. 실행 전 결과 해시를 기록하여 새 결과가 없으면 이전 성공을 이번 성공으로 표시하지 않는다. 새 파일 미저장·새 중단·실제 완료 자료의3개 보고 테스트 통과.
- 현재 필요한 답변: 방금 동작 중 큐브를 사람 손으로 바로잡거나 받쳐줘야 했는지(재실행/새 촬영 불필요). 다음은 작은 회전 시험 준비. 같은 접근 시험을 무조건 반복하지 않는다.
- 근거: 실행 폴더 `completion_review.json`, `prepared_cube_review_20261022/completed_loaded_approach_20261022.json`, `trial_reporting_workflow.json`, `leap_cube_grasp_summary.json`.

'''
    for filename in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/filename;text=path.read_text(encoding='utf-8-sig')
        if not text.startswith('## CURRENT 2026-10-22 14:15'):
            path.write_text(section+text,encoding='utf-8')
    for relative in ('prepared_cube_review_20261022/current_status.json',
                     'right_hand_start_20261019/objective_and_schedule.json'):
        path=ROOT/relative;d=json.loads(path.read_text(encoding='utf-8-sig'))
        d.update(status='loaded_approach_sequence_completed_rotation_unverified',
            latest_update='2026-10-22 14:15 completed v6',latest_completed_loaded_approach=report,
            next_user_task=task,complete_log_copy_required=False,physical_rotation_verified=False)
        if relative.endswith('objective_and_schedule.json'):
            d.update(current_user_task=task,latest_user_hardware_trial=report,
                current_blocker='Loaded approach completed. Final photo retains cube; human assistance and stable grasp force are not established. Rotation not yet tested.')
        save(path,d)
    print(json.dumps(dict(status=report['status'],motion_goals=report['motion_goal_writes'],
        minimum_voltage_v=summary['min_voltage_v'],peak_current_ma=summary['peak_current_ma'],
        max_tracking_rad=summary['max_tracking_rad'],camera_frames=report['camera_frames'],
        full_log_copy_required=False),ensure_ascii=False))
