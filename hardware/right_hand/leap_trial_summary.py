"""Read saved grasp results; never imports camera or motor code."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parent
RESULT=ROOT/'leap_cube_grasp_trial_result.json'
MARKER=ROOT/'.leap_cube_grasp_summary_start.json'
SUMMARY=ROOT/'leap_cube_grasp_summary.json'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def begin(result=RESULT,marker=MARKER):
    marker.write_text(json.dumps({'previous_result_sha256':digest(result)}),encoding='utf-8')


def read_result(result=RESULT,marker=None):
    data=result.read_bytes();sha=hashlib.sha256(data).hexdigest()
    if marker is not None:
        previous=json.loads(marker.read_bytes())['previous_result_sha256']
        if sha==previous:
            raise RuntimeError('이번 실행의 새 결과가 없습니다. 이전 성공 기록은 표시하지 않습니다.')
    r=json.loads(data);rows=r.get('samples',[])
    completed=r.get('status')=='cube_grasp_approach_trial_complete'
    return dict(source=str(result),source_sha256=sha,timestamp=r.get('timestamp'),
        status=r.get('status'),approach_sequence_completed=completed,
        error=r.get('error'),failed_phase=r.get('failed_phase'),
        all_torque_off_verified=r.get('all_torque_off_verified') is True,
        settings_restored=r.get('settings_restored') is True,
        setting_or_goal_writes=len(r.get('write_events',[])),
        min_voltage_v=min((min(s['voltage_v']) for s in rows),default=None),
        peak_current_ma=max((abs(x) for s in rows for x in s['current_ma']),default=None),
        max_tracking_rad=max((max(s['tracking_rad']) for s in rows),default=None),
        camera_status=r.get('camera_recording',{}).get('status'),
        cube_removal_confirmed=r.get('cube_removal_confirmed') is True,
        physical_grasp_verified=r.get('physical_grasp_verified') is True,
        physical_rotation_verified=r.get('physical_flip_verified') is True,
        report_workflow='실행이 끝나면 했어라고만 알리면 조수가 저장된 결과와 사진을 확인합니다.')


def print_summary(s):
    print('\n결과 요약: '+('파지 접근 완료' if s['approach_sequence_completed'] else '중단: '+str(s['error'] or s['status'])))
    restored=('확인' if s['settings_restored'] else '설정 변경 없음' if not s['setting_or_goal_writes'] else '미확인')
    print('토크 OFF: '+('확인' if s['all_torque_off_verified'] else '미확인')+' / 설정 복원: '+restored)
    if s['min_voltage_v'] is not None:
        print(f"최저 {s['min_voltage_v']:.1f}V / 최대 전류 {s['peak_current_ma']}mA / 최대 추적 오차 {s['max_tracking_rad']:.3f}rad")
    print('사진 기록: '+str(s['camera_status'] or '없음')+' / 큐브 회전 검증: '+('확인' if s['physical_rotation_verified'] else '아직'))
    print('긴 로그를 복사할 필요 없이, 실행 후 "했어"라고만 알려주세요.')


if __name__=='__main__':
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    parser=argparse.ArgumentParser();group=parser.add_mutually_exclusive_group()
    group.add_argument('--begin',action='store_true');group.add_argument('--finish',action='store_true')
    group.add_argument('--latest',action='store_true');args=parser.parse_args()
    try:
        if args.begin:begin()
        else:
            summary=read_result(marker=MARKER if args.finish else None)
            SUMMARY.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
            print_summary(summary)
    except (OSError,ValueError,KeyError,TypeError,RuntimeError) as exc:
        print('결과 요약을 확인할 수 없습니다: '+str(exc))
        print('이 경우에만 오류 마지막 몇 줄을 알려주세요.')
        raise SystemExit(1)
