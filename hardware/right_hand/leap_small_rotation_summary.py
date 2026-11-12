"""Saved-file summary only; no robot or camera imports."""
import argparse
import json
from pathlib import Path
import sys
from leap_trial_summary import begin,read_result

ROOT=Path(__file__).resolve().parent
RESULT=ROOT/'leap_cube_small_rotation_trial_result.json'
MARKER=ROOT/'.leap_small_rotation_summary_start.json'
SUMMARY=ROOT/'leap_small_rotation_summary.json'


def main():
    if hasattr(sys.stdout,'reconfigure'):sys.stdout.reconfigure(encoding='utf-8')
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group()
    g.add_argument('--begin',action='store_true');g.add_argument('--finish',action='store_true')
    g.add_argument('--latest',action='store_true');args=p.parse_args()
    try:
        if args.begin:begin(RESULT,MARKER);return 0
        s=read_result(RESULT,MARKER if args.finish else None)
        s['rotation_command_path_completed']=s['status']=='cube_small_rotation_path_complete'
        s['physical_rotation_verified']=False
        SUMMARY.write_text(json.dumps(s,ensure_ascii=False,indent=2),encoding='utf-8')
        print('\n결과: '+('작은 회전 명령 경로 완료' if s['rotation_command_path_completed'] else '중단: '+str(s['error'] or s['status'])))
        print('토크 OFF: '+('확인' if s['all_torque_off_verified'] else '미확인')+
              ' / 설정 복원: '+('확인' if s['settings_restored'] else '미확인'))
        if s['min_voltage_v'] is not None:
            print(f"최저 {s['min_voltage_v']:.1f}V / 최대 전류 {s['peak_current_ma']}mA / 최대 추적 오차 {s['max_tracking_rad']:.3f}rad")
        print('실제 큐브 회전 성공은 사진·영상 확인이 필요합니다.')
        print('긴 로그 대신 "했어"라고만 알려주세요. 큐브 앞면·모서리가 움직였는지만 같이 알려주면 됩니다.')
        return 0
    except (OSError,ValueError,KeyError,TypeError,RuntimeError) as exc:
        print('새 결과 요약을 확인할 수 없습니다: '+str(exc))
        print('이 경우에만 오류 마지막 몇 줄을 알려주세요.');return 1


if __name__=='__main__':raise SystemExit(main())
