"""Review completed operator run and publish evidence; no device access."""
import datetime,hashlib,json
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parent
def main():
    folder=ROOT/'fresh_empty_grasp_20261027_032949_661470';source=folder/'result.json';r=json.loads(source.read_bytes());assert r['status']=='fresh_empty_grasp_complete' and r['all_torque_off_verified'] and r['settings_restored'];samples=r['samples'];phases={}
    for phase in sorted({s['phase'] for s in samples}):
        rows=[s for s in samples if s['phase']==phase];phases[phase]=dict(samples=len(rows),minimum_voltage_v=min(min(s['voltage_v']) for s in rows),peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in rows),maximum_tracking_rad=max(max(s['tracking_rad']) for s in rows))
    out=folder/'visual_review';out.mkdir(exist_ok=True);images=sorted(p for p in (folder/'camera').rglob('*') if p.suffix.lower() in ('.png','.jpg','.jpeg'));selected=[]
    if images:
        canvas=np.full((620,1440,3),255,np.uint8)
        for i,index in enumerate((0,len(images)//4,len(images)//2,3*len(images)//4,max(0,len(images)-25),len(images)-1)):
            p=images[index];frame=cv2.imdecode(np.fromfile(p,dtype=np.uint8),cv2.IMREAD_COLOR);x=(i%3)*480;y=(i//3)*310;canvas[y+30:y+300,x:x+480]=cv2.resize(frame,(480,270));cv2.putText(canvas,f'saved frame {index+1}/{len(images)}',(x+5,y+20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),1);selected.append(p.relative_to(ROOT).as_posix())
        cv2.imencode('.jpg',canvas)[1].tofile(out/'completed_run_contact_sheet.jpg')
    summary=dict(updated_local=datetime.datetime.now().astimezone().isoformat(timespec='seconds'),status='new_empty_grasp_posture_physically_executed_complete',source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),plan_version=r['plan_version'],duration_from_enable_s=max(s['t'] for s in samples),samples=len(samples),minimum_voltage_v=min(min(s['voltage_v']) for s in samples),peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),voltage_events=r.get('voltage_events',[]),phases=phases,all_torque_off_verified=True,settings_restored=True,camera_frames=r['camera_recording']['frame_count'],blank_camera_frames=r['camera_recording']['image_signal_blank_frames'],selected_saved_frames=selected,final_on_state=r.get('final_on_state'),final_off_snapshots=r.get('final_off_snapshots'),final_off_reference_valid=r.get('final_off_reference_valid'),physical_cube_grasp_verified_in_new_pose=False,physical_horizontal_90_verified=False,assistant_motor_commands_sent=0)
    (out/'completion_review.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    state=dict(status=summary['status'],updated_local=summary['updated_local'],latest_review=(out/'completion_review.json').relative_to(ROOT).as_posix(),latest_user_trial=summary['source'],new_empty_grasp_posture_executed_complete=True,new_empty_grasp_posture_duration_s=summary['duration_from_enable_s'],new_empty_grasp_posture_minimum_voltage_v=summary['minimum_voltage_v'],new_empty_grasp_posture_peak_current_ma=summary['peak_current_ma'],physical_trial_of_v7_completed=True,voltage_trip_absent_in_this_one_empty_run=not summary['voltage_events'],physical_voltage_improvement_verified_in_one_empty_run=True,previous_voltage_failure_not_claimed_permanently_fixed=True,physical_cube_grasp_verified_in_new_pose=False,physical_horizontal_90_verified=False,horizontal_quarter_turn_verified=False,physical_vertical_successes_total=2,next_action='Use reached posture and saved OFF readings to prepare loaded grasp observation; then develop horizontal90 and combine with the proven vertical flip.',next_user_action=None,next_user_command=None,next_user_command_ready=False,next_motion_command_ready=False,ready_for_operator_empty_trial=False,operator_empty_execution_ready=False,ready_for_real_yaw=False,horizontal_loaded_execution_ready=False,pending_user_input=None,assistant_motor_commands_sent=0,motor_writes_sent_this_turn=0,camera_opened_this_turn=False,serial_ports_opened_this_turn=False)
    for path in (ROOT/'right_hand_rotation_20261026/horizontal_next_step_20261026.json',ROOT/'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json',ROOT/'right_hand_rotation_20261026/fresh_empty_start_20261026_235258/progress_20261027.json'):
        old=json.loads(path.read_bytes());old.update(state);path.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block=f'''## CURRENT 2026-10-27: 새 빈손 준비 자세 실물 이동 완료

- `fresh_empty_grasp_20261027_032949_661470/result.json`에서 v7의 새 준비 자세 이동이 끝까지 완료됐다. 활성화 후 약 {summary['duration_from_enable_s']:.2f}초, 건강 읽기 {len(samples):,}개, 최소 전압 {summary['minimum_voltage_v']:.1f}V, 최대 전류 {summary['peak_current_ma']}mA, 최대 추종 오차 {summary['maximum_tracking_rad']:.5f}rad다. 전압 중단 이벤트는 없었다. 토크 OFF와 설정 복구도 읽기로 확인됐다. 영상은 {summary['camera_frames']:,}장, 검은 프레임 0장이다. 전원 경로가 영구적으로 고쳐졌다는 뜻은 아니다.
- 이 실행은 빈손의 복귀와 새 파지 자세 확인이다. 새 자세에서 실제 큐브를 쥐거나 수평 회전한 검증으로 격상하지 않는다. 수직 뒤집기 실물 성공 총 2회는 별도로 유지한다. 다음 의미 있는 검증은 새 자세에서 큐브 지지/파지이며 같은 빈손 실행을 사용자에게 반복 요청하지 않는다.
- 현재 목표는 오른손으로 큐브 전체를 원하는 방향으로 돌리는 것이다. 남은 흐름은 새 자세의 실물 큐브 파지, 놓치지 않는 수평90도, 성공한 수직 동작과 연결해 원하는 면으로 바꾸기, 시작부터 끝까지 개입 없는 시연 영상이다. 실제 프로그램은 검토한 관절 경로를 보내는 방식이고 카메라는 기록/검증에 쓰인다. 실시간 카메라 기반 SAC 제어/온라인 학습 완료가 아니다. 최종 큐브 풀이는 별도로 왼손 레이어 회전 및 풀이 순서 연결이 필요하다.
- 준비할 다음 소프트웨어는 저장된 실제 ON/OFF 각도에서 열림/배치/동일 파지 접근을 연결하는 제한된 실물 큐브 파지 확인이다. 아직 다음 실행 명령을 발급하지 않는다. 현재 사용자에게 새 측정/촬영/로그 복사를 요청하지 않는다.
- 근거: `{(out/'completion_review.json').relative_to(ROOT).as_posix()}`와 저장 영상 모음. 이번 작업은 파일 읽기/요약만 했고 실제 장치 열기/모터 쓰기는 0회다.

<!-- END EMPTY POSTURE COMPLETE 20261027 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END EMPTY POSTURE COMPLETE 20261027 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip()
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(block+old,encoding='utf-8')
    assert hashlib.sha256((ROOT/'exhibition_20261023/족자_붙여넣기.txt').read_bytes()).hexdigest()=='cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0'
    print(json.dumps({k:v for k,v in summary.items() if k not in ('final_on_state','final_off_snapshots','selected_saved_frames')},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
