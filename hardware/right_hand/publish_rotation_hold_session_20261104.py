"""Publish the research review locally; never creates a motion admission."""
import json
from pathlib import Path
from datetime import datetime
import cv2 as cv

R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261104/rotation_hold_session'
def main():
    summary=json.loads((O/'session_summary.json').read_bytes());checks=json.loads((O/'verification.json').read_bytes());assert checks['passed']
    video=O/'views/body_hold_suffix.mp4';cap=cv.VideoCapture(str(video));opened=cap.isOpened();count=int(cap.get(cv.CAP_PROP_FRAME_COUNT));fps=float(cap.get(cv.CAP_PROP_FPS));ok,frame=cap.read();cap.release();assert opened and ok and count>50 and fps>0 and frame.mean()>1
    review={'opened':opened,'frames':count,'fps':fps,'duration_s':count/fps,'first_frame_nonblank':bool(ok and frame.mean()>1),'physical_video':False}
    (O/'video_verification.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    relative=O.relative_to(R).as_posix();handoff=dict(status=summary['status'],ready=False,user_action_required=False,next_user_motion_command=None,operator_guide=relative+'/index.html',review=relative+'/session_summary.json',hardware_execution_allowed=False,physical90_verified=False,physical_hold_verified=False,next_work=summary['next_work'],updated_at=datetime.now().astimezone().isoformat())
    (O/'handoff.json').write_text(json.dumps(handoff,indent=2)+'\n',encoding='utf-8')
    paths=['right_hand_rotation_20261104/resume_review.json','right_hand_rotation_20261104/raised_support_capture/handoff.json','right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261030/four_forward_review/handoff.json','right_hand_rotation_20261030/natural_cradle_grasp/handoff.json']
    for rel in paths:
        p=R/rel;d=json.loads(p.read_bytes());d.update(status=summary['status'],latest_body_hold_review=relative+'/session_summary.json',latest_body_hold_visual_review=relative+'/index.html',body_hold_physical_verified=False,body_hold_rotation90_verified=False,new_body_hold_operator_ready=False,repeat_capture_requested=False,next_user_action_required=False,user_action_required=False)
        for key in ('user_command','next_motion_command','next_user_command','next_command','next_user_motion_command'):
            if key in d:d[key]=None
        for key in ('ready','ready_for_user_run','ready_for_supervised_operator_run','next_user_command_ready','rotation_trial_ready','loaded_motion_ready'):
            if key in d:d[key]=False
        p.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    marker='<!-- body_hold_session_20261104 -->'
    note='''2026-11-04 약35분 오프라인 작업: 기존 약66.6도 회전 상태에서 중지/엄지 접촉을 낮추는 몸통 파지 후보를 구성했다. 자연 배치부터1535개 목표를 연속 재생한9조건 중8조건은 기존 한계와 지지를 유지했고1조건은 초기 배치에서 이탈했다. 통과 조건의 최종 방향은62.07~64.74도이며90도 성공이 아니다. 마지막 고정 관찰 구간의 최대 방향 변화1.23도/중심 이동0.914mm였다. 임의의±0.002Nm 강체 비틀림 진단은4종류 각9조건에서 유지 기준을 통과했으나 실제 왼손 힘이나 독립 레이어 동역학을 검증한 것은 아니다.

손끝만의 윗층90도 회전 공간은 새 후보에서 확보했지만, 손 전체56개 충돌 형상을 검사하면 중지의 중간/끝 하우징 쪽 간섭이 남았다. 검지 세 번째 접촉 경로7순서 중5순서는 기존 한계를 지켰고, 마디부터 굽히는2순서는 추종 한도에서 거부했다. 추가 받침128/144/152/168/176틱 가설 비교도90도를 달성하지 못했다. 새 실물 실행 명령과 승인 파일은 만들지 않았고 기존 정지 배치 촬영은 반복 요청하지 않는다. 장치 접근/모터 쓰기0회이며 원본 시퀀스와 보호 파일6개 해시를 확인했다. 시뮬레이션 영상과 수치 검토: right_hand_rotation_20261104/rotation_hold_session/index.html
'''
    p=R/'professor_progress_20261015.md';old=p.read_text(encoding='utf-8')
    if marker not in old:p.write_text(marker+'\n'+note+'\n'+old,encoding='utf-8')
    p=R/'CODEX_WEB_HANDOFF_20261104.md';old=p.read_text(encoding='utf-8');tag='## 2026-11-04 회전 후 몸통 파지 추가 연구'
    if tag not in old:
        old+='\n'+tag+'\n\n최신 검토는 `'+relative+'/index.html`과 `session_summary.json`에 있다. 새 몸통 파지 경로는 연속 재생9조건 중8조건을 통과했으나1조건 초기 배치 실패, 약62~65도에 그친 회전, 중지 몸통의 윗층 회전 공간 간섭이 남는다. 손끝만 비우는 것과 손 전체가 비키는 것은 다르다. 이 연구를 실물90도/파지력 성공으로 세지 않으며 새로운 실행 명령은 없다. 원본 동작과 중단 기준은 유지한다.\n'
        p.write_text(old,encoding='utf-8')
    print(json.dumps(dict(status=handoff['status'],visual_review=handoff['operator_guide'],video=review,verification_checks=len(checks['checks']),devices_opened=False),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
