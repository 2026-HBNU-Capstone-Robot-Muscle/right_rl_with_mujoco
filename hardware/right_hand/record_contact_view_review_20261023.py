"""Record latest contact-view result and persist the placement reference."""
import hashlib
import json
import shutil
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent
folder=ROOT/'cube_grasp_trial_20261023_041546_404551'
r=json.loads((folder/'result.json').read_bytes())
guide=ROOT/'right_hand_rotation_20261023/cube_placement_reference_annotated.png'
shutil.copyfile(Path('C:/Users/김준서/.codex/generated_images/01a0e65e-9c5f-7a81-b68b-a0b973139923/exec-cba87093-3260-4294-9d13-b071c9220089.png'),guide)
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),hand_id=41,required_cube_count=2)
detections=[]
for name in ('cube_before_approach.jpg','cube_after_approach.jpg'):
    image=cv.imdecode(np.frombuffer((folder/name).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    for label,im in [('whole',image),('top_roi_2x',cv.resize(image[100:250,380:760],None,fx=2,fy=2))]:
        corners,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
        found=[] if ids is None else ids.ravel().tolist()
        detections.append(dict(image=name,view=label,ids=found))
        print(name,label,found)
rows=[s for s in r['samples'] if s['phase']=='remove_cube']
delta=np.diff([s['t'] for s in rows]);i=int(np.argmax(delta))
frames=json.loads((folder/'camera/frames.json').read_bytes())['frames']
began=r['enabled_at']+rows[i]['t'];end=r['enabled_at']+rows[i+1]['t']
camera_during=[f for f in frames if began<f['camera_read_monotonic']<=end]
out=dict(status='grasp_complete_stopped_during_contact_view',source=folder.name,
    result_sha256=hashlib.sha256((folder/'result.json').read_bytes()).hexdigest(),
    failed_phase=r['failed_phase'],error=r['error'],
    all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
    completed_approach_samples=sum(s['phase']=='stage_3_loaded_approach' for s in r['samples']),
    camera_frames=r['camera_recording']['frame_count'],
    max_controller_sample_gap_s=float(delta[i]),camera_frames_during_gap=len(camera_during),
    last_snapshot_read_ms=rows[i+1]['read_ms'],
    next_snapshot_watchdog=rows[i+1]['watchdog'],
    image_detections=detections,
    gui_diagnosis='Camera imshow/waitKey runs on the motor/control thread. A GUI stall is a plausible cause; the exact blocked function was not instrumented.',
    physical_rotation_tested=False,policy_used=r['policy_used'],
    guide_file=str(guide.relative_to(ROOT)),guide_is_verified_successful_placement=False,
    current_control='fixed_waypoint_path_with_camera_recording_not_camera_conditioned_SAC',
    next_user_action='No repeat requested. Explain current implementation and show fixed-path placement reference.',
    hardware_commands_sent_by_agent=0)
(folder/'contact_view_review.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
text='''## CURRENT 2026-10-23 04:15 이후: 파지 촬영 저장 / 시각 SAC 연결은 미구현

- `cube_grasp_trial_20261023_041546_404551`: 기존 쥐기322목표까지 실행하고 사진 저장. `remove_cube` 촬영 대기 중16개 워치독255로중단. OFF·설정복원 확인. 영상1290개 모두정상. 제거SPACE확인은기록되지않았으므로 정상완주/제거확인으로표시하지않는다.
- 마지막 정상표본과오류표본 사이718ms 공백. 오류표본 읽기31ms이며같은공백에도카메라드라이버프레임은계속저장됐다. `empty_trial_camera.check()`는모터제어스레드에서imshow/waitKey를호출하므로GUI정체가유력한후보다. 실제함수구간계측이없어원인확정이나수정완료로기록하지않는다. 워치독시간을늘리지않는다.
- 사용자는촬영각도가맞는지모르겠다고했고,카메라로위치를보고SAC가회전을학습/수행하는것이아닌지질문했다. 현재사용자실행본은사전계산된고정경로이며카메라는영상기록/영상신호중단용이다. 카메라큐브자세가정책입력이나동작보정으로연결되지않았고,카메라를켜면자동학습이되는것도아니다. 최종목표는위치/자세관측과관절상태를받는학습정책및보정이며,배치오차를훈련조건에포함해야한다.
- 최신사진의앞면은초록(ID5)이고상면은과노출되어색을영상만으로단정하지않는다. 기존고정경로기준사진은초록위/주황앞이다. 색방향의차이만으로기하학적파지실패의원인으로단정하지않는다.
- 사용자요청대로기존실제기준사진에방향/공간안내를표시한`right_hand_rotation_20261023/cube_placement_reference_annotated.png`를만들었다. 회전성공이검증된정밀배치가아님을명시했다. 원본도보존하고치수/로봇/목표를바꾸지않았다. 추가촬영이나같은경로반복은요청하지않는다.
- 다음개발작업:GUI표시와모터상태읽기분리→큐브관측이연결되는실제입출력계약정리→새매핑/배치범위로학습환경·보상·초기상태검증. 기존잘못된매핑정책이나고정CEM경로를실물SAC성공이라고표현하지않는다.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if not old.startswith('## CURRENT 2026-10-23 04:15 이후:'):p.write_text(text+old,encoding='utf-8')
print(json.dumps({k:v for k,v in out.items() if k not in ('error','image_detections','gui_diagnosis')},ensure_ascii=False,indent=2))
