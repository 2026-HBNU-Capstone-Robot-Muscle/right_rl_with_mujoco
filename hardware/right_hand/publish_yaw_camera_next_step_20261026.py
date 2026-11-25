"""Preserve the two flips and publish the next CAMERA ONLY operator action."""
import ast,datetime as dt,hashlib,json
import xml.etree.ElementTree as ET
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path,obj):path.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def marker_svg(ident,x,y,size):
    bits=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),ident,60)
    cells=['<rect x="%g" y="%g" width="%g" height="%g" fill="#fff"/>'%(x-6,y-6,size+12,size+12)]
    for r in range(6):
        for c in range(6):
            if bits[r*10+5,c*10+5]<128:cells.append('<rect x="%g" y="%g" width="%g" height="%g" fill="#080b12"/>'%(x+c*size/6,y+r*size/6,size/6+.05,size/6+.05))
    return ''.join(cells)
def main():
    protected={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        ROOT/'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0'}
    assert all(sha(p)==expected for p,expected in protected.items())
    script=ROOT/'cube_visibility_capture_20261026.py';tree=ast.parse(script.read_text(encoding='utf-8'));imports=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.Import):imports.extend(v.name for v in node.names)
        elif isinstance(node,ast.ImportFrom):imports.append(node.module or '')
    assert not any(name.startswith(('dynamixel_sdk','serial','leap_book','loaded_thumb')) for name in imports)
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    reports=[p for p in OUT.rglob('*.json') if p.name in ('review.json','seating_review.json','pinch_approach_replay.json','original_reference_after_flip_review.json','same_run_pair_search.json') and p.parent.name!='saved_marker_registration']
    index=[]
    for p in reports:
        r=json.loads(p.read_bytes());cases=r.get('cases',[])
        index.append(dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p),cases=len(cases),
            accepted_cases=sum(bool(v.get('conditional_candidate_accepted') or v.get('promising_prefix') or v.get('promising_regrasp')) for v in cases),hardware_execution_allowed=False))
    pair=json.loads((OUT/'saved_marker_registration/same_run_pair_search.json').read_bytes())
    action=dict(command='leap_cube_visibility_check.cmd',camera_only=True,motor_power='OFF',motor_port_opened=False,
        cube='GREEN UP on table beside wrist ID41, outside robot hand; aim camera down from above at green centre ID5',
        visible_reference=41,marker40_required=False,keys='One SPACE in camera or terminal',
        recording_advisory_not_a_marker_gate=True,profiles='original, exposure -5, -6, -7 when supported',
        original_camera_settings_restoration_attempted_on_exit=True,restoration_success_checked_from_readbacks=True,physical_yaw_command_ready=False,
        result='leap_cube_visibility_result.json',do_not_repeat_until_saved_file_review=True)
    review=dict(timestamp=stamp,status='two_physical_vertical_successes_horizontal_candidate_not_ready_camera_visibility_action_ready',
        physical_forward_flip_successes=2,horizontal_quarter_turn_verified=False,arbitrary_direction_rotation_verified=False,
        simulation_reports=index,current_saved_images_checked=pair['images_checked'],same_run_hand_anchor_pairs=len(pair['pairs']),
        saved_pair_search_sampling_stride=6,
        representative_image_detection=dict(before=['ORANGE centre ID3','wrist ID41'],after=['wrist ID41'],green_and_red_top_markers_not_decoded_in_these_representatives=True),
        marker41_and_marker40_frame_consistency_not_certified=True,
        raw_sources_unchanged=True,protected_hashes={p.relative_to(ROOT).as_posix():sha(p) for p in protected},
        latest_readonly='repeat_off_readonly_20261026_183436_367005/result.json',latest_readonly_motor_writes=0,
        camera_program_direct_imports=sorted(set(imports)),camera_program_sha256=sha(script),camera_command_sha256=sha(ROOT/'leap_cube_visibility_check.cmd'),
        camera_program_default_mode_checked_without_devices=True,operator_camera_capture_not_yet_performed=True,
        next_user_action=action,assistant_motor_commands_sent=0,hardware_execution_allowed=False)
    write(OUT/'horizontal_next_step_20261026.json',review)
    svg='''<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="620" viewBox="0 0 1000 620">
<style>text{font-family:'Malgun Gothic','Noto Sans CJK KR',sans-serif;fill:#192438}.title{font-size:29px;font-weight:700}.label{font-size:22px}.small{font-size:19px}</style>
<rect width="1000" height="620" rx="20" fill="#f3f6fb"/>
<text x="42" y="53" class="title">카메라 전용 촬영 배치</text>
<text x="42" y="87" class="small">모터 전원 OFF   /   큐브는 책상 위   /   SPACE 한 번</text>
<rect x="45" y="113" width="910" height="365" rx="18" fill="white" stroke="#a6b6d0" stroke-dasharray="9 7"/>
<text x="70" y="147" class="small">카메라 화면에 함께 보이게</text>
<path d="M90 465 H900" stroke="#8c98aa" stroke-width="5"/>
<rect x="143" y="275" width="260" height="190" rx="18" fill="#222b39"/>
<rect x="155" y="192" width="40" height="132" rx="15" fill="#37465b"/>
<rect x="218" y="171" width="40" height="153" rx="15" fill="#37465b"/>
<rect x="281" y="187" width="40" height="137" rx="15" fill="#37465b"/>
<text x="173" y="340" class="label" fill="white" style="fill:white">로봇 본체</text>
'''+marker_svg(41,315,378,58)+'''
<text x="163" y="494" class="label">손목의 ID41도 보이게</text>
<polygon points="580,250 648,330 648,461 580,381" fill="#dce5ee" stroke="#a9b5c4" stroke-width="3"/>
<polygon points="648,330 819,289 819,420 648,461" fill="#f28b3d" stroke="#a9b5c4" stroke-width="3"/>
<g transform="matrix(0.75 -0.18 0.30 0.35 580 250)">
<rect x="0" y="0" width="228" height="228" rx="12" fill="#fff" stroke="#b8c3ce" stroke-width="3"/>
'''
    for i in range(3):
        for j in range(3):svg+=f'<rect x="{8+j*74}" y="{8+i*74}" width="64" height="64" rx="9" fill="#28b87c"/>'
    svg+=marker_svg(5,86,86,56)+'''</g>
<text x="542" y="175" class="label">초록 면은 위로, 카메라는 위쪽에서</text>
<text x="607" y="494" class="label">중앙 마커 ID5가 크게</text>
<path d="M560 401 H453" stroke="#3b6ad5" stroke-width="4" fill="none"/>
<text x="439" y="382" class="small">손 밖</text>
<text x="60" y="538" class="label">ID40은 필요 없어. 시야를 맞춘 뒤 SPACE 한 번 누르면 자동 저장돼.</text>
<text x="60" y="572" class="small">ID5 크기 40px 이상을 목표로, 위쪽에서 가까이 비춰줘. Q는 종료.</text>
<text x="60" y="597" class="small">마커가 약하게 읽혀도 촬영은 저장되니, 한 번만 실행한 뒤 “했어”라고 알려줘.</text>
</svg>'''
    ET.fromstring(svg)
    (OUT/'cube_camera_visibility_layout.svg').write_text(svg,encoding='utf-8')
    block='''## CURRENT 2026-10-26: 수직 뒤집기 2회 확인, 다음은 큐브 시야 확보

- 두 번째 실물 영상에서도 초록 위/주황 손목 방향에서 빨강 위/초록 손목 방향으로 바뀌고 유지됐다. 완주, 큐브 제거, 토크 OFF 및 설정 복원을 확인했으며 두 번째 실시간 영상도 보존했다. 성공 두 번 사이에 전압으로 중단한 한 번이 있으므로 연속 성공이나 통계적 반복성으로 표현하지 않는다.
- 수평 회전은 기존 시퀀스 재생, 엄지 쓸기, 손가락 접촉점, 재파지 순서, 받침 회피 및 중지 뿌리 내림 시점을 비교했다. 손과 받침이 겹치지 않는 경로도 찾았으나 자유 큐브의 수평 90도와 유지 조건까지 통과한 실물 실행 후보는 아직 없다. 접촉이 크게 겹친 계산을 회전 성공으로 채택하지 않았다. 기존 수직 성공 경로, 전압/전류/추종 기준과 원본 범위 및 전시 문구는 보존했다.
- 실제 실행 영상 3,128장 중 522장을 간격 6으로 검사해 같은 실행의 ID40/41 동시 관측 43장을 확보했다. 추가 ID40 사진 요청은 필요 없다. 다만 전후 대표 사진의 작은 큐브 윗면 마커는 기존 검출기에서 읽히지 않아, 새 수평 동작의 실제 큐브 위치를 안정적으로 맞출 수 없다. 모터 동작을 반복하기 전에 카메라 전용으로 큐브 중앙 마커의 선명도와 노출을 한 번 확보한다.
- 사용자 다음 작업은 `leap_cube_visibility_check.cmd`다. 모터 5V 전원을 끄고, 큐브를 손 밖 책상에서 손목 ID41 옆에 초록 면이 위로 오게 둔다. 카메라를 더 위쪽에서 아래로 비춰 초록 중앙 ID5와 ID41이 같은 화면에 보이게 하고 카메라 또는 터미널 SPACE 한 번으로 자동 촬영한다. 책상 위 초록 면은 위 방향 참고값으로만 쓰며 책상이 수평이라는 가정은 별도로 기록한다. ID40 요구나 마커 부족에 따른 촬영 거부는 없다. 약한 자료도 저장되므로 반복 실행하지 않고 저장 결과를 검토한다. 원래 카메라 설정은 종료 때 복원을 시도하고 읽기값을 기록한다. 모터 포트와 모터 API는 사용하지 않는다.
- `right_hand_rotation_20261026/cube_camera_visibility_layout.svg`에 배치 그림을 저장했다. 새 프로그램의 기본 검사 모드는 장치를 열지 않고 통과했으며 실제 촬영은 아직 실행하지 않았다. 18:34 별도 모터 읽기는 3회 최대 1틱 편차, 토크 OFF였고 명령/설정 쓰기는 0회였다. 상세 연구 결과와 다음 작업은 `right_hand_rotation_20261026/horizontal_next_step_20261026.json`에 정리했다.

<!-- END HORIZONTAL CAMERA NEXT STEP 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig');marker='<!-- END HORIZONTAL CAMERA NEXT STEP 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1);p.write_text(block+old,encoding='utf-8')
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        p=ROOT/name;r=json.loads(p.read_bytes());r.update(timestamp=stamp,next_user_command=action['command'],next_user_command_ready=True,
            next_user_command_photo_only=True,next_user_command_camera_only=True,next_user_command_empty_only=False,
            rotation_in_next_command=False,next_motion_command_ready=False,horizontal_loaded_execution_ready=False,
            next_user_action=action,next_action='Camera-only visibility capture once; cube on table beside wrist ID41, green ID5 facing camera.',
            horizontal_next_step_review='right_hand_rotation_20261026/horizontal_next_step_20261026.json')
        write(p,r)
    assert all(sha(p)==expected for p,expected in protected.items())
    print(json.dumps(dict(successful_physical_flips=2,next_command=action['command'],camera_only=True,protected_files_unchanged=True,simulation_reports=len(index))),flush=True)
if __name__=='__main__':main()
