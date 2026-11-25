"""Record the completed observed inverse and withdraw the previous trial cue."""
import datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;F=ROOT/'gapped_inverse_trial_20261029_184252_272166';O=F/'offline_review'
execution=json.loads((O/'physical_execution_review.json').read_bytes())
angles=json.loads((O/'side_marker_rotation_review.json').read_bytes())
assert angles['comparisons']['placement_to_turn']['palm_pairs']>=16
estimate=angles['comparisons']['placement_to_turn']['palm_relative_twist_p05_median_p95_deg'][1]
opened=angles['comparisons']['placement_to_open']['palm_relative_twist_p05_median_p95_deg'][1]
assert -80<estimate<-30 and -80<opened<-30
report=dict(execution,status='physical_inverse_motion_observed_angle_estimated',
    physical_angle_quantification_pending=False,physical_inverse_motion_observed=True,
    estimated_palm_relative_placed_to_hold_deg=estimate,
    estimated_palm_relative_placed_to_open_deg=opened,
    angle_source='side_marker_rotation_review.json',
    exact_robot_axis_90_verified=False,metric_accuracy_not_certified=True,
    frame_spread_not_absolute_accuracy=True,
    image_plane_and_palm_compensation_both_indicate_inverse_rotation=True,
    video_exports='video_exports.json')
(O/'physical_inverse_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json',
    'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    p=ROOT/rel;r=json.loads(p.read_bytes())
    r.update(status='physical_inverse55_observed_extension_model_in_progress',
        current_status='physical_inverse55_observed_extension_model_in_progress',timestamp=dt.datetime.now().astimezone().isoformat(),
        next_user_command=None,next_command=None,next_user_command_ready=False,next_user_action_required=False,
        next_user_action='Keep cube out; extension models are in progress. Do not repeat the previous trial.',
        ready_for_user_run=False,ready_for_supervised_operator_run=False,
        latest_actual_run=F.name,latest_actual_review=(O/'physical_inverse_review.json').relative_to(ROOT).as_posix(),
        last_actual_cube_rotation_review=(O/'physical_inverse_review.json').relative_to(ROOT).as_posix(),
        last_actual_cube_video=(O/'full_run_realtime.mp4').relative_to(ROOT).as_posix(),
        inverse_cube_physical_trial_not_yet_run=False,physical_inverse_motion_observed=True,
        physical_inverse_cube_rotation_verified=False,inverse_90_ready=False,
        estimated_inverse_hold_deg=estimate,estimated_inverse_after_open_deg=opened,
        cube_removal_seen_in_final_saved_frame=True,assistant_motor_or_setting_writes=0)
    p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
block=('2026-10-29 18:42 역회전 실물 완료. gapped_inverse_trial_20261029_184252_272166은 전체2703지령,역방향638목표,자동OPEN,제거,OFF와설정복구까지 완료했다. '
    '최저4.7V/최대111mA/최대0.07977rad,1898카메라프레임과검정0장을기록했다.3회읽기실패는모두복구됐으며구동중2회정상전체상태2회확인은97.3/87.6ms였다. '
    '처음2실행은카메라프리뷰에서중단돼모터쓰기를하지않았다.마지막프레임에서큐브제거도보여추가제거질문은필요없다. '
    '처음초록ID5미검출때문에초록만사용한측정은실패로보존했다.계속보이는흰면ID0와이후초록격자법선으로같은큐브축을추적하고,미검출손바닥ID40은기존해독후LK추적점으로오프라인본체운동만보정했다. '
    f'추정회전은배치→고정{estimate:.2f}도,배치→OPEN{opened:.2f}도다.초록위/흰면앞에서주황면앞으로역회전했고,사람손은선택사진에서받침에보인다. '
    '정밀로봇좌표각도/모든프레임무보조/90도성공은인증하지않는다.원속도전체248.4초,잡기/회전/OPEN107.5초영상을보존했다. '
    '현재다음동작명령은아직없고,기존구간뒤에목표를추가하는모델을검사중이다.반복실행하지않고큐브는밖에둔다. '
    '근거: gapped_inverse_trial_20261029_184252_272166/offline_review/physical_inverse_review.json\n\n')
marker='<!-- observed_inverse_20261029_184252 -->\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;text=p.read_text(encoding='utf-8-sig')
    if not text.startswith(marker):p.write_text(marker+block+text,encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('recovery_durations_ms',)},ensure_ascii=False))
