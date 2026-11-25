"""Preserve the user's grasp photo and distinguish it from the old encoder read."""
from pathlib import Path
import hashlib
import json
import shutil

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    source=Path('C:/Users/김준서/Downloads/KakaoTalk_20261110_223109802.jpg')
    angles=ROOT/'ring_reference_readonly_20261109_124719_945552/result.json'
    folder=OUT/'next_cycle/handoff_refinement_20261110/user_ring_photo_reference_20261110'
    if folder.exists():raise RuntimeError('Preserve previous photo review')
    record=json.loads(angles.read_text(encoding='utf-8'));motors=record['snapshots'][0]['motors'];lookup={r['id']:r for r in motors}
    folder.mkdir();photo=folder/'user_photo.jpg';shutil.copyfile(source,photo)
    assert sha(source)==sha(photo)
    report=dict(status='user_supplied_grasp_photo_qualitative_reference',photo_source=str(source),photo_file=photo.name,photo_sha256=sha(photo),
        recorded_encoder_source=angles.relative_to(ROOT).as_posix(),recorded_encoder_sha256=sha(angles),
        recorded_ring_tip_to_base=[dict(motor_id=mid,ticks=lookup[mid]['position_ticks'],model_coordinate_degrees=(lookup[mid]['position_ticks']-2048)*360/4096) for mid in (13,15,14,16)],
        photo_and_encoder_time_synchronization_confirmed=False,user_says_middle_and_thumb_posture_uncertain=True,
        visible_observations=['Red upper face and white front face of the cube are visible.',
            'Finger structures lie at the lower front of the cube; several contact surfaces are occluded.',
            'No obvious finger extends across the visible upper face in this view.'],
        unresolved=['Which visible structure belongs to each finger cannot be assigned confidently from this single view.',
            'Actual normal force, three independent loaded faces, and hidden support cannot be established from the photo.',
            'Complete swept clearance of the upper layer requires hidden-side geometry and is not certified by the picture.',
            'The current image has not been registered to the model hand frame or to a calibrated camera.',
            'The photograph is compatible with the intended lower grip but does not prove the four recorded angles are reproduced exactly.'],
        interpretation='Use as evidence of intended low finger placement and cube-in-hand arrangement; do not inherit assumed old cube coordinates.',
        encoder_values_are_reused_not_remeasured=True,limits_changed=False,new_motor_goals_created=False,devices_opened=0,hardware_execution_allowed=False)
    (folder/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (folder/'REVIEW.md').write_text('''# 사용자가 제공한 약지 파지 사진

사용자 설명: 중지와 엄지 자세는 확실하지 않지만, 저장한 약지 관절값이면 사진 같은 형태가 아니냐는 질문이다.

사진에는 큐브의 붉은 윗면과 흰 앞면이 보이고, 앞쪽 손가락 구조는 아래쪽에 놓여 있다. 보이는 윗면을 가로막는 손가락은 뚜렷하지 않다. 아랫부분의 여러 접촉면과 반대편은 가려져 있어 손가락별 정확한 접촉면과 지지력, 세 면의 독립된 하중 지지까지 판정하지 않는다.

약지 기록 ID13/15/14/16=3018/2961/2829/3053틱은 그대로 보존한다. 이 사진과 각도 기록이 같은 순간이라는 확인은 없으며, 사용자가 중지·엄지는 불확실하다고 설명했다. 따라서 기록된 전체16관절을 그린 모델을 사진의 정확한 재구성이라고 부르지 않는다.

이 사진을 하부 파지의 실제 모습과 큐브 배치를 맞추는 정성적 참고로 사용한다. 기존에 임의로 고정한 모델 큐브 배치에서 약지가 닿지 않았다는 결과로 이 실제 파지를 부정하지 않는다. 사진만으로 정확한 mm/각도 등록이나 전체 윗단 회전 공간을 인증하지 않는다. 원래 관절 제한과 원본 실행 파일은 변경하지 않았다.
''',encoding='utf-8')
    print(json.dumps(dict(folder=folder.relative_to(ROOT).as_posix(),photo_sha256=sha(photo),ring_ticks=[lookup[i]['position_ticks'] for i in (13,15,14,16)],synchronized=False,devices_opened=0)),flush=True)


if __name__=='__main__':main()
