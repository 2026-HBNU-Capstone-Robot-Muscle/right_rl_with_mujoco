"""Persist the user's face clarification without claiming camera registration."""
from pathlib import Path
import json


def main():
    root=Path(__file__).resolve().parents[2];folder=Path(__file__).resolve().parent/'next_cycle/handoff_refinement_20261110/user_ring_photo_reference_20261110'
    p=folder/'review.json';review=json.loads(p.read_text(encoding='utf-8'))
    review['user_contact_clarification']={'ring_contact_face':'white_front_face','user_words':'약지 흰색에 있잖아 그럼 앞면이겠지','provided_by_user_not_metric_camera_estimate':True,
        'middle_and_thumb_still_unspecified':True,'photo_front_face_to_model_axis_is_hypothesis':True}
    p.write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
    marker='## 사용자 접촉면 확인'
    text=(folder/'REVIEW.md').read_text(encoding='utf-8')
    if marker not in text:(folder/'REVIEW.md').write_text(text+'\n'+marker+'\n\n사용자가 약지는 사진의 흰색 앞면에 닿는다고 확인했다. 이 정보는 이후 약지 앞면–엄지 맞은편 모델 가설의 기준으로 쓴다. 중지·엄지의 정확한 자세와 사진–모델 좌표 등록은 아직 확인하지 않았다.\n',encoding='utf-8')
    print('user_confirmed_ring_on_white_front_face',flush=True)


if __name__=='__main__':main()
