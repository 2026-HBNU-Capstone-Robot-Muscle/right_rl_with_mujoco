import datetime,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/rigid_contact_review_20261025';OUT.mkdir(exist_ok=True)
r=dict(timestamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat(),
    question='Existing cube-contact index and middle fingertip material',user_answer='단단한 플라스틱',
    confirmed_contact_material='rigid plastic',coefficient_of_friction_measured=False,
    numeric_friction_cannot_be_inferred_from_material_alone=True,hardware_commands_sent=0)
(OUT/'material_confirmation.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(r,ensure_ascii=False))
