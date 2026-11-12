"""Enumerate camera names only. No stream, preview, or motor API."""
import datetime as dt,json
from camera_stream import enumerate_names,EXPECTED_NAME
from gapped_inverse_plan_20261029 import OUT
names=enumerate_names()
r=dict(timestamp=dt.datetime.now().astimezone().isoformat(),names=names,
    expected_camera=EXPECTED_NAME,
    expected_camera_present=sum(n.strip().casefold()==EXPECTED_NAME.casefold() for n in names)==1,
    camera_stream_opened=False,frames_captured=0,motor_api_used=False)
(OUT/'current_camera_inventory.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(r,ensure_ascii=False))
