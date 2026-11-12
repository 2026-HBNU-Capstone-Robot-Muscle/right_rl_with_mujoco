"""Enumerate camera names only; no VideoCapture or motor API."""
import datetime as dt,json
from pathlib import Path
from camera_stream import enumerate_names,EXPECTED_NAME
ROOT=Path(__file__).resolve().parent
def main():
    names=enumerate_names()
    result=dict(timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),names=names,
        expected_camera=EXPECTED_NAME,expected_camera_present=sum(n.strip().casefold()==EXPECTED_NAME.casefold() for n in names)==1,
        frames_captured=0,camera_stream_opened=False,motor_api_used=False)
    (ROOT/'right_hand_rotation_20261029/pitch_then_slow_yaw_v1/camera_inventory.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))
if __name__=='__main__':main()
