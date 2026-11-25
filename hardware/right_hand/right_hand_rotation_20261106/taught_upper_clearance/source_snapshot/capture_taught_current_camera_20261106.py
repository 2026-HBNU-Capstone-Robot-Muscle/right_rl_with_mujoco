"""One unattended camera-only observation; robot port construction is blocked."""
import datetime as dt,json,time
from pathlib import Path
from unittest.mock import patch
from leap_natural_cradle_grasp_20261030 import NaturalCamera
R=Path(__file__).resolve().parent
def main():
    out=R/'right_hand_rotation_20261106/taught_upper_clearance'/('current_camera_'+dt.datetime.now().strftime('%H%M%S'));out.mkdir();r=dict(phase='preview',recording_requested=True,camera_instruction='Camera only. Do not touch hand. No motion.',paired_encoder_capture=False,hardware_execution_allowed=False);camera=None
    try:
        with patch('dynamixel_sdk.PortHandler',side_effect=RuntimeError('Motor access forbidden')) as port:
            camera=NaturalCamera(out/'raw',r);began=time.monotonic()
            while time.monotonic()-began<3:camera.check();time.sleep(.04)
            with camera.lock:packet=camera.latest
            assert packet is not None and time.monotonic()-packet[0]<.3
            (out/'latest.jpg').write_bytes(__import__('cv2').imencode('.jpg',packet[1])[1].tobytes());r.update(status='camera_only_photo_saved',motor_port_constructions=port.call_count,raw_shape=list(packet[1].shape))
    except (Exception,KeyboardInterrupt) as exc:r.update(status='stopped',error=str(exc))
    finally:
        if camera is not None:r['camera_recording']=camera.finish()
        (out/'result.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.relative_to(R).as_posix(),status=r.get('status'),error=r.get('error'),motor_port_constructions=r.get('motor_port_constructions'))))
if __name__=='__main__':main()
