"""Camera-only check of the plain preview; motor construction prohibited."""
import json,time
from unittest.mock import patch
from natural_cradle_grasp_plan_20261030 import OUT
from leap_natural_cradle_grasp_20261030 import NaturalCamera
def main():
 folder=OUT/'camera_probe';folder.mkdir(exist_ok=True);r=dict(phase='preview',recording_requested=True,camera_instruction='CAMERA ONLY CHECK. NO MOTION.');camera=None
 try:
  with patch('dynamixel_sdk.PortHandler',side_effect=RuntimeError('Motor access forbidden')) as port:
   camera=NaturalCamera(folder/'raw',r);began=time.monotonic()
   while time.monotonic()-began<3:camera.check();time.sleep(.04)
   with camera.lock:packet=camera.latest
   assert packet is not None and time.monotonic()-packet[0]<.3 and packet[1].shape==(720,1280,3);r.update(status='plain_camera_only_checked',motor_port_constructions=port.call_count,raw_shape=list(packet[1].shape),yellow_overlay_used=False)
 except (Exception,KeyboardInterrupt) as exc:r.update(status='stopped',error=str(exc))
 finally:
  if camera is not None:r['camera_recording']=camera.finish()
  (folder/'result.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
 print(json.dumps({k:r.get(k) for k in ('status','error','raw_shape','motor_port_constructions','yellow_overlay_used')}))
if __name__=='__main__':main()
