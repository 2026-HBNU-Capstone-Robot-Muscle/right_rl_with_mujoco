"""Three-second GUI/raw-frame check. Motor port construction is forbidden."""
import json,time
from pathlib import Path
from unittest.mock import patch
import numpy as np,cv2 as cv
from placement_alignment_camera_20261030 import AlignmentCamera,REFERENCE,comparison_frame
from placement_alignment_plan_20261030 import OUT
R=Path(__file__).resolve().parent;O=OUT/'camera_probe';O.mkdir(exist_ok=True)
def main():
 report=dict(phase='place_cube',recording_requested=True,hardware_authorized=False,camera_instruction='PLACE CAMERA ONLY: YELLOW = TARGET. NO MOTOR ACCESS.');camera=None
 try:
  with patch('dynamixel_sdk.PortHandler',side_effect=RuntimeError('Motor access forbidden in camera-only probe')) as port:
   camera=AlignmentCamera(O/'raw',report);began=time.monotonic()
   while time.monotonic()-began<3:camera.check();time.sleep(.04)
   with camera.lock:packet=camera.latest
   if packet is None or time.monotonic()-packet[0]>.3:raise RuntimeError('No fresh raw camera frame')
   stamp,frame=packet;assert frame.shape==(720,1280,3);ref=cv.imdecode(np.frombuffer(REFERENCE.read_bytes(),np.uint8),cv.IMREAD_COLOR);display=comparison_frame(cv,frame,ref,'CAMERA ONLY CHECK - NO MOTOR ACCESS');ok,data=cv.imencode('.jpg',display);assert ok;(O/'comparison_preview.jpg').write_bytes(data.tobytes());report.update(status='camera_only_comparison_checked',raw_shape=list(frame.shape),display_shape=list(display.shape),motor_port_constructions=port.call_count)
 except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
 finally:
  if camera is not None:report['camera_recording']=camera.finish()
  (O/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
 print(json.dumps({k:report.get(k) for k in ('status','error','raw_shape','display_shape','motor_port_constructions')}))
if __name__=='__main__':main()
