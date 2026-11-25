"""Run only the OFF native screen and independent validator; no devices."""
import json,time
from loaded_thumb_rotation_plan_20261026 import FIXTURE,LoadedThumbRotationPlan
from loaded_thumb_rotation_gate_20261026 import run_screen,validate_proof
def main():
    began=time.monotonic();run_screen(FIXTURE/'request.json',FIXTURE/'screen.json',cancel=lambda:None)
    r=validate_proof(LoadedThumbRotationPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
    record=dict(accepted=True,wall_seconds=time.monotonic()-began,static=len(r['static_screens']),normal=len(r['dynamic_sensitivity']),
        empty_camera_hold=len(r['camera_gap_hold_sensitivity']),hardware_commands_sent=0,devices_opened=False)
    (FIXTURE/'off_process_review.json').write_text(json.dumps(record,indent=2),encoding='utf-8');print(json.dumps(record),flush=True)
if __name__=='__main__':main()
