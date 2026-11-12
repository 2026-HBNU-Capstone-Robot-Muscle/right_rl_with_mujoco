"""OFF-only simulator subprocess and independent saved-proof validation."""
import json,time
from loaded_repeat_plan_20261026 import FIXTURE,LoadedRepeatPlan
from loaded_repeat_gate_20261026 import run_screen,validate_proof
def main():
    began=time.perf_counter();run_screen(FIXTURE/'request.json',FIXTURE/'screen.json',cancel=lambda:None)
    r=validate_proof(LoadedRepeatPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
    review=dict(accepted=True,wall_seconds=time.perf_counter()-began,static=len(r['static_screens']),normal=len(r['dynamic_sensitivity']),
        empty_camera_hold=len(r['camera_gap_hold_sensitivity']),hardware_commands_sent=0,devices_opened=False)
    (FIXTURE/'off_process_review.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8');print(json.dumps(review),flush=True)
if __name__=='__main__':main()
