"""Exercise the real OFF-only process wrapper, then independently verify proof."""
import json,time
from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,ThumbRootEmptyPlan
from thumb_clear_root_empty_gate_20261026 import run_screen,validate_proof
def main():
    start=time.monotonic()
    run_screen(FIXTURE/'request.json',FIXTURE/'screen.json',cancel=lambda:None)
    proof=validate_proof(ThumbRootEmptyPlan(FIXTURE/'request.json'),FIXTURE/'screen.json')
    result=dict(wrapper_completed=True,wall_seconds=time.monotonic()-start,
                static_cases=len(proof['static_screens']),response_cases=len(proof['dynamic_sensitivity']),
                hardware_commands_sent=0,devices_opened=False)
    (FIXTURE/'off_process_review.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)
if __name__=='__main__':main()
