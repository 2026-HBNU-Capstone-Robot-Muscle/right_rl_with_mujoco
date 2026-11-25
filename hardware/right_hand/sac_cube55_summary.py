"""Compact offline evaluation summary; no device dependencies."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    source=ROOT/'sac_forward_support_reference_v1_20k'/'supported_batch_eval_sac_cube55_resets_cube55mm.json'
    report=json.loads(source.read_text());rows=report['summaries']
    assert report['diagnostic_cube_side_mm']==55 and report['diagnostic_cube_physics']['side_m']==.055
    assert report['geometry_transfer_test'] and report['motor_commands_sent']==0
    out=dict(source=str(source),status=report['status'],cube_side_mm=55,successes=report['successes'],trials=report['trials'],
             required_hold_seconds=report['required_hold_steps']*report['control_dt'],
             final_tip_range_deg=[min(r['final_tip_deg'] for r in rows),max(r['final_tip_deg'] for r in rows)],
             final_target_error_max_deg=max(r['final_target_error_deg'] for r in rows),
             last_step_range=[min(r['steps'] for r in rows),max(r['steps'] for r in rows)],
             support_fault_trials=sum(r['support_fault'] for r in rows),all_finite=all(r['finite'] for r in rows),
             policy='Existing reference trajectory plus trained SAC residual; no retraining',physical_flip_verified=False)
    (ROOT/'sac_cube55_evaluation_summary.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out,indent=2))

if __name__=='__main__':main()
