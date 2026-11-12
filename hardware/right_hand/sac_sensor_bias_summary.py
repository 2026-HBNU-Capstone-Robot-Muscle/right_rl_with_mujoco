"""Summarize completed camera-position bias trials without loading devices."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    path=ROOT/'sac_forward_support_reference_v1_20k'/'supported_batch_eval_sac_cube55_resets_cube55mm_positionbias15mm.json'
    r=json.loads(path.read_text());groups={}
    for row in r['summaries']:
        key=str(row['camera_position_bias_mm']);g=groups.setdefault(key,dict(trials=0,successes=0,support_faults=0))
        g['trials']+=1;g['successes']+=int(row['success']);g['support_faults']+=int(row['support_fault'])
    report=dict(source=str(path),successes=r['successes'],trials=r['trials'],groups=groups,
                unsuccessful=[s for s in r['summaries'] if not s['success']],
                note='Constant single-axis position observation bias only; physical placement, orientation errors and vision dropout differ.',motor_commands_sent=0)
    (ROOT/'sac_sensor_bias_summary_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
