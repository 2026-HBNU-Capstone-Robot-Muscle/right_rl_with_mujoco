"""Inspect an actually completed whole-hand trial without running physics."""
from pathlib import Path
import json
OUT=Path(__file__).resolve().parent
PATH=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_whole_hand_v1_active_ccd50/review.json'


def main():
    report=json.loads(PATH.read_text(encoding='utf-8'));trace=report['trace']
    supported=[row for row in trace if row['whole_hand_support']['supported_sample']]
    best=min(trace,key=lambda row:row['error_deg'])
    reduced=[dict(step=row['step'],error_deg=row['error_deg'],tilt_deg=row['tilt_deg'],
                  station_mm=row['center_error_mm'],upper_gap_mm=row['upper_gap_mm'],
                  upward_over_weight=row['whole_hand_support']['upward_support_over_weight'],
                  contacts=row['whole_hand_support']['contact_hand_geoms']) for row in trace]
    print(json.dumps(dict(best=next(row for row in reduced if row['step']==best['step']),
                         sampled_supported_states=len(supported),sampled_states=len(trace),
                         sampled_contact_not_full_1ms_history=True,
                         rows=reduced),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
