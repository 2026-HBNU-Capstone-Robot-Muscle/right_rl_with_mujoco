"""Correct a derived tick-difference field from the saved arrays, no devices."""
import json
from pathlib import Path
import numpy as np
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_loaded_model'
trace=np.load(OUT/'nominal_trace.npz')
observed=json.loads((ROOT/'reverse_start_placement_20261025_075405_164917/result.json').read_bytes())['placement_position_ticks']
diff=(ticks_to_sim(np.array(observed))-trace['initial_state'][1:17])*4096/(2*np.pi)
p=OUT/'contact_timing.json';r=json.loads(p.read_bytes())
r['native_initial_hand_difference_from_observation_ticks']=np.round(diff,3).tolist()
r['native_initial_hand_difference_motor_ids']=list(SIM_TO_MOTOR_IDS)
r['tick_difference_metadata_corrected_from_saved_arrays']=True
p.write_text(json.dumps(r,indent=2),encoding='utf-8')
print('OBSERVED_MINUS_NATIVE_TICKS',np.round(diff,2).tolist())
