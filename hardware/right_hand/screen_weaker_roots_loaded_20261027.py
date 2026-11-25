"""Additional loaded stress: weaker MF/RF response suggested by measured sag."""
import json,sys,hashlib
import numpy as np
from pathlib import Path
import screen_loaded_yaw_path_20261027 as screen
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261027/open_settle_221539/weaker_root_matrix'
original=screen.build
def build(*args,**kwargs):
    model,manifest=original(*args,**kwargs)
    for name,factor in (('if_mcp_act',1.3),('mf_mcp_act',.5),('rf_mcp_act',.7)):
        act=model.actuator(name).id;model.actuator_gainprm[act,0]*=factor;model.actuator_biasprm[act,1]*=factor
    return model,manifest
screen.build=build
sys.argv=['screen','--candidate','right_hand_rotation_20261027/yaw90_clearance_t8_early_root16/candidate.npz',
    '--output-folder',OUT.relative_to(ROOT).as_posix()]
screen.main()
path=OUT/'review.json';report=json.loads(path.read_text(encoding='utf-8'))
required=[v for v in report['cases'] if v['gain_scale']==.5]
report.update(response_multiplier_by_root={'8':1.3,'12':.5,'16':.7},required_response_scale=.5,
    required_cases=len(required),required_cases_passed=all(v['passed'] for v in required),
    matching_is_stress_hypothesis_not_verified_calibration=True,
    measured_plateau_reference='right_hand_rotation_20261027/open_settle_221539/measured_review.json',
    required_model_yaw_range_deg=[min(v['yaw_before_release_deg'] for v in required if v['yaw_before_release_deg'] is not None),
        max(v['yaw_before_release_deg'] for v in required if v['yaw_before_release_deg'] is not None)] if all(v['yaw_before_release_deg'] is not None for v in required) else None)
old=np.load(ROOT/'right_hand_rotation_20261027/yaw90_clearance_t8_early_root16/full_loaded_27/loaded_path.npz');new=np.load(OUT/'loaded_path.npz')
report['loaded_arrays_unchanged']=all(np.array_equal(old[k],new[k]) for k in old.files)
path.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(required_cases=len(required),required_passed=report['required_cases_passed'],loaded_arrays_unchanged=report['loaded_arrays_unchanged'],all27passed=report['all_passed'])),flush=True)
