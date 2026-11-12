"""User-configured ID4 lower limit for the current supervised trial only."""
import hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
SETTING=ROOT/'loaded_quarter_id4_lower_setting_20261025.json'
BASE=ROOT/'right_hand_rotation_20261023/native_motor_limits.json'
def configured_limits():
    blob=BASE.read_bytes();r=json.loads(blob);setting=json.loads(SETTING.read_bytes())
    if (setting['motor_id']!=4 or setting['lower_degrees_by']!=2.0
        or setting['base_limits_sha256']!=hashlib.sha256(blob).hexdigest()
        or setting['hardware_eeprom_modified'] is not False):raise ValueError('ID4 lower-limit setting changed unexpectedly')
    lo=np.asarray(r['lower_ticks'],dtype=float);hi=np.asarray(r['upper_ticks'],dtype=float)
    lo[3]-=np.deg2rad(setting['lower_degrees_by'])/float(r['tick_rad'])
    if not np.isclose(lo[3],setting['configured_lower_ticks'],rtol=0,atol=1e-9):raise ValueError('ID4 lower-limit conversion mismatch')
    return lo,hi
def setting_metadata():
    r=json.loads(SETTING.read_bytes());r['setting_sha256']=hashlib.sha256(SETTING.read_bytes()).hexdigest();return r
