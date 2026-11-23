"""Install a scoped EMPTY checker and rebindable adapters; no device access."""
import json
import shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parent
BACK=ROOT/'right_hand_rotation_20261113/initial_empty_scope/before'
BACK.mkdir(parents=True,exist_ok=True)
frozen=json.loads((ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json').read_bytes())['code_hashes']

def edit(name,transforms):
    assert name not in frozen,name
    p=ROOT/name;backup=BACK/name
    if backup.exists():raise RuntimeError('Preserve prior patch backup: '+name)
    shutil.copyfile(p,backup);text=p.read_text(encoding='utf-8-sig')
    for old,new in transforms:
        assert text.count(old)==1,(name,old,text.count(old))
        text=text.replace(old,new)
    p.write_text(text,encoding='utf-8')

source=(ROOT/'screen_calibrated_empty_margin_20261028.py').read_text(encoding='utf-8-sig')
source=source.replace('from rewired_initial_pair_scope_20261027 import InitialPairScope,PAIR',
    'from rewired_initial_pair_scope_20261027 import PAIR\nfrom empty_start_thumb_index_scope_20261113 import EmptyInitialScope as InitialPairScope')
assert source.count('InitialPairScope(sp,self_scope_enabled)')==1
assert source.count('InitialPairScope(initial_self,self_scope_enabled)')==1
source=source.replace('InitialPairScope(sp,self_scope_enabled)',"InitialPairScope(sp,self_scope_enabled,mode='static')")
source=source.replace('InitialPairScope(initial_self,self_scope_enabled)',"InitialPairScope(initial_self,self_scope_enabled,mode='dynamic')")
source=source.replace('passed=passed,max_self_mm=', 'passed=passed,thumb_index_initial_scope=selfscope.review(),max_self_mm=')
assert source.count('thumb_index_initial_scope=selfscope.review()')==2
target=ROOT/'screen_empty_initial_scope_20261113.py'
assert not target.exists();target.write_text(source,encoding='utf-8')
for stem in ['support_first','tip_first']:
    plan=stem+'_trial_plan_20261112.py';gate=stem+'_trial_gate_20261112.py';checker='screen_'+stem+'_current_20261112.py'
    edit(plan,[("paths+=sorted((OUT/'models').glob('*.json'))", "paths+=sorted((OUT/'models').glob('*.json'))\n    paths += [ROOT/'empty_start_thumb_index_scope_20261113.py',ROOT/'screen_empty_initial_scope_20261113.py']")])
    edit(gate,[('from calibrated_quarter_gate_20261028 import bounded','from calibrated_quarter_gate_20261028 import bounded\nfrom empty_start_thumb_index_scope_20261113 import validate_report as validate_initial_scope'),
               ('    r=transport.make_review(plan,empty,transport.hashes());new=models()', '    initial_scope=validate_initial_scope(empty)\n    r=transport.make_review(plan,empty,transport.hashes());new=models()'),
               ('loaded_goals=len(candidate()[\'goals\']),support_first_models=new,active_motion_limit_s=500,', "loaded_goals=len(candidate()['goals']),support_first_models=new,active_motion_limit_s=500,\n        empty_initial_thumb_index_scope=initial_scope,")])
    edit(checker,[('import screen_calibrated_empty_margin_20261028 as screen','import screen_empty_initial_scope_20261113 as screen')])
print('Scoped EMPTY checker installed; proof rebinding and model checks still required. No hardware opened.')
