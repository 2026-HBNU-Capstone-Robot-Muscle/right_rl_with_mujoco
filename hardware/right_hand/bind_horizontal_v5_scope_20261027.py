from pathlib import Path
path=Path(__file__).resolve().parent/'horizontal_quarter_gate_20261027.py';text=path.read_text(encoding='utf-8')
old='off_preload_confirmation_policy=dict(PRELOAD_POLICY),'
new='initial_self_separation=separation_review(plan,'+"empty"+'),'+old
if text.count(old)!=2 or 'initial_self_separation=separation_review' in text:raise ValueError('Unexpected gate source')
# Generation already receives empty; validation receives the saved report.
first=text.index(old);text=text[:first]+text[first:].replace(old,new,1)
second=text.index(old,first+len(new));text=text[:second]+text[second:].replace(old,"initial_self_separation=separation_review(plan,report['empty_recovery']),"+old,1)
path.write_text(text,encoding='utf-8')
print('Bound source-specific initial-separation metrics and actual-angle gate into both reviews.')
