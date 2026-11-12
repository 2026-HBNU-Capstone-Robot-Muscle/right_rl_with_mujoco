"""Copy the reviewed runner with scoped progress handling, preserving old tools."""
import ast
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def once(s,a,b):
    if s.count(a)!=1:raise ValueError('Ambiguous runner block')
    return s.replace(a,b)

def main():
    source=(ROOT/'leap_thumb_prefix_empty.py').read_text(encoding='utf-8');tree=ast.parse(source)
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='run')
    function='\n'.join(source.splitlines()[node.lineno-1:node.end_lineno])+'\n'
    old='''                if len(progress):
                    recent=progress_history.setdefault(phase,deque(maxlen=20))
                    recent.append(state['position'][progress].copy())
                    if len(recent)==20:
                        stopped=(np.ptp(np.array(recent),axis=0)<=2)&(abs(command[progress]-state['position'][progress])>=64)
                        if np.any(stopped):
                            motor=int(progress[np.flatnonzero(stopped)[0]])+1
                            raise RuntimeError(f'ID{motor} no progress over 20 fresh samples with >=64 tick error; no target advance')'''
    new='''                if len(progress):
                    try:progress_guard.check(phase,progress,command,state['position'],now())
                    except GoalAwareProgressFault as exc:
                        report['progress_fault']=exc.details
                        raise'''
    function=once(function,old,new)
    function=once(function,'progress_history={};paused_seconds=0.;checkpoint_done=False;release_done=False',
        "progress_history={};progress_guard=GoalAwareProgress();report['progress_policy']=progress_metadata();paused_seconds=0.;checkpoint_done=False;release_done=False")
    function=once(function,'final_command=rows[-1][1].tolist(),thumb_travel_ticks=',
        'planned_final_command=rows[-1][1].tolist(),final_command=command.tolist(),thumb_travel_ticks=')
    function=once(function,'        command=s[\'goal\'].copy();bus.adopt(command)',
        "        command=s['goal'].copy();report['last_issued_command']=command.tolist();bus.adopt(command)")
    function=once(function,'            command=target.copy()',
        "            command=target.copy();report['last_issued_command']=command.tolist();report['last_issued_phase']=phase")
    function=once(function,'    finally:\n        bus.off();',
        "    finally:\n        if 'command' in locals():\n            report['last_issued_command']=command.tolist();report['final_command']=command.tolist()\n        bus.off();")
    header='''"""Dedicated quarter-path runner; old shared motion tools are untouched."""
import time
from collections import deque
import numpy as np
import sequence_flip_checked as checked
from leap_roots_empty import check_off
from goal_aware_progress_guard_20261025 import GoalAwareProgress,GoalAwareProgressFault,metadata as progress_metadata

'''
    (ROOT/'quarter_motion_runner_20261025.py').write_text(header+function,encoding='utf-8')
    current=(ROOT/'leap_thumb_prepare_current_empty.py').read_text(encoding='utf-8')
    current=once(current,'from leap_thumb_prefix_empty import Bus as ScheduledBus,run,record_off_endpoint',
        'from leap_thumb_prefix_empty import Bus as ScheduledBus,record_off_endpoint\nfrom quarter_motion_runner_20261025 import run')
    (ROOT/'quarter_current_runtime_20261025.py').write_text(current,encoding='utf-8')

if __name__=='__main__':main()
