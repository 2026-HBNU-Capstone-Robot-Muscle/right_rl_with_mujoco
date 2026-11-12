"""Bounded return diagnostic. Uses the caller's OFF/restoration finally block."""
import time
import sequence_flip_checked as checked
from id12_fixed_hold import observe
from id12_backoff_spec import targets,DT

def run(bus,command,report,*,now=time.monotonic,sleep=time.sleep):
    goals=targets(command)
    observe(bus,command,report,now=now,sleep=sleep,report_key='approach_hold',
            phase='id12_approach_hold',next_action='bounded ID12 return')
    report['phase']='id12_backoff'
    report['backoff']=dict(start_goal=command.tolist(),end_goal=goals[-1].tolist(),
        write_attempts=0,commands_sent=0,completed=False)
    began=now()
    print('ID12 RETURN: 3290 -> 3130 ticks over 1.6s; all other goals fixed. Q stops.',flush=True)
    for goal in goals:
        checked.check_cancel()
        if now()-began>4:raise RuntimeError('ID12 return exceeded 4s; no target advance')
        cycle=now()
        # Fresh health before a new goal; no target retry or catch-up writes.
        checked.sample(bus,command,report,report['phase'])
        checked.check_cancel()
        if now()-began>4:raise RuntimeError('ID12 return exceeded 4s; no target advance')
        report['backoff']['write_attempts']+=1
        bus.write(116,4,goal,verify=False)
        command=goal.copy();report['backoff']['commands_sent']+=1
        sleep(.02)
        checked.sample(bus,command,report,report['phase'])
        remaining=DT-(now()-cycle)
        if remaining>0:sleep(remaining)
    report['backoff'].update(completed=True,elapsed_seconds=now()-began)
    observe(bus,command,report,now=now,sleep=sleep,report_key='return_hold',phase='id12_return_hold')
    before=report['approach_hold']['actual_end_median'][11]
    after=report['return_hold']['actual_end_median'][11]
    report['backoff']['actual_change_ticks']=after-before
    print(f'ID12 measured return: {before:.1f} -> {after:.1f} ticks. OFF follows; no forward retry.',flush=True)
