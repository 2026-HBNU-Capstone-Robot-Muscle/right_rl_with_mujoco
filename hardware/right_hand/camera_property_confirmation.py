"""Read-only confirmation of a transient camera property mismatch.

OpenCV DSHOW can return -1 when a value is unavailable. A mismatch is never
accepted as the requested setting: two consecutive fresh matching reads are
required within 100ms and three extra reads. No setter or old frame is used.
"""
import math
import time


def matches(value,expected):
    return math.isfinite(value) and abs(value-expected)<=.25


class PropertyReadConfirmation:
    def __init__(self,record,*,now=time.monotonic):
        self.record=record;self.now=now;self.recent=[]

    def read(self,cap,prop,expected,*,stop=None,label='exposure',initial=None):
        value=float(cap.get(prop) if initial is None else initial)
        if matches(value,expected):return value
        began=self.now();self.recent=[t for t in self.recent if began-t<=2.]
        event=dict(property=label,expected=expected,readbacks=[value],recovered=False,
            maximum_extra_reads=3,required_consecutive_matches=2,deadline_ms=100,settings_writes=0)
        self.record.setdefault('property_read_confirmations',[]).append(event)
        if len(self.recent)>=3:
            event['reason']='repeated_mismatch_limit'
            raise RuntimeError('Camera property repeatedly mismatched; confirmation limit exceeded')
        self.recent.append(began);streak=0
        for _ in range(3):
            if stop is not None and stop.is_set():raise KeyboardInterrupt('Camera user stop')
            if self.now()-began>.1:break
            value=float(cap.get(prop));event['readbacks'].append(value)
            elapsed=self.now()-began;event['elapsed_ms']=elapsed*1000
            if elapsed>.1:break
            streak=streak+1 if matches(value,expected) else 0
            if streak==2:event['recovered']=True;return value
        raise RuntimeError(f'Camera {label} not confirmed: expected {expected:g}, reads={event["readbacks"]}')
