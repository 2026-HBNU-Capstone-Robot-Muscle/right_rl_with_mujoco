"""Bounded placement/photo/removal with fixed goals and fresh health reads."""
import time
import numpy as np
import sequence_flip_checked as checked


class TerminalKeys:
    """Preserve SPACE through health checks; require release between prompts."""
    def __init__(self,reader=None,now=time.monotonic):
        self.reader=reader;self.now=now;self.pending=False;self.armed=False;self.wait_quiet=False;self.last_space=now()

    def poll(self):
        if self.reader is None:
            import msvcrt
            def read():return msvcrt.getwch() if msvcrt.kbhit() else None
        else:read=self.reader
        while True:
            key=read()
            if key is None:break
            if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
            if key==' ':
                if self.armed:self.pending=True;self.armed=False
                self.last_space=self.now()
        if self.wait_quiet and self.now()-self.last_space>=.35:
            self.armed=True;self.wait_quiet=False

    def arm(self):
        self.pending=False;self.armed=False;self.wait_quiet=True;self.last_space=self.now()

    def consume_space(self):
        value=self.pending;self.pending=False;return value


def capture(bus,command,report,camera,keys,*,cancel,sleep=time.sleep,now=time.monotonic):
    def observe(phase):
        cancel();state,_=checked.sample(bus,command,report,phase)
        return state

    def fresh_photo(name,phase):
        began=now()
        while True:
            state=observe(phase)
            photo=camera.capture_named(name,after=began+.2)
            if photo is not None:return state,photo
            if now()-began>2:raise RuntimeError('No fresh placement camera frame; OFF')
            sleep(.02)

    def wait_space(phase,timeout):
        keys.arm();began=now()
        while True:
            state=observe(phase)
            if keys.consume_space():return state
            if now()-began>timeout:raise RuntimeError(phase+' timed out; OFF')
            sleep(.02)

    report['phase']='prepared_empty_reference'
    state,photo=fresh_photo('prepared_empty.jpg',report['phase'])
    report['prepared_empty_photo']=photo
    report['prepared_empty_ticks']=state['position'].tolist()
    report['prepared_empty_on_state']={k:v.tolist() for k,v in state.items()}
    report['phase']='place_cube'
    print('PLACE CUBE NOW in the open space above the palm, between the fingers. Do not force a fit.',flush=True)
    print('Reference orientation: GREEN up, ORANGE toward camera. Keep the whole cube in view.',flush=True)
    print('Hands clear, SECOND SPACE in TERMINAL saves the photo (within 90s). Goals hold; no flip runs.',flush=True)
    print('ID40 may be hidden. This photo has no marker requirement. Q stops.',flush=True)
    wait_space('place_cube',90)
    state,photo=fresh_photo('cube_placed.jpg','place_cube')
    report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),
                  placement_goal_ticks=np.asarray(command).tolist(),registration_verified=False,
                  placement_current_ma=checked.signed_current(state['current']).tolist())
    report['phase']='remove_cube'
    print('Photo saved. REMOVE CUBE, then THIRD SPACE in TERMINAL confirms removal and turns torque OFF.',flush=True)
    state=wait_space('remove_cube',90)
    report['final_on_state']={k:v.tolist() for k,v in state.items()}
    report['cube_removal_confirmed']=True
