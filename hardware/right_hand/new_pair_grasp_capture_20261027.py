"""Three fresh SPACE checkpoints; automatically opens before cube removal."""
import time,numpy as np
from grasp_place_capture import GraspCapture
from new_pair_grasp_plan_20261027 import OPEN_PATH
class NewPairGraspCapture(GraspCapture):
    PLACEMENT_TIMEOUT=300
    def _space(self,phase,observe):
        self.keys.arm();began=self.now();last_notice=began;limit=self.PLACEMENT_TIMEOUT if phase=='place_cube' else 90
        while True:
            state=observe(phase)
            if self.keys.consume_space():return state
            elapsed=self.now()-began
            if elapsed>limit:raise RuntimeError(phase+f' timed out after {limit}s; OFF')
            if phase=='place_cube' and self.now()-last_notice>=30:
                print(f'PLACE wait: {max(0,int(limit-elapsed))}s left. Hands clear, SPACE here starts grasp; Q stops.',flush=True);last_notice=self.now()
            self.sleep(.02)
    def require_open(self,command):
        if not np.array_equal(command,np.load(OPEN_PATH)['open_goal']):raise RuntimeError('Open endpoint not reached; no placement/removal prompt')
    def _observer(self,bus,command,report,guard):
        def observe(phase):
            report['phase']=phase;self.cancel()
            state,_=bus.checked_sample(bus,command,report,phase,snapshot_check=lambda s:guard.check(s['position'],command,np.arange(16),phase));return state
        return observe
    def place(self,bus,command,report,guard):
        self.require_open(command)
        self.guard=guard;bus.grasp_protocol=self;observe=self._observer(bus,command,report,guard);state,photo=self._photo('open_before_cube.jpg','open_before_cube',observe);report.update(open_before_cube_photo=photo,open_before_cube_state={k:v.tolist() for k,v in state.items()},camera_instruction='PLACE CUBE; GREEN up; ORANGE toward WRIST/ID41; SPACE after hands clear')
        print('PLACE CUBE NOW above the palm at the finger roots, between thumb and middle. GREEN up; ORANGE toward WRIST/ID41.',flush=True)
        print('Let cube rest without your hand. Do not force a fit. Hands clear, SECOND SPACE in this TERMINAL starts slow grasp.',flush=True)
        print('Placement wait is now up to5min with fresh motor health checks. If cube cannot rest, Q stops. ID40 may be hidden.',flush=True);self._space('place_cube',observe);state,photo=self._photo('cube_before_grasp.jpg','place_cube',observe);report.update(placement_photo=photo,placement_position_ticks=state['position'].tolist(),placement_goal_ticks=np.asarray(command).tolist(),cube_placement_confirmed=True,cube_present=True,registration_verified=False)
        for n in (3,2,1):
            print(f'Loaded grasp in {n}s; hands clear. Q stops.',flush=True);began=self.now()
            while self.now()-began<1:observe('loaded_countdown');self.sleep(.02)
    def hold_frame(self,bus,command,report,state):
        if not getattr(self,'hold_started',False):
            self.hold_started=True;self.hold_began=self.now();report['camera_instruction']='GRASP HOLD3s; automatic OPEN follows; hands clear';report['grasp_position_ticks']=state['position'].tolist();report['grasp_goal_ticks']=np.asarray(command).tolist();print('Grasp hold3s. Observe cube. Automatic OPEN follows; keep hands clear. No rotation.',flush=True)
        if not report.get('grasp_observation_photo'):
            photo=self.camera.capture_named('cube_grasp_hold.jpg',after=self.hold_began+.2)
            if photo is not None:report['grasp_observation_photo']=photo
    def finish(self,bus,command,report):
        if not report.get('cube_placement_confirmed'):raise RuntimeError('No explicit placement; OFF')
        self.require_open(command)
        if not hasattr(bus,'pending') or bus.pending:raise RuntimeError('Opening queue incomplete; no cube removal prompt')
        observe=self._observer(bus,command,report,self.guard);state,photo=self._photo('cube_after_open.jpg','opened_before_remove',observe);report.update(opened_before_remove_photo=photo,physical_grasp_verified=False,physical_flip_verified=False,camera_instruction='OPEN complete. REMOVE CUBE; THIRD SPACE = OFF')
        print('OPEN complete. REMOVE CUBE, then THIRD SPACE in this TERMINAL confirms removal and turns torque OFF.',flush=True);state=self._space('remove_cube',observe);report.update(final_on_state={k:v.tolist() for k,v in state.items()},cube_removal_confirmed=True,cube_present=False)
