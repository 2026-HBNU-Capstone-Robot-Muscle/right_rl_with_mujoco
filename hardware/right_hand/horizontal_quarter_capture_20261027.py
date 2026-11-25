"""The same three operator checkpoints, with turn photographs before opening."""
import numpy as np
from new_pair_grasp_capture_20261027 import NewPairGraspCapture

class HorizontalQuarterCapture(NewPairGraspCapture):
    def hold_frame(self,bus,command,report,state):
        if not getattr(self,'hold_started',False):
            self.hold_started=True;self.hold_began=self.now();report.update(camera_instruction='GRASP HOLD3s; recorded horizontal turn follows; hands clear',grasp_position_ticks=state['position'].tolist(),grasp_goal_ticks=np.asarray(command).tolist());print('Grasp hold3s. The recorded LARGE horizontal turn follows; keep hands clear.',flush=True)
        if not report.get('grasp_observation_photo'):
            photo=self.camera.capture_named('cube_before_horizontal_turn.jpg',after=self.hold_began+.2)
            if photo is not None:report['grasp_observation_photo']=photo
    def rotation_frame(self,bus,command,report,state):
        if not getattr(self,'rotation_hold_started',False):
            self.rotation_hold_started=True;self.rotation_hold_began=self.now();report.update(camera_instruction='ROTATION HOLD3s; automatic OPEN follows; hands clear',rotation_goal_ticks=np.asarray(command).tolist(),rotation_position_ticks=state['position'].tolist(),physical_horizontal_90_verified=False);print('Large-turn targets complete. Hold3s, then automatic OPEN. Actual cube angle is not yet verified.',flush=True)
        if not report.get('rotation_observation_photo'):
            photo=self.camera.capture_named('cube_horizontal_turn_hold.jpg',after=self.rotation_hold_began+.2)
            if photo is not None:report['rotation_observation_photo']=photo
