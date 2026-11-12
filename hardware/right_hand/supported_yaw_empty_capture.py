"""Release observation plus final EMPTY photo, without placement prompts."""
import numpy as np
from grasp_place_capture import GraspCapture


class EmptySupportedYawCapture(GraspCapture):
    def release(self,bus,command,report,guard):
        self.guard=guard
        return super().release(bus,command,report,guard)

    def finish(self,bus,command,report):
        observe=self._observer(bus,command,report,self.guard)
        state,photo=self._photo('empty_suffix_finished.jpg','empty_suffix_observation',observe)
        report.update(empty_suffix_photo=photo,final_on_state={k:v.tolist() for k,v in state.items()},
                      cube_was_allowed=False,physical_rotation_verified=False,physical_flip_verified=False,
                      final_command=np.asarray(command).tolist())
        print('EMPTY suffix photographed. Automatic torque OFF follows; no cube placement or removal prompt.',flush=True)
