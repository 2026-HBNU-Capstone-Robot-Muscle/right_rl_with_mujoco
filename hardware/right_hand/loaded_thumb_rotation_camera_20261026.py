"""Recover image gaps only before cube placement; loaded motion stays strict."""
from recovering_empty_camera_20261026 import RecoveringEmptyCamera
from quarter_finish_empty_camera_20261025 import QuarterFinishEmptyCamera
from empty_camera_gap_hold_20261026 import metadata as parent_metadata
def metadata():
    r=parent_metadata();r.update(scope='empty_preparation_before_PLACE_prompt_only',loaded_camera_failure_stops=True,
                               recovery_disabled_during_placement_and_removal=True)
    return r
def placement_may_have_cube(report):
    return bool(report.get('cube_present') or report.get('cube_placement_window_open'))
class BeforePlacementGuard:
    def __init__(self,guard,report):self.guard=guard;self.report=report
    def wait_if_needed(self,*args,**kwargs):
        if placement_may_have_cube(self.report):return 0.
        return self.guard.wait_if_needed(*args,**kwargs)
class LoadedThumbCamera(RecoveringEmptyCamera):
    EMPTY_ONLY=False
    def check(self):
        if placement_may_have_cube(self.report):return QuarterFinishEmptyCamera.check(self)
        return super().check()
