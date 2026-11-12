"""Reuse the genuine three checkpoints; update the new trajectory label only."""
from gapped_inverse_capture_20261029 import GappedInverseCapture
class SupportedInverseCapture(GappedInverseCapture):
    def hold_frame(self,bus,command,report,state):
        super().hold_frame(bus,command,report,state)
        report['camera_instruction']='GRASP HOLD; supported inverse trajectory follows; hands clear'
