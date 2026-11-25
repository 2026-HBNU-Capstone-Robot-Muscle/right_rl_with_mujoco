"""Bounded escape for one observed initial CAD/real disagreement, not a global tolerance."""
from horizontal_empty_recovery_20261027 import RING_CLEAR,needs_ring_clearance
from new_pair_grasp_plan_20261027 import ENTRY
PAIR='cadtip_mf_rf_mf_ds_0|cadtip_mf_rf_rf_ds_0'
MAX_INITIAL_MM=2.
POLICY=dict(version='observed_middle_ring_initial_separation_v1_20261027',pair=PAIR,
    maximum_initial_depth_mm=MAX_INITIAL_MM,ring_delta_ticks=96,maximum_ring_goals=48,
    static_depth_must_not_increase=True,dynamic_depth_must_not_exceed_initial=True,
    raw_self_contact_zero_before_roots=True,other_pairs_and_all_later_contacts_unchanged=True,
    physics_shapes_and_contact_forces_unchanged=True,physical_geometry_calibration_verified=False,
    observation='user reported no touching; fresh camera shows a gap; model and true CAD disagree with physical observation')

class InitialPairScope:
    def __init__(self,initial,enabled):
        self.enabled=bool(enabled);self.initial=dict(initial);self.initial_depth=float(initial.get(PAIR,0.))
        self.valid=bool((not initial) if not enabled else set(initial)<={PAIR} and 0<=self.initial_depth<=MAX_INITIAL_MM)
        self.maximum_raw=max(initial.values(),default=0.);self.maximum_increase=0.
    def filtered(self,phase,pairs):
        self.maximum_raw=max(self.maximum_raw,max(pairs.values(),default=0.))
        result=dict(pairs)
        if self.enabled and self.valid and phase in ('initial_hold',ENTRY,RING_CLEAR):
            depth=float(pairs.get(PAIR,0.));self.maximum_increase=max(self.maximum_increase,depth-self.initial_depth)
            if depth<=self.initial_depth+1e-6:result.pop(PAIR,None)
        return result
