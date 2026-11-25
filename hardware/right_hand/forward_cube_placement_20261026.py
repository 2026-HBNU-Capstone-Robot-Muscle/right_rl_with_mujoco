"""Placement-only comparison; motor route and all checks stay in the reviewed parent."""
import json
import hashlib
from pathlib import Path
import numpy as np
import sequence_flip_checked as checked
from loaded_quarter_finish_plan_20261025 import LoadedQuarterFinishPlan, ROOT, FIXTURE
from loaded_quarter_finish_protocol_20261025 import LoadedQuarterFinishCapture
from leap_loaded_quarter_finish_trial_20261025 import LoadedFinishCamera, require_offline_review

REVIEW = ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/forward_placement_readiness.json'
SOURCE = ROOT/'loaded_quarter_finish_trial_20261026_005744_776918/result.json'
SOURCE_SHA = 'b596675734addf96a159b57715c04136060ead089fb94de19f9b7a2e1edf63d8'
FILES = ('forward_cube_placement_20261026.py', 'leap_forward_cube_placement_20261026.py', 'leap_forward_cube_placement.cmd')

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

class ForwardPlacementCapture(LoadedQuarterFinishCapture):
    def place(self, bus, command, report, guard):
        self.guard = guard
        observe = self._observer(bus, command, report, guard)
        state, photo = self._photo('empty_before_place.jpg', 'prepared_empty_reference', observe)
        report.update(prepared_empty_photo=photo, prepared_empty_ticks=state['position'].tolist(),
            cube_rotation_requested=True, physical_flip_verified=False,
            expected_reference_seconds=38., expected_new_finish_seconds=3.2,
            placement_comparison=dict(previous_source=SOURCE.parent.name,
                requested_forward_shift_mm=5., shift_is_operator_estimate=True,
                shift_measured_or_camera_registered=False, motor_route_changed=False,
                prior_physical_flip_completed=False, prior_operator_report='Cube remained tilted; appeared caught on thumb'))
        report['camera_instruction'] = 'PLACE: about5mm farther toward FINGERTIPS than prior placement. GREEN UP, ORANGE toward WRIST/ID41. Hands clear, SPACE.'
        print('PLACE CUBE NOW, about5mm farther toward FINGERTIPS than last time, away from the thumb/wrist.', flush=True)
        print('Keep GREEN UP and ORANGE toward WRIST/ID41. Keep it on the flat INDEX/MIDDLE motor-link tops.', flush=True)
        print('Placement direction guide: right_hand_rotation_20261024/loaded_finish_review_0057_20261026/forward_placement_guide.html', flush=True)
        print('It must rest by itself. Do not force it in or balance it on an unsupported edge. Q stops if it will not rest.', flush=True)
        print('Hands clear, SECOND SPACE in TERMINAL starts the SAME38s reference +3.2s finish after3s.', flush=True)
        self._space('place_cube', observe)
        state, photo = self._photo('cube_before_turn.jpg', 'place_cube', observe)
        report.update(placement_photo=photo, placement_position_ticks=state['position'].tolist(),
            placement_goal_ticks=np.asarray(command).tolist(), cube_placement_confirmed=True, cube_present=True,
            registration_verified=False, placement_current_ma=checked.signed_current(state['current']).tolist())
        report['camera_instruction'] = 'LOADED: same reference + verified finish once. Hands clear. Q stops.'
        for n in range(3, 0, -1):
            print(f'Same reference and finish in {n}s; hands clear. Q stops.', flush=True)
            began = self.now()
            while self.now()-began < 1:
                observe('loaded_countdown')
                self.sleep(.02)

class ForwardPlacementPlan(LoadedQuarterFinishPlan):
    # The parent plan version and hash are intentionally retained: no target,
    # phase, health check, current, PID, range or timing change is made here.
    FOLDER_PREFIX = 'forward_cube_placement'
    STATUS = 'forward_cube_placement_complete'
    TITLE = 'PLACEMENT COMPARISON: EMPTY -> PLACE5mm toward tips -> SAME41.2s turn -> REMOVE -> OFF.'
    COMPLETE_MESSAGE = 'Placement comparison ended. OFF/settings restored. Physical flip awaits photo/video and operator review.'
    CAMERA_TYPE = LoadedFinishCamera
    CAPTURE_PROTOCOL_TYPE = ForwardPlacementCapture

def require_placement_review(path=REVIEW):
    proof = require_offline_review()
    r = json.loads(Path(path).read_bytes())
    if (sha(SOURCE) != SOURCE_SHA or r.get('source_sha256') != SOURCE_SHA
            or r.get('review_ready') is not True
            or r.get('prior_physical_flip_completed') is not False
            or r.get('placement_only_comparison') is not True
            or r.get('requested_forward_shift_mm') != 5.
            or r.get('cube_hand_registration_verified') is not False
            or r.get('physical_ninety_degree_rotation_verified') is not False
            or r.get('mode_code_hashes') != {n: sha(ROOT/n) for n in FILES}):
        raise RuntimeError('Placement comparison review/source mismatch; no devices opened')
    for name in ('plan', 'frozen_for_phase', 'progress_for_phase', 'release_indices', 'proof_hashes'):
        if getattr(ForwardPlacementPlan, name) is not getattr(LoadedQuarterFinishPlan, name):
            raise RuntimeError('Placement-only mode changed motor planning; no devices opened')
    plan = ForwardPlacementPlan(FIXTURE/'request.json')
    base = LoadedQuarterFinishPlan(FIXTURE/'request.json')
    if plan.plan_digest(plan.source_position()) != base.plan_digest(base.source_position()):
        raise RuntimeError('Placement-only motor route digest mismatch; no devices opened')
    from loaded_quarter_finish_gate_20261025 import validate_proof
    validate_proof(plan, FIXTURE/'screen.json')
    return proof
