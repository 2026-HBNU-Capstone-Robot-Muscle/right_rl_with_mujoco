"""Bounds for separating an already resting OFF hand from support proxies.

These are local model-screen parameters, not allowable physical penetration or
manufacturer limits. They apply only to existing initial structural contacts.
New/deeper contact and overlap after recovery remain rejected. Actual current,
voltage, tracking, no-progress and enable-hold checks are unchanged.
"""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
SOURCE='book_settled_off_readonly_20261024_212026_419273/request.json'
# The previous stable OFF neighborhood reaches4.739mm in the conservative
# -5mm book-front hypothesis.5mm bounds that passive proxy mismatch; it never
# grants5mm of new contact along a trajectory.
INITIAL_SUPPORT_DEPTH_MM=5.
ROOT_ESCAPE_GOALS=70
ROOT_ESCAPE_SECONDS=3.5
INITIAL_MODEL_TRACKING_RAD=.16
AFTER_CLEARANCE_MODEL_TRACKING_RAD=.12


def policy():
    data=(ROOT/SOURCE).read_bytes();r=json.loads(data)
    if any(any(r['off_state'][key]) for key in ('torque','error','watchdog')):
        raise ValueError('Passive recovery source is not an OFF observation')
    return dict(version='passive_structural_support_separation_v1',
        measured_off_source=SOURCE,measured_off_source_sha256=hashlib.sha256(data).hexdigest(),
        maximum_initial_model_overlap_mm=INITIAL_SUPPORT_DEPTH_MM,
        root_escape_goals=ROOT_ESCAPE_GOALS,root_escape_seconds=ROOT_ESCAPE_SECONDS,
        initial_model_tracking_rad=INITIAL_MODEL_TRACKING_RAD,
        after_clearance_model_tracking_rad=AFTER_CLEARANCE_MODEL_TRACKING_RAD,
        actual_tracking_stop_rad=.18,existing_initial_contacts_only=True,
        new_or_deeper_contact_allowed=False,physical_penetration_measured=False,
        book_geometry_registration_verified=False)
