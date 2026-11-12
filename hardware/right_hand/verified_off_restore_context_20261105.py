"""Ignore a failed camera only for exact RAM restoration after verified OFF."""
from contextlib import contextmanager
import numpy as np
FIELDS={98:'watchdog',102:'current_cap',108:'acceleration',112:'velocity',80:'d_gain',82:'i_gain',84:'p_gain'}
POLICY=dict(version='camera_independent_verified_OFF_RAM_restore_v1_20261105',torque_OFF_must_already_be_verified=True,addresses=sorted(FIELDS),targets_must_equal_pre_run_settings=True,camera_cancel_unchanged_while_ON=True,terminal_cancel_preserved=True,position_torque_mode_EEPROM_writes_outside_scope=True,health_and_communication_bounds_unchanged=True)

@contextmanager
def post_off_restore(bus,address,values,*,terminal_cancel=None):
    if not bus.report.get('all_torque_off_verified') or address not in FIELDS:
        yield;return
    key=FIELDS[address];initial=bus.report.get('initial',{}).get(key)
    if initial is None or not np.array_equal(np.asarray(values),np.asarray(initial)):raise RuntimeError('Post-OFF restoration differs from initial RAM settings')
    if terminal_cancel is None:
        from sequence_flip_checked import check_cancel
        terminal_cancel=check_cancel
    original=bus.cancel_callback;bus.cancel_callback=terminal_cancel
    bus.report['post_off_restore_policy']=dict(POLICY)
    try:yield
    finally:bus.cancel_callback=original
