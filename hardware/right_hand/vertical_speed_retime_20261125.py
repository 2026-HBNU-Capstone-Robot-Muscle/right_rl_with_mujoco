"""Offline source-row selection for an isolated speed experiment.

No device imports or writes. Source phase boundaries, reversals and static
holds are mandatory; the entry path remains at its original sampling rate.
"""
import numpy as np


def select_rows(start, rows, phases, factor=3):
    rows = np.asarray(rows)
    start = np.asarray(start)
    phases = np.asarray(phases)
    if (factor not in (1, 2, 3) or rows.ndim != 2 or rows.shape[1] != 16
            or start.shape != (16,) or len(rows) != len(phases) or not len(rows)
            or not np.isfinite(rows).all() or np.any(rows != np.rint(rows))):
        raise ValueError('Invalid retiming source.')
    whole = np.vstack([start, rows])
    if np.max(abs(np.diff(whole, axis=0))) > 2:
        raise ValueError('Source must retain the original2tick bound.')
    mandatory = {0, len(rows)-1}
    last_nonzero = np.zeros(16, dtype=int)
    previous_phase = None
    for i, delta in enumerate(np.diff(whole, axis=0)):
        if phases[i] != previous_phase:
            mandatory.add(i)
            if i:
                mandatory.add(i-1)
            last_nonzero[:] = 0
            previous_phase = phases[i]
        signs = np.sign(delta).astype(int)
        if np.any((signs != 0) & (last_nonzero != 0) & (signs != last_nonzero)):
            mandatory.add(i)
            if i:
                mandatory.add(i-1)
        last_nonzero[signs != 0] = signs[signs != 0]
        if str(phases[i]).startswith('direct_entry_'):
            mandatory.add(i)
    # Keep every tick of any constant run so settling/holding time is unchanged.
    equal = np.all(np.diff(whole, axis=0) == 0, axis=1)
    for i in np.flatnonzero(equal):
        mandatory.add(int(i))
        if i:
            mandatory.add(int(i-1))
    anchors = sorted(mandatory)
    selected = [anchors[0]]
    for target in anchors[1:]:
        index = selected[-1]
        while index+factor < target:
            index += factor
            selected.append(index)
        if selected[-1] != target:
            selected.append(target)
    indices = np.asarray(selected, dtype=int)
    chosen = rows[indices].astype(int)
    if np.max(abs(np.diff(np.vstack([start, chosen]), axis=0))) > 2*factor:
        raise ValueError('Retimed target exceeded its proposed step bound.')
    # Verify the skipped source points remain within1tick of their new straight
    # interpolation. Extra pins are added instead of cutting across a corner.
    while True:
        extras = set()
        for left, right in zip(indices[:-1], indices[1:]):
            span = right-left
            expected = rows[left]+(rows[right]-rows[left])*np.arange(span+1)[:, None]/span
            bad = np.flatnonzero(np.max(abs(rows[left:right+1]-expected), axis=1) > 1.+1e-9)
            extras.update(int(left+i) for i in bad)
        if not extras:
            break
        indices = np.asarray(sorted(set(indices.tolist()) | extras), dtype=int)
        chosen = rows[indices].astype(int)
    return chosen, phases[indices], indices, np.asarray(sorted(mandatory), dtype=int)
