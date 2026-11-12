"""Offline check for persistent commanded-against-stop motor behavior."""

import json
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
RUN = ROOT / 'leap_sequence_flip_20261011_051518_433595.json'
EMPTY_RUNS = (
    ROOT / 'leap_sequence_flip_20261006_003351_919305.json',
    ROOT / 'leap_sequence_flip_20261006_003605_661164.json',
)
TICK_RAD = 2 * np.pi / 4096
JOINTS = ('th_cmc', 'th_axl', 'th_mcp', 'th_ipl',
          'if_mcp', 'if_rot', 'if_pip', 'if_dip',
          'mf_mcp', 'mf_rot', 'mf_pip', 'mf_dip',
          'rf_mcp', 'rf_rot', 'rf_pip', 'rf_dip')


def main():
    run = json.loads(RUN.read_text(encoding='utf-8'))
    phases = defaultdict(list)
    for row in run['samples']:
        if row['phase'].startswith('stage_'):
            phases[row['phase']].append(row)
    report = dict(status='offline_mechanical_stop_pattern_check',
                  motor_commands_sent=0, source_file=str(RUN), phases={})
    for phase, rows in sorted(phases.items()):
        # Focus on the end of each stage, where a hard stop would leave a
        # persistent signed goal-minus-position error under load.
        tail = rows[-min(8, len(rows)):]
        goal = np.asarray([r['goal'] for r in tail], float)
        actual = np.asarray([r['actual'] for r in tail], float)
        current = np.asarray([r['current_ma'] for r in tail], float)
        error = (goal - actual) * TICK_RAD
        last = rows[-1]
        joints = []
        for i, name in enumerate(JOINTS):
            joints.append(dict(id=i + 1, joint=name,
                               median_signed_error_rad=float(np.median(error[:, i])),
                               last_signed_error_rad=float(error[-1, i]),
                               median_abs_current_ma=float(np.median(abs(current[:, i]))),
                               last_goal_tick=int(last['goal'][i]),
                               last_actual_tick=int(last['actual'][i])))
        report['phases'][phase] = dict(samples=len(rows),
                                       largest_end_errors=sorted(
                                           joints, key=lambda r: abs(r['median_signed_error_rad']),
                                           reverse=True)[:6])
    empty_compare = []
    for path in EMPTY_RUNS:
        empty = json.loads(path.read_text(encoding='utf-8'))
        for phase in ('stage_4', 'stage_5'):
            rows = [r for r in empty['samples'] if r['phase'] == phase]
            if not rows:
                continue
            tail = rows[-min(8, len(rows)):]
            i = 11  # ID12 / mf_dip
            empty_compare.append(dict(file=path.name, status=empty['status'], phase=phase,
                                      median_signed_error_rad=float(np.median([
                                          (r['goal'][i] - r['actual'][i]) * TICK_RAD for r in tail])),
                                      median_abs_current_ma=float(np.median([
                                          abs(r['current_ma'][i]) for r in tail])),
                                      last_goal_tick=int(rows[-1]['goal'][i]),
                                      last_actual_tick=int(rows[-1]['actual'][i])))
    report['historical_empty_ID12_comparison'] = empty_compare
    report['caveat'] = ('Persistent error/current can suggest a hard stop, but these logs contain '
                        'no direct contact sensor or video of the hidden joint stop. Joint names '
                        'follow the motor ID map; physical collision remains unverified.')
    (ROOT / 'sac_mechanical_stop_trace_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
