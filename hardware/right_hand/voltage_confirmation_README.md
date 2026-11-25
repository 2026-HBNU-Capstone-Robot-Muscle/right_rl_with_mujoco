# Voltage confirmation in the checked sequence runner

2026-10-13: user requested adjustment of nuisance low-voltage stops. Applied to
`sequence_flip_checked.py`, which is also used by `leap_sac_start_empty.cmd`.
Other independent SAC/controller implementations are not automatically changed.

The manufacturer's XC330-M288 input range is 3.7–6.0V, recommended 5V. Voltage
telemetry uses 0.1V units. Source:
https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/

This change does not claim reliable operation below that range. It replaces
immediate termination on a single 3.6V sample during an already-enabled motion
with a bounded confirmation procedure:

- Do not advance or resend targets, re-enable torque, change current/PID, or
  restart a trajectory during confirmation. The prior goal remains active;
  this is not a mechanical freeze and the joints may still approach that goal.
- Read fresh full motor-health snapshots. Require every ID >=3.8V in two
  consecutive reads within a 150ms confirmation budget, at most six reads.
- Stop if not confirmed. A reading <=3.5V, hardware fault, watchdog/torque change,
  overcurrent, excessive tracking error, overtemperature or overvoltage still
  terminates immediately upon detection. Communication failure propagates.
- Three low-voltage events in two seconds stop rather than repeatedly resuming.
- Enable-time health checks remain strict: no transient continuation there.
- Ordinary healthy samples perform no additional reads or delays.

The timing/margin/repetition settings are engineering choices, not manufacturer
certified transient-undervoltage allowances. The 150ms deadline is checked
before/after reads; a blocking SDK read can delay return beyond it. A late read
cannot authorize resumption. The existing 200ms completed-snapshot limit and
device watchdog remain in effect. The software is not a hard real-time cutoff.

`voltage_policy` and `voltage_events` in result JSON record the active rules,
initial/recheck voltages, phase, confirmation count, duration and outcome.
Voltage and temperature failures now identify their own offending ID correctly.

Validation: 14 new offline fault-injection/log-input tests plus 11 existing
start-check/export tests pass (25 total). New tests assert no write during
confirmation and OFF/settings restoration after persistent undervoltage.
The archived 3.6V trial stops at its first low sample; its later recovery is
unknown, so no successful hardware recovery is claimed.

Next physical validation: empty hand, existing `leap_sac_start_empty.cmd`,
one terminal SPACE. No cube or policy motion. This one repeat validates changed
voltage handling; it is not a request to keep retrying a persistent fault.
