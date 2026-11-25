"""Add joint identity to an isolated model diagnostic; no criteria relaxed."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
code=(ROOT/'screen_horizontal_empty_recovery_20261027.py').read_text(encoding='utf-8')
code=code.replace('for front in FRONTS:','for front in (-10.,):').replace('for scale in (.5,1.,1.5):','for scale in (.5,):').replace('for offset in offsets():','for offset in [np.zeros(16,dtype=int)]:')
code=code.replace('raw_thumb_held=0.','raw_thumb_held=0.;range_worst=None')
old='range_excess=max(range_excess,float(max(ex)))'
new="if float(max(ex))>range_excess:\n                        range_excess=float(max(ex));j=int(np.argmax(ex));range_worst=dict(motor_id=int(simids[j])+1,phase=phase,model_ticks=float(d.qpos[j]/TICK+2048),native_low_ticks=float(native_low[j]/TICK+2048),native_high_ticks=float(native_high[j]/TICK+2048),excess_ticks=range_excess/TICK,command_ticks=int(q[simids[j]]),source_ticks=int(raw[simids[j]]))"
assert old in code;code=code.replace(old,new)
code=code.replace('unapproved_range_excess_rad=max(0,range_excess),initial_support_increase_mm','unapproved_range_excess_rad=max(0,range_excess),range_worst=range_worst,initial_support_increase_mm')
code=code.replace("status='current_off_empty_recovery_diagnostic'","status='diagnostic_only_identify_range_joint_not_operator_proof'")
(ROOT/'diagnose_reconnected_range_coarse_20261028.py').write_bytes(code.encode())
(ROOT/'diagnose_reconnected_range_fine_20261028.py').write_bytes(code.replace('.005','.002').encode())
print('Isolated coarse/fine diagnostic entries ready; original checker and limits unchanged.')
