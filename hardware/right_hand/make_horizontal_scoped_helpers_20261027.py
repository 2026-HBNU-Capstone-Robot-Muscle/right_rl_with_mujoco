"""Copy unchanged health/model checks; scope the two horizontal-only changes."""
import ast,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    source=ROOT/'loaded_repeat_voltage_20261026.py';old=source.read_text(encoding='utf-8')
    health=old[:old.index('\ndef sample(')]
    needle='from goal_readback_confirmation import confirm_previous_goal'
    if health.count(needle)!=1:raise ValueError('Unexpected health source')
    health=health.replace(needle,'from horizontal_goal_confirmation_20261027 import confirm_previous_goal')
    base=(ROOT/'leap_new_pair_grasp_20261027.py').read_text(encoding='utf-8')
    sample=ast.get_source_segment(base,next(n for n in ast.parse(base).body if isinstance(n,ast.FunctionDef) and n.name=='sample'))
    health+='\nfrom read_recovery_confirmation import confirm_recovered_read\nfrom new_pair_grasp_plan_20261027 import HOLD\n\n'+sample+'\n'
    (ROOT/'horizontal_quarter_health_20261027.py').write_text(health,encoding='utf-8')
    screen_source=ROOT/'screen_current_yaw_recovery_20261027.py';screen=screen_source.read_text(encoding='utf-8')
    needle='    validate_off_state=staticmethod(healthy_off)\n'
    if screen.count(needle)!=1:raise ValueError('Unexpected model-screen source')
    screen=screen.replace(needle,needle+'    def plan(self,raw):\n        from horizontal_empty_recovery_20261027 import serial_roots\n        prefix=[(p,q) for p,q in super().plan(raw) if p in (ENTRY,ROOTS,OPEN,SETTLE)]\n        return serial_roots(self.validate_start(raw),prefix)\n')
    (ROOT/'screen_horizontal_empty_recovery_20261027.py').write_text(screen,encoding='utf-8')
    manifest=dict(health_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),model_screen_source_sha256=hashlib.sha256(screen_source.read_bytes()).hexdigest(),changes=['horizontal-only goal confirmation import; voltage body unchanged','existing complete-snapshot recovery and HOLD callback unchanged','horizontal-only serial-root queue; model checks unchanged'],devices_opened=False,motor_writes=0)
    out=ROOT/'right_hand_rotation_20261027/horizontal_scoped_helpers_manifest.json';out.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(manifest))
if __name__=='__main__':main()
