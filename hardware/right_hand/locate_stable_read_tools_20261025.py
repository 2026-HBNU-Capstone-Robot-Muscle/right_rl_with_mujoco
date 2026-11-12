from pathlib import Path
root=Path(__file__).resolve().parent
for p in sorted(root.glob('*.py')):
    if 'readonly' in p.name and any(s in p.name for s in ('reverse','hardware','hand')):print(p.name)
for name in ('hand_readonly_check.cmd','leap_reverse_start_readonly.cmd'):
    p=root/name
    if p.exists():print(name,p.read_text(encoding='utf-8-sig'))
