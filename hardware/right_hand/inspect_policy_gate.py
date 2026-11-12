from pathlib import Path
p=Path(__file__).resolve().parent/'held_policy_shadow.py'
lines=p.read_text(encoding='utf-8-sig').splitlines()
for i,line in enumerate(lines):
    if line.startswith('def main('):
        print('\n'.join(f'{j+1}: {lines[j]}' for j in range(i,min(i+65,len(lines)))))
