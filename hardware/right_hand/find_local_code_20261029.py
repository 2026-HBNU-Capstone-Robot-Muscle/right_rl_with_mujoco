"""Bounded source-text search for environments without rg."""
from pathlib import Path
import sys
needle=sys.argv[1];pattern=sys.argv[2] if len(sys.argv)>2 else '*.py'
for path in sorted(Path('.').glob(pattern)):
    if not path.is_file():continue
    matches=[(n,line.strip()) for n,line in enumerate(path.read_text(encoding='utf-8-sig',errors='replace').splitlines(),1) if needle in line]
    for n,line in matches[:2]:print(f'{path}:{n}: {line[:180]}')
