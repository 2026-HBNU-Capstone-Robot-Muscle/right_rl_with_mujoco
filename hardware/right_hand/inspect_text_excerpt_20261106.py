"""Read selected local source lines for offline review."""
from pathlib import Path
import sys
sys.stdout.reconfigure(encoding='utf-8')
p=Path(sys.argv[1]);start=int(sys.argv[2]);end=int(sys.argv[3])
for n,line in enumerate(p.read_text(encoding='utf-8').splitlines(),1):
    if start<=n<=end:print(f'{n}: {line}')
