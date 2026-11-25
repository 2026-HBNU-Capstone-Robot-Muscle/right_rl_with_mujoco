from pathlib import Path
p=Path(__file__).resolve().parent/'build_thumb_clear_root_empty_mode_20261026.py'
s=p.read_text(encoding='utf-8')
old="g=rep(g,'return r',\"stress="
new="g=rep(g,'    return r\\n',\"    stress="
if s.count(old)!=1:raise ValueError('Unexpected builder return replacement')
p.write_text(s.replace(old,new),encoding='utf-8')
