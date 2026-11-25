"""Read-only search for custom SAC trainer helper modules in WSL home."""

from pathlib import Path

for name in ('sac_asym_networks.py', 'sac_simba_networks.py',
             'sac_demo_mix.py', 'sac_nstep.py'):
    matches = list(Path('/home/star1209042').rglob(name))
    print(name, [str(p) for p in matches[:10]], flush=True)
