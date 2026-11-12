"""Create a separate research search; retain the first study unchanged."""
from pathlib import Path
root=Path(__file__).resolve().parent
source=(root/'search_inverse_yaw_contact_20261029.py').read_text(encoding='utf-8')
source=source.replace("inverse_contact_search'","inverse_contact_search_v2'")
source=source.replace("lower=np.ceil(lo[ACTIVE]+.18/TICK);upper=np.floor(hi[ACTIVE]-.18/TICK)",
    "lower=np.ceil(lo[ACTIVE]+16);upper=np.floor(hi[ACTIVE]-16) # native limits unchanged; actual-state bounds still enforced")
source=source.replace("center=np.array([2810,2710,3020,2050,2790,2550,3050,2350.])",
    "center=np.array([2720,2674,3068,2308,2791,2613,3022,2122.])")
source=source.replace("spread=np.array([130,150,180,80,150,70,160,160.])",
    "spread=np.array([90,100,140,80,120,80,120,100.])")
source=source.replace('range(3):\n        proposals=', 'range(4):\n        proposals=')
source=source.replace('for _ in range(15)]','for _ in range(19)]')
(root/'search_inverse_yaw_contact_v2_20261029.py').write_text(source,encoding='utf-8')
print('Separate v2 research script written; no devices opened.')
