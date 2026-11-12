"""Bind the existing third-contact search to today's recorded forward ending."""
from pathlib import Path
R=Path(__file__).resolve().parent
s=(R/'research_postturn_third_contact_20261104.py').read_text(encoding='utf-8')
anchor='import json,time';assert s.count(anchor)==1
s=s.replace(anchor,"import os\nos.environ['JAX_PLATFORMS']='cpu'\n"+anchor)
anchor='def main():';assert s.count(anchor)==1
s=s.replace(anchor,"CP=__import__('pathlib').Path(__file__).resolve().parent/'right_hand_rotation_20261105/preintegration/reference_end'\nOUT=CP.parent/'third_contact'\nOUT.mkdir(parents=True,exist_ok=True)\n\n"+anchor)
s=s.replace("CP/'closest_model.mjb'","CP/'end_model.mjb'")
anchor="OUT/'turn_0_lower_8_normal_10/end_state.npz'";assert s.count(anchor)==1
s=s.replace(anchor,"CP.parent/'body_grip/lower_m20_t16_n10/end_state.npz'")
anchor="for finger,active in [('if',np.arange(0,4)),('rf',np.arange(8,12))]:";assert s.count(anchor)==1
s=s.replace(anchor,"for finger,active in [('if',np.arange(0,4))]:")
anchor='np.clip(initial,low[active]+1e-7,high[active]-1e-7),0.,0.';assert s.count(anchor)==1
s=s.replace(anchor,'np.clip(initial,low[active]+1e-7,high[active]-1e-7),14.,0.')
anchor="np.r_[low[active],-8.,-20.],np.r_[high[active],18.,20.]";assert s.count(anchor)==1
s=s.replace(anchor,"np.r_[low[active],10.,-20.],np.r_[high[active],22.,20.]")
(R/'search_lower_index_contact_20261105.py').write_text(s,encoding='utf-8')
print('Offline-only index contact search created; native/contact checks preserved.')
