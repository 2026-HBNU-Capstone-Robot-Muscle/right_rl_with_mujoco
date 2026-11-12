"""One fixed, current-bounded ID12 P-gain comparison; no device access."""
import numpy as np

def make_profile():
    caps=np.full(16,450,dtype=int);caps[11]=225
    trips=np.full(16,550,dtype=int);trips[11]=250
    p=np.full(16,600,dtype=int);p[[7,15]]=450
    d=np.full(16,200,dtype=int);d[[7,11,15]]=150
    return dict(caps=caps,gains=dict(p_gain=p,i_gain=np.zeros(16,dtype=int),d_gain=d),current_trip_ma=trips)
