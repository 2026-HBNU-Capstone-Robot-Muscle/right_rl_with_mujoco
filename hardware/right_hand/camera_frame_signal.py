"""Pixel-content checks, independent of drivers, GUI and all motor code.

This distinguishes blank frames from a scene, not a hand/cube detector.
"""
import numpy as np


class CameraImageUnavailable(RuntimeError):
    pass


def frame_signal(frame):
    a=np.asarray(frame)
    if a.ndim!=3 or a.shape[2]!=3 or min(a.shape[:2])<2:
        return dict(usable=False,reason='invalid_shape',maximum=None,std=None)
    # Subsample to avoid adding a full-frame image pass to control cycles.
    a=a[::8,::8] if min(a.shape[:2])>=16 else a
    if not np.isfinite(a).all():
        return dict(usable=False,reason='nonfinite',maximum=None,std=None)
    low=float(a.min());high=float(a.max());std=float(a.std())
    reason='black' if high<=3 else 'flat' if high-low<=1 or std<=.25 else 'scene_signal'
    return dict(usable=reason=='scene_signal',reason=reason,minimum=low,maximum=high,std=std)


class FrameSignal:
    def __init__(self):
        self.latest=None;self.stamp=None;self.good_streak=0;self.bad_since=None

    def observe(self,stamp,frame):
        if self.stamp is not None and stamp<=self.stamp:return self.latest
        self.latest=frame_signal(frame);self.stamp=stamp
        if self.latest['usable']:
            self.good_streak+=1;self.bad_since=None
        else:
            self.good_streak=0
            if self.bad_since is None:self.bad_since=stamp
        return self.latest

    def ready(self):
        return self.latest is not None and self.latest['usable'] and self.good_streak>=3

    def prolonged_blank(self,now):
        return self.bad_since is not None and now-self.bad_since>=.5
