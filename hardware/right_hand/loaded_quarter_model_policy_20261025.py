"""Apply the user's scoped ID4 limit; retain the old passive OFF policy."""
import numpy as np
from observed_book_model_policy import policy as original_policy
from loaded_quarter_joint_limits_20261025 import configured_limits,setting_metadata
def policy(raw=None):
    r=original_policy(raw);lo,_=configured_limits();configured=float(lo[3]);original=float(r['native_ID4_lower_ticks'])
    r.update(version='user_ID4_lower2deg_with_observed_passive_reentry_v1',
        configured_ID4_lower_ticks=configured,original_reference_ID4_lower_ticks=original,
        extended_lower_ticks=min(configured,int(r['start_ID4_ticks'])-4),
        model_range_extended=True,user_configured_joint_limit=setting_metadata(),
        only_reentry_toward_original_range=False,only_reentry_toward_configured_range=True)
    return r
def correct_model(model,raw=None):
    import mujoco
    before_joint=model.jnt_range.copy();before_ctrl=model.actuator_ctrlrange.copy();r=policy(raw)
    joint=model.joint('th_cmc').id;act=model.actuator('th_cmc_act').id
    model.jnt_range[joint,0]=(r['extended_lower_ticks']-2048)*2*np.pi/4096
    model.actuator_ctrlrange[act,0]=model.jnt_range[joint,0]
    expected_joint=before_joint.copy();expected_ctrl=before_ctrl.copy()
    expected_joint[joint,0]=model.jnt_range[joint,0];expected_ctrl[act,0]=model.actuator_ctrlrange[act,0]
    if not np.array_equal(model.jnt_range,expected_joint) or not np.array_equal(model.actuator_ctrlrange,expected_ctrl):raise RuntimeError('Unrequested model range change')
    mujoco.mj_setConst(model,mujoco.MjData(model));return r
