"""Conditional marker-to-model geometry for visual checking only, not deployment."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parent


def candidate():
    data = json.loads((ROOT / 'hand_registration_measurements.json').read_text(encoding='utf-8'))
    xml = ET.parse(ROOT.parent / 'work/Leap_Test/train/env/xmls/leap_rh_mjx.xml')
    palm = next(b for b in xml.iter('body') if b.get('name') == 'palm')
    site = next(s for s in xml.iter('site') if s.get('name') == 'grasp_site')
    bodies = {b.get('name'): b for b in palm if b.tag == 'body'}
    q = np.fromstring(palm.get('quat'), sep=' ')
    q /= np.linalg.norm(q)
    w, x, y, z = q
    R_wp = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
    t_wp = np.fromstring(palm.get('pos'), sep=' ')
    p9_p = np.fromstring(bodies['mf_bs'].get('pos'), sep=' ')
    g_w = np.fromstring(site.get('pos'), sep=' ')
    # H = fingertips, away from thumb, outward normal. Mapping is provisional.
    R_ph = np.diag([1., -1., -1.])
    # Photo-checked cardinal correspondence: marker +X toward thumb; +Y fingertips.
    R_hm = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
    R_pm = R_ph @ R_hm
    m = dict(data['marker_center_to_reference_axis_mm'])
    thumb_constraint = data.get('thumb_axis_lateral_constraint')
    if thumb_constraint is not None:
        # The finger MCP axes run laterally; the thumb axis constrains this
        # remaining translation. Shift the entire rigid registration, never
        # the thumb landmark alone. Preserve the original measurement in JSON.
        thumb_p = np.fromstring(bodies['th_mp'].get('pos'), sep=' ')
        thumb_minus_middle_m = R_pm.T @ (thumb_p - p9_p)
        middle_thumbward_m = (thumb_constraint['marker_center_to_axis_toward_thumb_mm'] / 1000
                              - thumb_minus_middle_m[0])
        m['away_from_thumb'] = -1000 * middle_thumbward_m
    d_h = np.array([m['toward_fingertips'], m['away_from_thumb'], m['above_marker_plane']]) / 1000
    marker_p = p9_p - R_ph @ d_h
    g_p = R_wp.T @ (g_w - t_wp)
    g_m = R_pm.T @ (g_p - marker_p)
    landmarks = {str(i): (R_pm.T @ (np.fromstring(bodies[name].get('pos'), sep=' ') - marker_p)).tolist()
                 for i, name in ((5, 'if_bs'), (9, 'mf_bs'), (13, 'rf_bs'), (1, 'th_mp'))}
    R_wm = R_wp @ R_pm
    assert np.allclose(R_wm.T @ R_wm, np.eye(3)) and np.isclose(np.linalg.det(R_wm), 1)
    assert np.allclose(R_wp @ (marker_p + R_pm @ g_m) + t_wp, g_w)
    assert np.allclose(landmarks['9'], R_hm.T @ d_h)
    if thumb_constraint is not None:
        assert np.isclose(landmarks['1'][0] * 1000,
                          thumb_constraint['marker_center_to_axis_toward_thumb_mm'])
    return {'status': 'unverified_visual_candidate', 'hardware_authorized': False,
            'hand_to_world_candidate': R_wm.tolist(), 'grasp_site_in_marker_m_candidate': g_m.tolist(),
            'motor_axis_landmarks_in_marker_m': landmarks,
            'assumptions': data['assumptions_requiring_validation'],
            'measurements': m,
            'measurement_note': 'Effective registration inputs; lateral motor-9 coordinate is derived when thumb constraint is present',
            'thumb_axis_lateral_constraint': thumb_constraint}


if __name__ == '__main__':
    print(json.dumps(candidate(), indent=2))
