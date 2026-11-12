"""Rigid palm identity during EMPTY preparation of the loaded experiment."""
from confirmed_thumb_rest_policy_20261026 import metadata as parent_metadata,PAIR,MAX_INITIAL_MM
def metadata(raw):
    r=parent_metadata(raw);body=r['confirmed_initial_palm_body'].copy()
    body.update(active=True,current_pose_physical_contact_operator_verified=False,
        basis='same compiled rigid palm; source admission and separate native escape check required')
    r['confirmed_initial_palm_body']=body
    if PAIR not in r['additional_initial_pairs']:r['additional_initial_pairs'].append(PAIR)
    return r
