"""Finish the three saved-file geometry analyses in one offline interpreter."""
from audit_latest_direct_pose_20261023 import main as audit
from search_latest_grip_closure_20261023 import main as closure
from search_cube_placement_from_contact_20261023 import main as placement

if __name__=='__main__':
    audit()
    closure()
    placement()
