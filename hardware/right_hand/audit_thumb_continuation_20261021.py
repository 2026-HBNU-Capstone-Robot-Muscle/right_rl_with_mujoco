"""Offline-only continuation screen; never opens camera or motor devices."""
import thumb_continuation_plan as plan
from audit_thumb_prefix_20261021 import main

if __name__ == '__main__':
    main(plan, plan.REVIEW_PATH)
