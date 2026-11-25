"""User-triggered next EMPTY thumb segment with the tested observation runner."""
import thumb_continuation_plan as plan
from leap_thumb_prefix_empty import Bus as PrefixBus, main


class Bus(PrefixBus):
    plan_module=plan
    planner=staticmethod(plan.validate_start)


if __name__=='__main__':
    raise SystemExit(main(plan, Bus))
