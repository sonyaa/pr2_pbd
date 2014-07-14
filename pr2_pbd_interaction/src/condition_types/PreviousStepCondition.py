#!/usr/bin/env python
import rospy
from condition_types.Condition import Condition
from pr2_pbd_interaction.msg import Strategy, StepExecutionStatus


class PreviousStepCondition(Condition):
    """
    Condition related to previous step.
    """

    def __init__(self, *args, **kwargs):
        Condition.__init__(self, *args, **kwargs)
        self.prev_step_status = None
        if len(args) > 0:
            self.prev_step_status = args[0]
        self.available_strategies = [Strategy.FAIL_FAST, Strategy.CONTINUE, Strategy.SKIP_STEP,
                                     Strategy.GO_TO_PREVIOUS_STEP]
        self.current_strategy_index = 0

    def set_prev_step_status(self, status):
        self.prev_step_status = status

    def check(self):
        return True