#!/usr/bin/env python
import rospy
from condition_types.Condition import Condition
from pr2_pbd_interaction.msg import Strategy, StepExecutionStatus


class PreviousStepNotSkippedCondition(Condition):
    """
    Checks that the previous step has failed.
    """

    def __init__(self, *args, **kwargs):
        Condition.__init__(self, *args, **kwargs)
        self.prev_step = None
        if len(args) > 0:
            self.prev_step = args[0]
        self.available_strategies = [Strategy.FAIL_FAST, Strategy.CONTINUE, Strategy.SKIP_STEP,
                                     Strategy.GO_TO_PREVIOUS_STEP]
        self.current_strategy_index = 0

    def set_prev_step(self, step):
        self.prev_step = step

    def check(self):
        if self.prev_step.execution_status == StepExecutionStatus.SKIPPED:
            return False
        return True