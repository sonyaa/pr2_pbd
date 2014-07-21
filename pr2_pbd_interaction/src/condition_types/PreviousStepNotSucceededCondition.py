#!/usr/bin/env python
import rospy
from condition_types.PreviousStepCondition import PreviousStepCondition
from pr2_pbd_interaction.msg import StepExecutionStatus, Strategy


class PreviousStepNotSucceededCondition(PreviousStepCondition):
    """
    Checks that the previous step has succeeded.
    """

    def __init__(self, *args, **kwargs):
        PreviousStepCondition.__init__(self, *args, **kwargs)
        self.available_strategies = [Strategy.CONTINUE, Strategy.SKIP_STEP]
        self.current_strategy_index = 0

    def check(self):
        if self.prev_step_status == StepExecutionStatus.SUCCEEDED:
            rospy.loginfo("Condition failure: previous step has succeeded")
            return False
        return True