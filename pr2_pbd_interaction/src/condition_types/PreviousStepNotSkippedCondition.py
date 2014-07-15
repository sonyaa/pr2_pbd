#!/usr/bin/env python
import rospy
from condition_types.PreviousStepCondition import PreviousStepCondition
from pr2_pbd_interaction.msg import StepExecutionStatus


class PreviousStepNotSkippedCondition(PreviousStepCondition):
    """
    Checks that the previous step was not skipped.
    """

    def __init__(self, *args, **kwargs):
        PreviousStepCondition.__init__(self, *args, **kwargs)

    def check(self):
        if self.prev_step_status == StepExecutionStatus.SKIPPED:
            rospy.loginfo("Condition failure: previous step was skipped")
            return False
        return True