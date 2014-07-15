#!/usr/bin/env python
import rospy
from condition_types.PreviousStepCondition import PreviousStepCondition
from pr2_pbd_interaction.msg import StepExecutionStatus


class PreviousStepNotFailedCondition(PreviousStepCondition):
    """
    Checks that the previous step has not failed.
    """

    def __init__(self, *args, **kwargs):
        PreviousStepCondition.__init__(self, *args, **kwargs)

    def check(self):
        if self.prev_step_status == StepExecutionStatus.FAILED:
            rospy.loginfo("Condition failure: previous step has failed")
            return False
        return True