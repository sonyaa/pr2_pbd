#!/usr/bin/env python
import rospy
from Robot import Robot
from condition_types.Condition import Condition


class GripperCondition(Condition):
    """
    Checks that the grippers are in the same state as specified.
    """

    def __init__(self, *args, **kwargs):
        Condition.__init__(self, *args, **kwargs)
        self.r_gripper_position = None
        self.l_gripper_position = None
        if len(args) > 1:
            self.r_gripper_position = args[0]
            self.l_gripper_position = args[1]
        self.threshold = 0.015

    def set_gripper_positions(self, r_gripper, l_gripper):
        self.r_gripper_position = r_gripper
        self.l_gripper_position = l_gripper

    def set_threshold(self, threshold):
        self.threshold = threshold

    def check(self):
        robot = Robot.get_robot()
        if self.r_gripper_position is not None:
            if abs(self.r_gripper_position - robot.get_gripper_position(0)) > self.threshold:
                rospy.logwarn("Condition failure: right gripper is not in the same position")
                return False
        if self.l_gripper_position is not None:
            if abs(self.l_gripper_position - robot.get_gripper_position(1)) > self.threshold:
                rospy.logwarn("Condition failure: left gripper is not in the same position")
                return False
        return True