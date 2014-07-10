#!/usr/bin/env python
import rospy
from condition_types.Condition import Condition
from pr2_pbd_interaction.msg import Strategy


class IKCondition(Condition):
    """
    Checks that the poses in the manipulation are reachable.
    """

    def __init__(self, *args, **kwargs):
        Condition.__init__(self, *args, **kwargs)
        self.step = None
        if len(args) > 0:
            self.step = args[0]
        self.available_strategies = [Strategy.FAIL_FAST, Strategy.SKIP_STEP]
        self.current_strategy_index = 0

    def set_step(self, step):
        self.step = step

    def check(self):
        from Robot import Robot
        robot = Robot.get_robot()
        if not robot.solve_ik_for_manipulation_step(self.step):
            rospy.logwarn('Condition failure: Problems in finding IK solutions')
            return False
        return True