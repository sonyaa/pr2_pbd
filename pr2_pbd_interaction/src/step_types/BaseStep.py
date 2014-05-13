#!/usr/bin/env python
import rospy
from Exceptions import BaseObstructedError, ConditionError
from pr2_pbd_interaction.msg import ExecutionStatus
from step_types.Step import Step


class BaseStep(Step):
    ''' Step that moves the base.
    '''

    def __init__(self, *args, **kwargs):  #(self, end_pose):
        Step.__init__(self, *args, **kwargs)
        self.end_pose = args[0]


    def execute(self):
        from Robot import Robot
        robot = Robot.get_robot()
        # If self.is_while, execute everything in a loop until a condition fails. Else execute everything once.
        while True:
            for condition in self.conditions:
                if not condition.check():
                    rospy.logwarn("Condition failed when executing base step.")
                    if self.is_while:
                        break
                    if self.strategy == Step.STRATEGY_FAILFAST:
                        robot.status = ExecutionStatus.CONDITION_FAILED
                        raise ConditionError()
            # send a request to Robot to move to end_pose
            if not robot.move_base(self.end_pose):
                raise BaseObstructedError()
            if not self.is_while:
                break