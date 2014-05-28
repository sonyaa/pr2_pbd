#!/usr/bin/env python
import rospy
from Exceptions import BaseObstructedError, ConditionError, StoppedByUserError
from pr2_pbd_interaction.msg import ExecutionStatus, Strategy
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
                    if self.strategy == Strategy.FAIL_FAST:
                        rospy.loginfo("Strategy is to fail-fast, stopping.")
                        robot.status = ExecutionStatus.CONDITION_FAILED
                        raise ConditionError()
                    elif self.strategy == Strategy.CONTINUE:
                        rospy.loginfo("Strategy is to continue, skipping this step.")
                        break
                    else:
                        rospy.logwarn("Unknown strategy " + str(self.strategy))
            if robot.preempt:
                # robot.preempt = False
                robot.status = ExecutionStatus.PREEMPTED
                rospy.logerr('Execution of base step failed, execution preempted by user.')
                raise StoppedByUserError()
            # send a request to Robot to move to end_pose
            if not robot.move_base(self.end_pose):
                if robot.preempt:
                    robot.status = ExecutionStatus.PREEMPTED
                    rospy.logerr('Execution of base step failed, execution preempted by user.')
                    raise StoppedByUserError()
                raise BaseObstructedError()
            if not self.is_while:
                break