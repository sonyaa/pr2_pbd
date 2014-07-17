#!/usr/bin/env python
from geometry_msgs.msg import Point
import rospy
from numpy import array
from numpy.linalg import norm
import time
from Exceptions import ConditionError, StoppedByUserError
from pr2_pbd_interaction.msg import ExecutionStatus, Strategy
from pr2_pbd_interaction.msg._StepExecutionStatus import StepExecutionStatus
from step_types.Step import Step


class HeadStep(Step):
    ''' Step that moves the head.
    '''

    def __init__(self, *args, **kwargs):
        Step.__init__(self, *args, **kwargs)
        self.head_position = None
        if len(args) > 0:
            self.head_position = args[0]

    def execute(self, action_data):
        from Robot import Robot
        robot = Robot.get_robot()
        # If self.is_while, execute everything in a loop until a condition fails. Else execute everything once.
        while True:
            if not self.ignore_conditions:
                for condition in [self.conditions[i] for i in self.condition_order]:
                    if not condition.check():
                        rospy.logwarn("Condition failed when executing base step.")
                        if self.is_while:
                            #TODO
                            return
                        strategy = condition.available_strategies[condition.current_strategy_index]
                        if strategy == Strategy.FAIL_FAST:
                            rospy.loginfo("Strategy is to fail-fast, stopping.")
                            robot.status = ExecutionStatus.CONDITION_FAILED
                            raise ConditionError()
                        elif strategy == Strategy.SKIP_STEP:
                            rospy.loginfo("Strategy is to skip step, skipping.")
                            self.execution_status = StepExecutionStatus.SKIPPED
                            return
                        elif strategy == Strategy.CONTINUE:
                            rospy.loginfo("Strategy is to continue, ignoring condition failure.")
                        elif strategy == Strategy.GO_TO_PREVIOUS_STEP:
                            rospy.loginfo("Strategy is to go to previous step.")
                            action_data.go_back = True
                            return
                        else:
                            rospy.logwarn("Unknown strategy " + str(self.strategy))
            else:
                rospy.loginfo('Ignoring conditions for head step')
            if robot.preempt:
                # robot.preempt = False
                robot.status = ExecutionStatus.PREEMPTED
                rospy.logerr('Execution of head step failed, execution preempted by user.')
                raise StoppedByUserError()
            # send a request to Robot to move to head_position
            robot.move_head_to_point(self.head_position)
            time.sleep(2)
            if self.distance_from_real() < 0.2:
                self.execution_status = StepExecutionStatus.SUCCEEDED
            else:
                self.execution_status = StepExecutionStatus.FAILED
                return
            if not self.is_while:
                return

    def distance_from_real(self):
        from Robot import Robot
        robot = Robot.get_robot()
        real_position = robot.get_head_position()
        needed_position = self.head_position
        arr1 = array([real_position.x,
                      real_position.y, real_position.z])
        arr2 = array([needed_position.x,
                      needed_position.y, needed_position.z])
        dist = norm(arr1 - arr2)
        if dist < 0.0001:
            dist = 0
        return dist