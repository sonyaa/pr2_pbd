#!/usr/bin/env python
from geometry_msgs.msg import Point
import rospy
from Exceptions import ConditionError, StoppedByUserError
from pr2_pbd_interaction.msg import ExecutionStatus, Strategy
from step_types.Step import Step


class HeadStep(Step):
    ''' Step that moves the head.
    '''

    def __init__(self, *args, **kwargs):
        Step.__init__(self, *args, **kwargs)
        self.head_position = None
        if len(args) > 0:
            self.head_position = args[0]

    def execute(self):
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
                            return
                        elif strategy == Strategy.CONTINUE:
                            rospy.loginfo("Strategy is to continue, ignoring condition failure.")
                        elif strategy == Strategy.GO_TO_PREVIOUS_STEP:
                            rospy.loginfo("Strategy is to go to previous step.")
                            #TODO
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
            if not self.is_while:
                return