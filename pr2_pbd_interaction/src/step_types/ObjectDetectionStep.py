#!/usr/bin/env python
import rospy
from Exceptions import NoObjectError, ConditionError, StoppedByUserError
from World import World
from pr2_pbd_interaction.msg import ExecutionStatus, Strategy
from step_types.Step import Step
import time


class ObjectDetectionStep(Step):
    """ Step that looks for objects on the table.
    """

    def __init__(self, *args, **kwargs):
        from Robot import Robot
        Step.__init__(self, *args, **kwargs)
        self.head_position = Robot.get_head_position()

    def execute(self):
        from Robot import Robot
        robot = Robot.get_robot()
        robot.move_head_to_point(self.head_position)
        time.sleep(1)
        # If self.is_while, execute everything in a loop until a condition fails. Else execute everything once.
        while True:
            if not self.ignore_conditions:
                for condition in self.conditions:
                    if not condition.check():
                        rospy.logwarn("Condition failed when executing object detection step.")
                        if self.is_while:
                            return
                        if self.strategy == Strategy.FAIL_FAST:
                            rospy.loginfo("Strategy is to fail-fast, stopping.")
                            robot.status = ExecutionStatus.CONDITION_FAILED
                            raise ConditionError()
                        elif self.strategy == Strategy.CONTINUE:
                            rospy.loginfo("Strategy is to continue, skipping this step.")
                            return
                        else:
                            rospy.logwarn("Unknown strategy " + str(self.strategy))
            else:
                rospy.loginfo('Ignoring conditions for object detection step')
            if robot.preempt:
                robot.status = ExecutionStatus.PREEMPTED
                rospy.logerr('Execution of object detection step failed, execution preempted by user.')
                raise StoppedByUserError()
            # call object detection
            world = World.get_world()
            if not world.update_object_pose():
                if robot.preempt:
                    robot.status = ExecutionStatus.PREEMPTED
                    rospy.logerr('Execution of object detection step failed, execution preempted by user.')
                    raise StoppedByUserError()
                rospy.logwarn("Object detection failed.")
                robot.status = ExecutionStatus.OBJECT_DETECTION_FAILED
                raise NoObjectError()
            time.sleep(1)
            if not self.is_while:
                return