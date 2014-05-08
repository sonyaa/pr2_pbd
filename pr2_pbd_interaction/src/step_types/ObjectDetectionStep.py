#!/usr/bin/env python
import rospy
from Exceptions import NoObjectError, ConditionError
from Robot import Robot
from World import World
from pr2_pbd_interaction.msg import ExecutionStatus
from step_types.Step import Step


class ObjectDetectionStep(Step):
    """ Step that looks for objects on the table.
    """

    def execute(self):
        robot = Robot.get_robot()
        # If self.is_while, execute everything in a loop until a condition fails. Else execute everything once.
        while True:
            for condition in self.conditions:
                if not condition.check():
                    rospy.logwarn("Condition failed when executing object detection step.")
                    if self.is_while:
                        break
                    if self.strategy == Step.STRATEGY_FAILFAST:
                        robot.status = ExecutionStatus.CONDITION_FAILED
                        raise ConditionError()
            # call object detection
            world = World.get_world()
            if not world.update_object_pose():
                rospy.logwarn("Object detection failed.")
                robot.status = ExecutionStatus.OBSTRUCTED
                raise NoObjectError()
            if not self.is_while:
                break