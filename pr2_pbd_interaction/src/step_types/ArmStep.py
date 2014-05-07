#!/usr/bin/env python
from geometry_msgs.msg import Pose
import rospy
import time
from Exceptions import ArmObstructedError
from Robot import Robot
from condition_types.GripperCondition import GripperCondition
from pr2_pbd_interaction.msg import ExecutionStatus, ArmState, ArmTrajectory, ArmTarget, GripperAction
from step_types.Step import Step


class ArmStep(Step):
    """ Holds one arm step - a pose for both arms.
    """
    ARM_TARGET = 0
    ARM_TRAJECTORY = 1
    def __init__(self, *args, **kwargs):
        Step.__init__(self, *args, **kwargs)
        self.conditions = [GripperCondition()]
        self.type = ArmStep.ARM_TARGET
        self.armTarget = None
        self.armTrajectory = None
        self.gripperAction = None
        self.postCond = None

    def add_condition(self, condition):
        self.conditions.append(condition)

    def set_gripper_condition_poses(self, r_gripper, l_gripper):
        self.conditions[0].set_gripper_positions(r_gripper, l_gripper)

    def set_gripper_condition(self, condition):
        self.conditions[0] = condition

    def execute(self):
        for condition in self.conditions:
            try:
                condition.check()
            except:
                rospy.logerr("Condition failed when executing arm step.")
                raise
        # send a request to Robot to move the arms to their respective targets
        robot = Robot.get_robot()
        if (self.type == ArmStep.ARM_TARGET):
            rospy.loginfo('Will perform arm target action step.')

            if (not robot.move_to_joints(self.armTarget.rArm,
                                         self.armTarget.lArm)):
                robot.status = ExecutionStatus.OBSTRUCTED
                raise ArmObstructedError()
        # If arm trajectory action
        elif (self.type == ArmStep.ARM_TRAJECTORY):
            rospy.loginfo('Will perform arm trajectory action step.')
            # First move to the start frame
            if (not robot.move_to_joints(self.armTrajectory.r_arm[0],
                                         self.armTrajectory.l_arm[0])):
                robot.status = ExecutionStatus.OBSTRUCTED
                raise ArmObstructedError()

            #  Then execute the trajectory
            Robot.arms[0].exectute_joint_traj(self.armTrajectory.r_arm,
                                              self.armTrajectory.timing)
            Robot.arms[1].exectute_joint_traj(self.armTrajectory.l_arm,
                                              self.armTrajectory.timing)

            # Wait until both arms complete the trajectory
            while ((Robot.arms[0].is_executing() or Robot.arms[1].is_executing())
                   and not robot.preempt):
                time.sleep(0.01)
            rospy.loginfo('Trajectory complete.')

            # Verify that both arms succeeded
            if ((not Robot.arms[0].is_successful()) or
                    (not Robot.arms[1].is_successful())):
                rospy.logwarn('Aborting execution; ' +
                              'arms failed to follow trajectory.')
                robot.status = ExecutionStatus.OBSTRUCTED
                raise ArmObstructedError()

    def copy(self):
        """Makes a copy of an arm step"""
        copy = ArmStep()
        copy.type = int(self.type)
        if (copy.type == ArmStep.ARM_TARGET):
            copy.armTarget = ArmTarget()
            copy.armTarget.rArmVelocity = float(
                                    self.armTarget.rArmVelocity)
            copy.armTarget.lArmVelocity = float(
                                    self.armTarget.lArmVelocity)
            copy.armTarget.rArm = ArmStep._copy_arm_state(
                                                self.armTarget.rArm)
            copy.armTarget.lArm = ArmStep._copy_arm_state(
                                                self.armTarget.lArm)
        elif (copy.type == ArmStep.ARM_TRAJECTORY):
            copy.armTrajectory = ArmTrajectory()
            copy.armTrajectory.timing = self.armTrajectory.timing[:]
            for j in range(len(self.armTrajectory.timing)):
                copy.armTrajectory.rArm.append(
                    ArmStep._copy_arm_state(
                                        self.armTrajectory.rArm[j]))
                copy.armTrajectory.lArm.append(
                    ArmStep._copy_arm_state(
                                        self.armTrajectory.lArm[j]))
            copy.armTrajectory.rRefFrame = int(
                    self.armTrajectory.rRefFrame)
            copy.armTrajectory.lRefFrame = int(
                    self.armTrajectory.lRefFrame)
            ## WARNING: the following is not really copying
            r_obj = self.armTrajectory.rRefFrameObject
            l_obj = self.armTrajectory.lRefFrameObject
            copy.armTrajectory.rRefFrameObject = r_obj
            copy.armTrajectory.lRefFrameObject = l_obj
        copy.gripperAction = GripperAction(self.gripperAction.rGripper,
                                           self.gripperAction.lGripper)
        return copy

    @staticmethod
    def _copy_arm_state(arm_state):
        '''Makes a copy of the arm state'''
        copy = ArmState()
        copy.refFrame = int(arm_state.refFrame)
        copy.joint_pose = arm_state.joint_pose[:]
        copy.ee_pose = Pose(arm_state.ee_pose.position,
                            arm_state.ee_pose.orientation)
        ## WARNING: the following is not really copying
        copy.refFrameObject = arm_state.refFrameObject
        return copy