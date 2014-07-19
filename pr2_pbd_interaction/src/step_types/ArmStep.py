#!/usr/bin/env python
from geometry_msgs.msg import Pose
import rospy
import time
from Exceptions import ArmObstructedError, ConditionError, StoppedByUserError
from Response import Response
from condition_types.GripperCondition import GripperCondition
from pr2_pbd_interaction.msg import ExecutionStatus, ArmState, ArmTrajectory, ArmTarget, GripperAction, Strategy
from pr2_pbd_interaction.msg._StepExecutionStatus import StepExecutionStatus
from pr2_social_gaze.msg import GazeGoal
from step_types.Step import Step


class ArmStep(Step):
    """ Holds one arm step - a pose for both arms.
    """
    ARM_TARGET = 0
    ARM_TRAJECTORY = 1
    def __init__(self, *args, **kwargs):
        Step.__init__(self, *args, **kwargs)
        self.conditions = [GripperCondition()]
        self.condition_order = range(len(self.conditions))
        self.type = ArmStep.ARM_TARGET
        self.armTarget = None
        self.armTrajectory = None
        self.gripperAction = None
        self.postCond = None

    def set_gripper_condition_poses(self, r_gripper, l_gripper):
        for condition in self.conditions:
            if isinstance(condition, GripperCondition):
                condition.set_gripper_positions(r_gripper, l_gripper)
                break

    def set_gripper_condition(self, new_condition):
        to_remove = None
        for condition in self.conditions:
            if isinstance(condition, GripperCondition):
                to_remove = condition
        if to_remove is not None:
            self.conditions.remove(to_remove)
        self.conditions.append(new_condition)

    def execute(self, action_data):
        from Robot import Robot
        robot = Robot.get_robot()
        # If self.is_while, execute everything in a loop until a condition fails. Else execute everything once.
        while True:
            if not self.ignore_conditions:
                for condition in [self.conditions[i] for i in self.condition_order]:
                    if not condition.check():
                        rospy.logwarn("Condition failed when executing arm step.")
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
                rospy.loginfo('Ignoring conditions for arm step')
            if robot.preempt:
                # robot.preempt = False
                robot.status = ExecutionStatus.PREEMPTED
                rospy.logerr('Execution of arm step failed, execution preempted by user.')
                raise StoppedByUserError()
            # send a request to Robot to move the arms to their respective targets
            if (self.type == ArmStep.ARM_TARGET):
                rospy.loginfo('Will perform arm target action step.')

                if (not robot.move_to_joints(self.armTarget.rArm,
                                             self.armTarget.lArm)):
                    if robot.preempt:
                        robot.status = ExecutionStatus.PREEMPTED
                        rospy.logerr('Execution of arm step failed, execution preempted by user.')
                        raise StoppedByUserError()
                    rospy.logwarn('Arms failed to reach target')
                    self.execution_status = StepExecutionStatus.FAILED
                    return
            # If arm trajectory action
            elif (self.type == ArmStep.ARM_TRAJECTORY):
                rospy.loginfo('Will perform arm trajectory action step.')
                # First move to the start frame
                if (not robot.move_to_joints(self.armTrajectory.r_arm[0],
                                             self.armTrajectory.l_arm[0])):
                    if robot.preempt:
                        robot.status = ExecutionStatus.PREEMPTED
                        rospy.logerr('Execution of arm step failed, execution preempted by user.')
                        raise StoppedByUserError()
                    rospy.logwarn('Arms failed to reach target')
                    self.execution_status = StepExecutionStatus.FAILED
                    return

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
                    self.execution_status = StepExecutionStatus.FAILED
                    return

            # If hand action do it for both sides
            if (self.gripperAction.rGripper !=
                    Robot.arms[0].get_gripper_state()):
                rospy.loginfo('Will perform right gripper action ' +
                              str(self.gripperAction.rGripper))
                Robot.arms[0].set_gripper(self.gripperAction.rGripper)
                # Response.perform_gaze_action(GazeGoal.FOLLOW_RIGHT_EE)

            if (self.gripperAction.lGripper !=
                    Robot.arms[1].get_gripper_state()):
                rospy.loginfo('Will perform LEFT gripper action ' +
                              str(self.gripperAction.lGripper))
                Robot.arms[1].set_gripper(self.gripperAction.lGripper)
                # Response.perform_gaze_action(GazeGoal.FOLLOW_LEFT_EE)

            # Wait for grippers to be done
            while (Robot.arms[0].is_gripper_moving() or
                       Robot.arms[1].is_gripper_moving()):
                time.sleep(0.01)
            rospy.loginfo('Hands done moving.')

            self.execution_status = StepExecutionStatus.SUCCEEDED

            # Verify that both grippers succeeded
            if ((not Robot.arms[0].is_gripper_at_goal()) or
                    (not Robot.arms[1].is_gripper_at_goal())):
                rospy.logwarn('Hand(s) did not fully close or open!')
            if not self.is_while:
                return


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
        ## WARNING: the following is not really copying
        copy.conditions = self.conditions[:]
        copy.postCond = self.postCond
        copy.is_while = self.is_while
        copy.ignore_conditions = self.ignore_conditions
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

    def is_relative(self, arm_index):
        if self.type == ArmStep.ARM_TARGET:
            if arm_index == 0 and self.armTarget.rArm.refFrame == ArmState.OBJECT:
                    return True
            if arm_index == 1 and self.armTarget.lArm.refFrame == ArmState.OBJECT:
                    return True
        elif self.type == ArmStep.ARM_TRAJECTORY:
            ## TODO
            pass
        return False
