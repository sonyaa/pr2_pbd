''' Distribution of possible poses. '''

import numpy as np
from geometry_msgs.msg import Pose, Vector3, Quaternion

#step:
# type=ARM_TARGET
# armTarget = ArmTarget():
#  lArm = ArmState():
#   refFrame = ArmState.<...>
#   ee_pose = Pose()
#   joint_pose = float64[]
#   refFrameObject = Object()
#  rArm = ArmState()
#  rArmVelocity = 0.02
#  lArmVelocity = 0.02
# gripperAction = GripperAction():
#  rGripper = GripperState():
#   state = GripperState.OPEN or GripperState.CLOSED
#  lGripper = GripperState()
import rospy
from pr2_pbd_interaction.msg import ActionStep, ArmTarget, GripperAction, ArmState


class ActionStepDistribution:
    def __init__(self):
        self._n_steps = 0
        self._ref_frames = [None, None]
        self._ref_frame_objects = [None, None]
        self._ee_poses = [[], []]
        self._joint_poses = [[], []]
        self._gripper_states = [[], []]

    def add_action_step(self, step):
        if self._n_steps == 0:
            self._ref_frames[0] = step.armTarget.rArm.refFrame
            self._ref_frames[1] = step.armTarget.lArm.refFrame
            self._ref_frame_objects[0] = step.armTarget.rArm.refFrameObject
            self._ref_frame_objects[1] = step.armTarget.lArm.refFrameObject
        self._ee_poses[0].append(self._get_array_from_pose(step.armTarget.rArm.ee_pose))
        self._ee_poses[1].append(self._get_array_from_pose(step.armTarget.lArm.ee_pose))
        self._joint_poses[0].append(np.array(step.armTarget.rArm.joint_pose))
        self._joint_poses[1].append(np.array(step.armTarget.lArm.joint_pose))
        self._gripper_states[0].append(np.array(step.gripperAction.rGripper))
        self._gripper_states[1].append(np.array(step.gripperAction.rGripper))
        self._n_steps += 1

    def _sample(self, data_points):
        mean = np.mean(data_points, axis=0)
        cov_matrix = np.cov(data_points, rowvar=0)
        return np.random.multivariate_normal(mean, cov_matrix)

    def _sample_1D(self, data_points):
        mean = np.mean(data_points)
        sd = np.std(data_points)
        if sd < 1e-3:
            sd = 1e-3
        return np.random.normal(mean, sd)

    @staticmethod
    def _get_pose_from_array(arr):
        return Pose(Vector3(arr[0], arr[1], arr[2]), Quaternion(arr[3], arr[4], arr[5], arr[6]))

    @staticmethod
    def _get_array_from_pose(pose):
        return np.array([pose.position.x, pose.position.y, pose.position.z,
                        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def get_sampled_action_step(self):
        step = ActionStep()
        step.type = ActionStep.ARM_TARGET
        arm_states = []
        for arm_index in [0,1]:
            arm_state = ArmState(self._ref_frames[arm_index],
                                 self._get_pose_from_array(self._sample(self._ee_poses[arm_index])),
                                 self._sample(self._joint_poses[arm_index]),
                                 self._ref_frame_objects[arm_index])
            arm_states.append(arm_state)
        step.armTarget = ArmTarget(arm_states[0], arm_states[1], 0.2, 0.2)
        step.gripperAction = GripperAction(round(self._sample_1D(self._gripper_states[0])),
                                           round(self._sample_1D(self._gripper_states[1])))
        return step
