''' Distribution of possible poses for one step of one action. '''

import numpy as np
import PyKDL
from geometry_msgs.msg import Pose, Vector3, Quaternion
from math import sqrt
import rospy
from visualization_msgs.msg import Marker, visualization_msgs

from pr2_pbd_interaction.msg import ActionStep, ArmTarget, GripperAction, ArmState, Object
from World import World

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


class ActionStepDistribution:
    def __init__(self, index):
        self._n_actions = 0
        self._ref_frames = [None, None]
        self._ref_frame_objects = [None, None]
        self._ee_poses = [[], []]
        self._joint_poses = [[], []]
        self._gripper_states = [None, None]
        self._id = index
        self._marker_publisher = rospy.Publisher('visualization_marker', Marker)

    @staticmethod
    def _get_absolute_pose(step, arm_index):
        if arm_index == 0:
            if step.armTarget.rArm.refFrame == ArmState.ROBOT_BASE:
                return step.armTarget.rArm.ee_pose
            else:
                rospy.loginfo(step.armTarget.rArm.ee_pose)
                rospy.loginfo(World.get_absolute_pose(step.armTarget.rArm))
                return World.get_absolute_pose(step.armTarget.rArm)
        else:
            if step.armTarget.lArm.refFrame == ArmState.ROBOT_BASE:
                return step.armTarget.lArm.ee_pose
            else:
                return World.get_absolute_pose(step.armTarget.lArm)

    def add_action_step(self, step):
        if self._n_actions == 0:
            self._ref_frames[0] = step.armTarget.rArm.refFrame
            self._ref_frames[1] = step.armTarget.lArm.refFrame
            self._ref_frame_objects[0] = step.armTarget.rArm.refFrameObject
            self._ref_frame_objects[1] = step.armTarget.lArm.refFrameObject
            self._gripper_states[0] = np.array(step.gripperAction.rGripper)
            self._gripper_states[1] = np.array(step.gripperAction.rGripper)
        self._ee_poses[0].append(self._get_array_from_pose(self._get_absolute_pose(step, 0)))
        self._ee_poses[1].append(self._get_array_from_pose(self._get_absolute_pose(step, 1)))
        self._joint_poses[0].append(np.array(step.armTarget.rArm.joint_pose))
        self._joint_poses[1].append(np.array(step.armTarget.lArm.joint_pose))
        self._n_actions += 1
        #self._update_viz()

    @staticmethod
    def _sample(data_points):
        mean = np.mean(data_points, axis=0)
        cov_matrix = np.cov(data_points, rowvar=0)
        return np.random.multivariate_normal(mean, cov_matrix)

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
        for arm_index in [0, 1]:
            arm_state = ArmState(ArmState.ROBOT_BASE,
                                 self._get_pose_from_array(self._sample(self._ee_poses[arm_index])),
                                 self._sample(self._joint_poses[arm_index]),
                                 Object())
            relative_arm_state = World.convert_ref_frame(arm_state,
                                                         self._ref_frames[arm_index],
                                                         self._ref_frame_objects[arm_index])
            arm_states.append(relative_arm_state)
        step.armTarget = ArmTarget(arm_states[0], arm_states[1], 0.2, 0.2)
        step.gripperAction = GripperAction(self._gripper_states[0], self._gripper_states[1])
        rospy.loginfo(step)
        return step

    def _update_viz(self):
        if self._n_actions > 1:
            for arm_index in [0, 1]:
                if self._ref_frames[arm_index] == ArmState.ROBOT_BASE:
                    frame_id = 'base_link'
                else:
                    frame_id = self._ref_frame_objects[arm_index]
                marker = self.get_ee_pose_variance_marker(arm_index, frame_id)
                rospy.loginfo("Got marker: " + str(marker.id))
                self._marker_publisher.publish(marker)

    def _get_ee_pose_mean_and_cov(self, arm_index):
        data_points = [x[0:3] for x in self._ee_poses[arm_index]]
        rospy.loginfo(data_points)
        rospy.loginfo(np.mean(data_points, axis=0))
        rospy.loginfo(np.cov(data_points, rowvar=0))
        rospy.loginfo(np.cov(np.transpose(np.array(data_points))))
        return np.mean(data_points, axis=0), np.cov(data_points, rowvar=0)

    def get_ee_pose_variance_marker(self, arm_index, frame_id):
        mean, cov = self._get_ee_pose_mean_and_cov(arm_index)
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ee_pose_variance"
        marker.id = abs(hash(marker.ns + str(self._id) + str(arm_index))) % (10 ** 8)
        marker.type = visualization_msgs.msg.Marker.SPHERE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.pose.position.x = mean[0]
        marker.pose.position.y = mean[1]
        marker.pose.position.z = mean[2]
        eig_values, eig_vectors = np.linalg.eig(cov)
        rospy.loginfo(eig_values)
        eigx_n = PyKDL.Vector(eig_vectors[0,0], eig_vectors[0,1], eig_vectors[0,2])
        eigy_n = -PyKDL.Vector(eig_vectors[1,0], eig_vectors[1,1], eig_vectors[1,2])
        eigz_n = PyKDL.Vector(eig_vectors[2,0], eig_vectors[2,1], eig_vectors[2,2])
        eigx_n.Normalize()
        eigy_n.Normalize()
        eigz_n.Normalize()
        rot = PyKDL.Rotation(eigx_n, eigy_n, eigz_n)
        quat = rot.GetQuaternion()
        rospy.loginfo(quat)
        #painting the Gaussian Ellipsoid Marker
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        for i in [0, 1, 2]:
            if eig_values[i] < 1e-6:
                eig_values[i] = 1e-6
            eig_values[i] = sqrt(eig_values[i])
        marker.scale.x = 0.1 #eig_values[0]*2
        marker.scale.y = 0.1 #eig_values[1]*2
        marker.scale.z = 0.1 #eig_values[2]*2
        rospy.loginfo(marker.scale)
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker
