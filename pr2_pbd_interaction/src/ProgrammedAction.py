'''Classes related to programmed actions'''

import roslib
from World import World

roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import threading
import os
from geometry_msgs.msg import Vector3, Pose, Quaternion, Point
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np

# ROS Libraries
import rospy
import rosbag
from pr2_pbd_interaction.msg import ArmState, ActionStepSequence
from pr2_pbd_interaction.msg import ActionStep, ArmTarget
from pr2_pbd_interaction.msg import GripperAction, ArmTrajectory
from ActionStepMarker import ActionStepMarker
from std_msgs.msg import Header, ColorRGBA


class ProgrammedAction:
    '''Class that holds information for one action'''

    _marker_publisher = None

    def __init__(self, action_index, step_click_cb):
        self.seq = ActionStepSequence()
        self.action_index = action_index
        self.step_click_cb = step_click_cb
        self.r_markers = []
        self.l_markers = []
        self.r_links = dict()
        self.l_links = dict()
        self.lock = threading.Lock()

        if ProgrammedAction._marker_publisher == None:
            ProgrammedAction._marker_publisher = rospy.Publisher(
                    'visualization_marker_array', MarkerArray)

    def get_name(self):
        '''Returns the name of the action'''
        return 'Action' + str(self.action_index)

    def add_action_step(self, step, object_list):
        '''Function to add a new step to the action'''
        self.lock.acquire()
        self.seq.seq.append(self._copy_action_step(step))
        if (step.type == ActionStep.ARM_TARGET
            or step.type == ActionStep.ARM_TRAJECTORY):
            last_step = self.seq.seq[len(self.seq.seq) - 1]
            r_marker = ActionStepMarker(self.n_frames(), 0,
                        last_step, self.marker_click_cb)
            r_marker.update_ref_frames(object_list, step.armTarget.rArm.refFrameObject)
            l_marker = ActionStepMarker(self.n_frames(), 1,
                        last_step, self.marker_click_cb)
            l_marker.update_ref_frames(object_list, step.armTarget.lArm.refFrameObject)
            self.r_markers.append(r_marker)
            self.l_markers.append(l_marker)
            if (self.n_frames() > 1):
                self.r_links[self.n_frames() - 1] = self._get_link(0,
                                                        self.n_frames() - 1)
                self.l_links[self.n_frames() - 1] = self._get_link(1,
                                                        self.n_frames() - 1)
        self.lock.release()

    def _get_link(self, arm_index, to_index):
        '''Returns a marker representing a link b/w two
        consecutive action steps'''
        if (arm_index == 0):
            start = self.r_markers[to_index - 1].get_absolute_position(
                                                            is_start=True)
            end = self.r_markers[to_index].get_absolute_position(
                                                            is_start=False)
        else:
            start = self.l_markers[to_index - 1].get_absolute_position(
                                                            is_start=True)
            end = self.l_markers[to_index].get_absolute_position(
                                                            is_start=False)

        return Marker(type=Marker.ARROW, id=(2 * to_index + arm_index),
                      lifetime=rospy.Duration(2),
                      scale=Vector3(0.01, 0.03, 0.01),
                      header=Header(frame_id='base_link'),
                      color=ColorRGBA(0.8, 0.8, 0.8, 0.3), points=[start, end])

    def update_objects(self, object_list):
        '''Updates the object list for all action steps'''
        self.lock.acquire()
        self._update_markers()
        action_objects = self._get_action_objects()
        action_object_unique = self._get_unique_action_objects(action_objects)
        map_of_objects_old_to_new = World.get_map_of_most_similar_obj(action_object_unique, object_list)
        for i in range(len(self.r_markers)):
            r_new_object = None
            if map_of_objects_old_to_new is not None:
                r_old_object = action_objects[i][0]
                if r_old_object is not None:
                    r_new_object = map_of_objects_old_to_new[r_old_object.name]
            self.r_markers[i].update_ref_frames(object_list, r_new_object)
        for i in range(len(self.l_markers)):
            l_new_object = None
            if map_of_objects_old_to_new is not None:
                l_old_object = action_objects[i][1]
                if l_old_object is not None:
                    l_new_object = map_of_objects_old_to_new[l_old_object.name]
            self.l_markers[i].update_ref_frames(object_list, l_new_object)
        self.lock.release()

    def _update_markers(self):
        '''Updates the markers after a change'''
        for i in range(len(self.r_markers)):
            self.r_markers[i].update_viz()
        for i in range(len(self.l_markers)):
            self.l_markers[i].update_viz()

    def reset_targets(self, arm_index):
        '''Resets requests after reaching a previous target'''
        self.lock.acquire()
        if (arm_index == 0):
            for i in range(len(self.r_markers)):
                self.r_markers[i].pose_reached()
        else:
            for i in range(len(self.l_markers)):
                self.l_markers[i].pose_reached()
        self.lock.release()

    def delete_requested_steps(self):
        '''Delete steps that were requested from interactive
        marker menus'''
        self.lock.acquire()
        to_delete = None
        for i in range(len(self.r_markers)):
            if (self.r_markers[i].is_deleted or
                self.l_markers[i].is_deleted):
                rospy.loginfo('Will delete step ' + str(i + 1))
                self.r_markers[i].is_deleted = False
                self.l_markers[i].is_deleted = False
                to_delete = i
                break
        if (to_delete != None):
            self._delete_step(to_delete)
        self.lock.release()

        if (to_delete != None):
            self.update_viz()
            self._update_markers()

    def _delete_step(self, to_delete):
        '''Deletes a step from the action'''
        if (len(self.r_links) > 0):
            self.r_links[self.r_links.keys()[-1]].action = Marker.DELETE
            self.l_links[self.l_links.keys()[-1]].action = Marker.DELETE
            self.r_links.pop(self.r_links.keys()[-1])
            self.l_links.pop(self.l_links.keys()[-1])

        self.r_markers[-1].destroy()
        self.l_markers[-1].destroy()
        for i in range(to_delete + 1, self.n_frames()):
            self.r_markers[i].decrease_id()
            self.l_markers[i].decrease_id()
        self.r_markers.pop(to_delete)
        self.l_markers.pop(to_delete)
        self.seq.seq.pop(to_delete)

    def change_requested_steps(self, r_arm, l_arm):
        '''Change an arm step to the current end effector
        pose if requested through the interactive marker menu'''
        self.lock.acquire()
        for i in range(len(self.r_markers)):
            if (self.r_markers[i].is_edited):
                self.r_markers[i].set_target(r_arm)
        for i in range(len(self.l_markers)):
            if (self.l_markers[i].is_edited):
                self.l_markers[i].set_target(l_arm)
        self.lock.release()

    def get_requested_targets(self, arm_index):
        '''Get arm steps that might have been requested from
        the interactive marker menus'''
        pose = None
        self.lock.acquire()
        if (arm_index == 0):
            for i in range(len(self.r_markers)):
                if (self.r_markers[i].is_requested):
                    pose = self.r_markers[i].get_target()
        else:
            for i in range(len(self.l_markers)):
                if (self.l_markers[i].is_requested):
                    pose = self.l_markers[i].get_target()
        self.lock.release()
        return pose

    def _update_links(self):
        '''Updates the visualized links b/w action steps'''
        for i in self.r_links.keys():
            self.r_links[i] = self._get_link(0, i)
        for i in self.l_links.keys():
            self.l_links[i] = self._get_link(1, i)

    def update_viz(self):
        '''Update the visualization of the action'''
        self.lock.acquire()
        self._update_links()
        m_array = MarkerArray()
        for i in self.r_links.keys():
            m_array.markers.append(self.r_links[i])
        for i in self.l_links.keys():
            m_array.markers.append(self.l_links[i])
        self._marker_publisher.publish(m_array)
        self.lock.release()

    def clear(self):
        '''Clear the action'''
        self.reset_viz()
        self.lock.acquire()
        self.seq = ActionStepSequence()
        self.r_markers = []
        self.l_markers = []
        self.r_links = dict()
        self.l_links = dict()
        self.lock.release()

    def undo_clear(self):
        '''Undo effect of clear'''
        self.seq = []

    def _get_filename(self, ext='.bag'):
        '''Returns filename for the bag that holds the action'''
        if ext[0] != '.':
            ext = '.' + ext
        return self.get_name() + ext

    def n_frames(self):
        '''Returns the number of steps in the action'''
        return len(self.seq.seq)

    def save(self, data_dir):
        '''Saves the action into a file'''
        if (self.n_frames() > 0):
            self.lock.acquire()
            demo_bag = rosbag.Bag(data_dir + self._get_filename(), 'w')
            demo_bag.write('sequence', self.seq)
            demo_bag.close()
            self.lock.release()
        else:
            rospy.logwarn('Could not save demonstration because ' +
                          'it does not have any frames.')

    def load(self, data_dir):
        '''Loads an action from a file'''
        filename = data_dir + self._get_filename()
        if (os.path.exists(filename)):
            self.lock.acquire()
            demo_bag = rosbag.Bag(filename)
            for dummy, msg, bag_time in demo_bag.read_messages(
                                            topics=['sequence']):
                rospy.loginfo('Reading demo bag file at time '
                                + str(bag_time.to_sec()))
                self.seq = msg
            demo_bag.close()
            self.lock.release()
        else:
            rospy.logwarn('File does not exist, cannot load demonstration: '
                          + filename)

    def reset_viz(self):
        '''Removes all visualization from RViz'''
        self.lock.acquire()
        for i in range(len(self.r_markers)):
            self.r_markers[i].destroy()
            self.l_markers[i].destroy()
        for i in self.r_links.keys():
            self.r_links[i].action = Marker.DELETE
            self.l_links[i].action = Marker.DELETE
        m_array = MarkerArray()
        for i in self.r_links.keys():
            m_array.markers.append(self.r_links[i])
        for i in self.l_links.keys():
            m_array.markers.append(self.l_links[i])
        self._marker_publisher.publish(m_array)
        self.r_markers = []
        self.l_markers = []
        self.r_links = dict()
        self.l_links = dict()
        self.lock.release()

    def marker_click_cb(self, uid, is_selected):
        '''Callback for when one of the markers is clicked.
        Goes over all markers and un-selects them'''
        for i in range(len(self.r_markers)):
            if (self.r_markers[i].get_uid() == uid):
                self.r_markers[i].is_control_visible = is_selected
                self.r_markers[i].update_viz()
            else:
                if (self.r_markers[i].is_control_visible):
                    self.r_markers[i].is_control_visible = False
                    self.r_markers[i].update_viz()

        for i in range(len(self.l_markers)):
            if (self.l_markers[i].get_uid() == uid):
                self.l_markers[i].is_control_visible = is_selected
                self.l_markers[i].update_viz()
            else:
                if (self.l_markers[i].is_control_visible):
                    self.l_markers[i].is_control_visible = False
                    self.l_markers[i].update_viz()

        if is_selected:
            self.step_click_cb(uid)

    def select_step(self, step_id):
        ''' Makes the interactive marker for the indicated action
        step selected, by showing the 6D controls'''
        self.marker_click_cb(step_id, True)

    def initialize_viz(self, object_list):
        '''Initialize visualization'''
        self.lock.acquire()
        action_objects = self._get_action_objects()
        action_objects_unique = self._get_unique_action_objects(action_objects)
        map_of_objects_old_to_new = World.get_map_of_most_similar_obj(action_objects_unique, object_list)
        for i in range(len(self.seq.seq)):
            step = self.seq.seq[i]
            if (step.type == ActionStep.ARM_TARGET or
                step.type == ActionStep.ARM_TRAJECTORY):
                r_marker = ActionStepMarker(i + 1, 0, step,
                                            self.marker_click_cb)
                l_marker = ActionStepMarker(i + 1, 1, step,
                                            self.marker_click_cb)
                r_new_object = None
                l_new_object = None
                if map_of_objects_old_to_new is not None:
                    r_old_object = action_objects[i][0]
                    if r_old_object is not None:
                        r_new_object = map_of_objects_old_to_new[r_old_object.name]
                    l_old_object = action_objects[i][1]
                    if l_old_object is not None:
                        l_new_object = map_of_objects_old_to_new[l_old_object.name]
                r_marker.update_ref_frames(object_list, r_new_object)
                l_marker.update_ref_frames(object_list, l_new_object)

                self.r_markers.append(r_marker)
                self.l_markers.append(l_marker)

                if (i > 0):
                    self.r_links[i] = self._get_link(0, i)
                    self.l_links[i] = self._get_link(1, i)

        self._update_markers()
        self.lock.release()

    def _get_action_objects(self):
        steps = self.seq.seq
        objects = []
        for i in range(len(steps)):
            step = steps[i]
            r_object = None
            l_object = None
            if step.armTarget.rArm.refFrame == ArmState.OBJECT:
                r_object = step.armTarget.rArm.refFrameObject
            if step.armTarget.lArm.refFrame == ArmState.OBJECT:
                l_object = step.armTarget.lArm.refFrameObject
            objects.append((r_object, l_object))
        return objects

    @staticmethod
    def _get_unique_action_objects(action_objects):
        objects = []
        object_names = []
        for r_object, l_object in action_objects:
            if r_object is not None and r_object.name not in object_names:
                object_names.append(r_object.name)
                objects.append(r_object)
            if l_object is not None and l_object.name not in object_names:
                object_names.append(l_object.name)
                objects.append(l_object)
        return objects

    def get_last_step(self):
        '''Returns the last step of the action'''
        self.lock.acquire()
        #If there are no steps yet, return None.
        if len(self.seq.seq) == 0:
            last_step = None
        else:
            last_step = self.seq.seq[len(self.seq.seq) - 1]
        self.lock.release()
        return last_step

    def delete_last_step(self):
        '''Deletes the last step of the action'''
        self.lock.acquire()
        self._delete_step(len(self.seq.seq) - 1)
        self.lock.release()

    def resume_deleted_step(self):
        '''Resumes the deleted last step'''
        self.seq.seq.append(None)

    def is_object_required(self):
        '''Checks if the action has steps that are relative to
        objects in the world'''
        is_required = False
        self.lock.acquire()
        for i in range(len(self.seq.seq)):
            if ((self.seq.seq[i].type == ActionStep.ARM_TARGET and
            (self.seq.seq[i].armTarget.rArm.refFrame == ArmState.OBJECT or
            self.seq.seq[i].armTarget.lArm.refFrame == ArmState.OBJECT)) or
            (self.seq.seq[i].type == ActionStep.ARM_TRAJECTORY and
            (self.seq.seq[i].armTrajectory.rRefFrame == ArmState.OBJECT or
            self.seq.seq[i].armTrajectory.lRefFrame == ArmState.OBJECT))):
                is_required = True
        self.lock.release()
        return is_required

    def get_step_gripper_state(self, arm_index, index):
        ''' Returns the gripper state of indicated action step,
        for the indicated side'''
        action_step = self.get_step(index)
        if arm_index == 0:
            return action_step.gripperAction.rGripper
        else:
            return action_step.gripperAction.rGripper

    def get_step_ref_frame(self, arm_index, index):
        ''' Returns the gripper state of indicated action step,
        for the indicated side'''
        action_step = self.get_step(index)
        if arm_index == 0:
            return action_step.armTarget.rArm.refFrameObject.name
        else:
            return action_step.armTarget.lArm.refFrameObject.name

    def get_step(self, index):
        '''Returns a step of the action'''
        self.lock.acquire()
        requested_step = self.seq.seq[index]
        self.lock.release()
        return requested_step

    def copy(self):
        '''Makes a copy of the instance'''
        action = ProgrammedAction(self.action_index, self.step_click_cb)
        action.seq = ActionStepSequence()
        for i in range(len(self.seq.seq)):
            action_step = self.seq.seq[i]
            copy = ProgrammedAction._copy_action_step(action_step)
            action.seq.seq.append(copy)
        return action

    @staticmethod
    def _copy_action_step(action_step):
        '''Makes a copy of an action step'''
        copy = ActionStep()
        copy.type = int(action_step.type)
        copy.baseTarget = action_step.baseTarget
        if (copy.type == ActionStep.ARM_TARGET):
            copy.armTarget = ArmTarget()
            copy.armTarget.rArmVelocity = float(
                                    action_step.armTarget.rArmVelocity)
            copy.armTarget.lArmVelocity = float(
                                    action_step.armTarget.lArmVelocity)
            copy.armTarget.rArm = ProgrammedAction._copy_arm_state(
                                                action_step.armTarget.rArm)
            copy.armTarget.lArm = ProgrammedAction._copy_arm_state(
                                                action_step.armTarget.lArm)
        elif (copy.type == ActionStep.ARM_TRAJECTORY):
            copy.armTrajectory = ArmTrajectory()
            copy.armTrajectory.timing = action_step.armTrajectory.timing[:]
            for j in range(len(action_step.armTrajectory.timing)):
                copy.armTrajectory.rArm.append(
                    ProgrammedAction._copy_arm_state(
                                        action_step.armTrajectory.rArm[j]))
                copy.armTrajectory.lArm.append(
                    ProgrammedAction._copy_arm_state(
                                        action_step.armTrajectory.lArm[j]))
            copy.armTrajectory.rRefFrame = int(
                    action_step.armTrajectory.rRefFrame)
            copy.armTrajectory.lRefFrame = int(
                    action_step.armTrajectory.lRefFrame)
            ## WARNING: the following is not really copying
            r_obj = action_step.armTrajectory.rRefFrameObject
            l_obj = action_step.armTrajectory.lRefFrameObject
            copy.armTrajectory.rRefFrameObject = r_obj
            copy.armTrajectory.lRefFrameObject = l_obj
        copy.gripperAction = GripperAction(action_step.gripperAction.rGripper,
                                           action_step.gripperAction.lGripper)
        copy.preCond = action_step.preCond
        copy.postCond = action_step.postCond
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

    def get_action_for_reversed_hands(self):
        '''Returns a new action that is like the original one except that all left-hand steps are assigned
            to the right hand and vice versa.'''
        reversed_action = self.copy()
        for i in range(len(self.seq.seq)):
            action_step = self.seq.seq[i]
            rev_action_step = reversed_action.seq.seq[i]
            if (action_step.type == ActionStep.ARM_TARGET):
                rev_action_step.armTarget.rArm = action_step.armTarget.lArm
                if rev_action_step.armTarget.rArm.refFrame == ArmState.ROBOT_BASE:
                    rev_action_step.armTarget.rArm = ProgrammedAction._mirror_ee_pose_and_joints(rev_action_step.armTarget.rArm)
                else:
                    rev_action_step.armTarget.rArm = ProgrammedAction._mirror_joints(rev_action_step.armTarget.rArm)
                rev_action_step.armTarget.lArm = action_step.armTarget.rArm
                if rev_action_step.armTarget.lArm.refFrame == ArmState.ROBOT_BASE:
                    rev_action_step.armTarget.lArm = ProgrammedAction._mirror_ee_pose_and_joints(rev_action_step.armTarget.lArm)
                else:
                    rev_action_step.armTarget.lArm = ProgrammedAction._mirror_joints(rev_action_step.armTarget.lArm)
                rev_action_step.armTarget.rArmVelocity = action_step.armTarget.lArmVelocity
                rev_action_step.armTarget.lArmVelocity = action_step.armTarget.rArmVelocity
            elif (action_step.type == ActionStep.ARM_TRAJECTORY):
                rev_action_step.armTrajectory.rArm = action_step.armTrajectory.lArm
                new_traj = []
                for state in rev_action_step.armTarget.rArm:
                    if rev_action_step.armTrajectory.rRefFrameObject == ArmState.ROBOT_BASE:
                        new_traj.append(ProgrammedAction._mirror_ee_pose_and_joints(state))
                    else:
                        new_traj.append(ProgrammedAction._mirror_joints(state))
                rev_action_step.armTarget.rArm = new_traj
                rev_action_step.armTrajectory.lArm = action_step.armTrajectory.rArm
                new_traj = []
                for state in rev_action_step.armTarget.lArm:
                    if rev_action_step.armTrajectory.lRefFrameObject == ArmState.ROBOT_BASE:
                        new_traj.append(ProgrammedAction._mirror_ee_pose_and_joints(state))
                    else:
                        new_traj.append(ProgrammedAction._mirror_joints(state))
                rev_action_step.armTarget.lArm = new_traj
                rev_action_step.armTrajectory.rRefFrame = action_step.armTrajectory.lRefFrame
                rev_action_step.armTrajectory.lRefFrame = action_step.armTrajectory.rRefFrame
                rev_action_step.armTrajectory.rRefFrameObject = action_step.armTrajectory.lRefFrameObject
                rev_action_step.armTrajectory.lRefFrameObject = action_step.armTrajectory.rRefFrameObject
            rev_action_step.gripperAction.rGripper = action_step.gripperAction.lGripper
            rev_action_step.gripperAction.lGripper = action_step.gripperAction.rGripper
            reversed_action.seq.seq[i] = rev_action_step
        return reversed_action

    @staticmethod
    def _mirror_joints(arm_state):
        ''' Create new arm state with reflected joints with respect to the x-z plane. '''
        new_state = ArmState()
        new_state.refFrame = int(arm_state.refFrame)
        new_state.refFrameObject = arm_state.refFrameObject
        new_state.ee_pose = arm_state.ee_pose
        new_state.joint_pose = list(arm_state.joint_pose[:])
        for i in range(len(new_state.joint_pose)):
            if i == 0 or i == 2 or i == 4:
                new_state.joint_pose[i] = -new_state.joint_pose[i]
        return new_state

    @staticmethod
    def _mirror_ee_pose_and_joints(arm_state):
        ''' Create new arm state with reflected ee pose and joints with respect to the x-z plane. '''
        new_state = ArmState()
        new_state.refFrame = int(arm_state.refFrame)
        new_state.refFrameObject = arm_state.refFrameObject
        new_state.joint_pose = list(arm_state.joint_pose[:])
        for i in range(len(new_state.joint_pose)):
            if i == 0 or i == 2 or i == 4:
                new_state.joint_pose[i] = -new_state.joint_pose[i]
        old_o = arm_state.ee_pose.orientation
        refl_m = np.array([[-1,0,0,0], [0,1,0,0], [0,0,-1,0], [0,0,0,1]])
        new_o = Quaternion(*np.dot(refl_m, np.transpose(np.array([old_o.x, old_o.y, old_o.z, old_o.w]))))
        old_p = arm_state.ee_pose.position
        new_p = Point(old_p.x, -old_p.y, old_p.z)
        new_state.ee_pose = Pose(new_p, new_o)
        return new_state


