'''Stuff related to a single marker for base steps of an action'''

import roslib
roslib.load_manifest('pr2_pbd_interaction')

import numpy
import rospy
import tf
from step_types.ArmStep import ArmStep
from pr2_pbd_interaction.msg import ArmState, Object, GripperState
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.menu_handler import MenuHandler
from Robot import Robot
from World import World


class BaseStepMarker:
    ''' Marker for visualizing the base steps of an action'''

    _marker_click_cb = None

    def __init__(self, action_step, marker_click_cb, im_server):
        self.action_step = action_step
        self.is_requested = False
        self.is_deleted = False
        self.is_control_visible = False
        self.is_edited = False

        self.im_server = im_server

        self._sub_entries = None
        self._menu_handler = None
        BaseStepMarker._marker_click_cb = marker_click_cb

    def _get_name(self):
        '''Generates the unique name for the marker'''
        return ('basestep')

    def destroy(self):
        '''Removes marker from the world'''
        self.im_server.erase(self._get_name())
        self.im_server.applyChanges()

    def _update_menu(self):
        '''Recreates the menu when something has changed'''
        self._menu_handler = MenuHandler()
        self._menu_handler.insert('Move base here', callback=self.move_to_cb)
        self._menu_handler.insert('Move to current base location',
                            callback=self.move_pose_to_cb)
        self._menu_handler.insert('Delete', callback=self.delete_step_cb)
        self._update_viz_core()
        self._menu_handler.apply(self.im_server, self._get_name())
        self.im_server.applyChanges()

    def set_new_pose(self, new_pose):
        '''Changes the pose of the base step'''
        self.action_step.end_pose = new_pose
        rospy.loginfo('Set new pose for base step.')

    def set_target_pose(self, target):
        '''Sets the new pose for the base step'''
        self.action_step.end_pose = target
        self._update_menu()
        self.is_edited = False

    def get_target_pose(self):
        '''Returns the base step'''
        return self.action_step.end_pose

    def _update_viz_core(self):
        '''Updates visualization after a change'''
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        pose = self.get_target_pose()

        menu_control = self._make_base_marker(menu_control)

        # Calculate text position so that they "orbit" around the marker;
        # this is done so that poses in identical or similar positions
        # have non-overlapping text. Note that to do this without moving
        # the text around as the camera is moved, we assume that the viewer
        # is always looking directly at the robot, so we assume the x dimension
        # is constant and "orbin" in the y-z plane.
        n_orbitals = 8 # this should be a constant
        offset = 0.15 # this should be a constant
        orbital = 0 # - 1 to make 0-based
        angle_rad = (float(orbital) / n_orbitals) * (-2 * numpy.pi) + \
            (numpy.pi / 2.0) # start above, at pi/2 (90 degrees)
        text_pos = Point()
        text_pos.x = pose.position.x
        text_pos.y = pose.position.y + numpy.cos(angle_rad) * offset
        text_pos.z = pose.position.z + numpy.sin(angle_rad) * offset
        r,g,b = 1.0, 0.5, 0.0
        menu_control.markers.append(Marker(type=Marker.TEXT_VIEW_FACING,
                        id=0, scale=Vector3(0, 0, 0.05),
                        text='Base step',
                        color=ColorRGBA(r, g, b, 1.0),
                        header=Header(frame_id='base_link'),
                        pose=Pose(text_pos, Quaternion(0, 0, 0, 1))))

        int_marker = InteractiveMarker()
        int_marker.name = self._get_name()
        int_marker.header.frame_id = 'base_link'
        int_marker.pose = pose
        int_marker.scale = 0.2
        self._add_3dof_marker(int_marker, True)

        int_marker.controls.append(menu_control)
        self.im_server.insert(int_marker,
                                           self.marker_feedback_cb)

    def marker_feedback_cb(self, feedback):
        '''Callback for when an event occurs on the marker'''
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.set_new_pose(feedback.pose)
            self.update_viz()
        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            # Set the visibility of the 6DOF controller
            rospy.loginfo('Changing visibility of the pose controls.')
            if (self.is_control_visible):
                self.is_control_visible = False
            else:
                self.is_control_visible = True
            BaseStepMarker._marker_click_cb(self.is_control_visible)
        else:
            rospy.loginfo('Unknown event' + str(feedback.event_type))

    def delete_step_cb(self, dummy):
        '''Callback for when delete is requested'''
        self.is_deleted = True

    def move_to_cb(self, dummy):
        '''Callback for when moving to a pose is requested'''
        self.is_requested = True

    def move_pose_to_cb(self, dummy):
        '''Callback for when a pose change to current is requested'''
        self.is_edited = True

    def pose_reached(self):
        '''Update when a requested pose is reached'''
        self.is_requested = False

    def update_viz(self):
        '''Updates visualization fully'''
        self._update_viz_core()
        self._menu_handler.reApply(self.im_server)
        self.im_server.applyChanges()

    def _add_3dof_marker(self, int_marker, is_fixed):
        '''Adds a 3 DoF control marker to the interactive marker'''
        control = self._make_6dof_control('rotate_x',
                        Quaternion(1, 0, 0, 1), False, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('move_x',
                        Quaternion(1, 0, 0, 1), True, is_fixed)
        int_marker.controls.append(control)
        control = self._make_6dof_control('move_y',
                        Quaternion(0, 0, 1, 1), True, is_fixed)
        int_marker.controls.append(control)

    def _make_6dof_control(self, name, orientation, is_move, is_fixed):
        '''Creates one component of the 6dof controller'''
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation = orientation
        control.always_visible = False
        if (self.is_control_visible):
            if is_move:
                control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            else:
                control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        else:
            control.interaction_mode = InteractiveMarkerControl.NONE
        if is_fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        return control

    def _make_mesh_marker(self):
        '''Creates a mesh marker'''
        mesh = Marker()
        mesh.mesh_use_embedded_materials = False
        mesh.type = Marker.MESH_RESOURCE
        mesh.scale.x = 1.0
        mesh.scale.y = 1.0
        mesh.scale.z = 1.0
        alpha = 0.6
        r,g,b = 1.0, 0.5, 0.0
        mesh.color = ColorRGBA(r, g, b, alpha)
        return mesh

    def _make_base_marker(self, control):
        '''Makes a base marker'''
        mesh1 = self._make_mesh_marker()
        mesh1.mesh_resource = ('package://pr2_description/meshes/' +
                                'base_v0/base.dae')
        mesh1.pose.position.x = 0
        mesh1.pose.orientation.w = 1
        control.markers.append(mesh1)

        return control
