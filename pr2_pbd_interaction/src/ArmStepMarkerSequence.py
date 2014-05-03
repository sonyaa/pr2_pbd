#!/usr/bin/env python
from geometry_msgs.msg import Point, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rospy
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from ArmStepMarker import ArmStepMarker


class ArmStepMarkerSequence:
    """ Class that visualizes the arm markers for one manipulation step.
    """

    def __init__(self, im_server, marker_publisher, step_click_cb):
        self.im_server = im_server
        self.marker_publisher = marker_publisher
        self.ref_object_list = None
        self.ref_names = None
        self.total_n_markers = 0
        self.r_markers = []
        self.l_markers = []
        self.r_links = dict()
        self.l_links = dict()
        self.step_click_cb = step_click_cb

    def add_arm_step(self, arm_step, last_step, world_objects):
        r_marker = ArmStepMarker(self.total_n_markers, 0,
                                 last_step, self.marker_click_cb, self)
        r_marker.update_ref_frames(world_objects, arm_step.armTarget.rArm.refFrameObject)
        l_marker = ArmStepMarker(self.total_n_markers, 1,
                                 last_step, self.marker_click_cb, self)
        l_marker.update_ref_frames(world_objects, arm_step.armTarget.lArm.refFrameObject)
        self.r_markers.append(r_marker)
        self.l_markers.append(l_marker)
        if (self.total_n_markers > 1):
            self.r_links[self.total_n_markers - 1] = self._get_link(0,
                                                                     self.total_n_markers - 1)
            self.l_links[self.total_n_markers - 1] = self._get_link(1,
                                                                     self.total_n_markers - 1)
        self._update_markers()

    def _get_link(self, arm_index, to_index):
        """Returns a marker representing a link b/w two
        consecutive arm steps"""
        # Grab the start/end point via the two markers
        markers = self.r_markers if arm_index == 0 else self.l_markers
        start = markers[to_index - 1].get_absolute_position(is_start=True)
        end = markers[to_index].get_absolute_position(is_start=False)

        # Scale the arrow
        scale = 0.8  # should be constant; shortening arrows to see ends
        diff = Point(end.x - start.x, end.y - start.y, end.z - start.z)
        diff_scaled = Point(diff.x * scale, diff.y * scale, diff.z * scale)
        new_start = Point(end.x - diff_scaled.x, end.y - diff_scaled.y,
                          end.z - diff_scaled.z)
        new_end = Point(start.x + diff_scaled.x, start.y + diff_scaled.y,
                        start.z + diff_scaled.z)

        return Marker(type=Marker.ARROW, id=(2 * to_index + arm_index),
                      lifetime=rospy.Duration(2),
                      scale=Vector3(0.01, 0.03, 0.07),
                      header=Header(frame_id='base_link'),
                      color=ColorRGBA(0.8, 0.8, 0.5, 0.4),
                      points=[new_start, new_end])

    def update_objects(self, action_objects, world_objects, map_of_objects_old_to_new):
        self._update_markers()
        for i in range(len(self.r_markers)):
            r_new_object = None
            if map_of_objects_old_to_new is not None:
                r_old_object = action_objects[2 * i]
                if r_old_object is not None:
                    r_new_object = map_of_objects_old_to_new[r_old_object.name]
            self.r_markers[i].update_ref_frames(world_objects, r_new_object)
        for i in range(len(self.l_markers)):
            l_new_object = None
            if map_of_objects_old_to_new is not None:
                l_old_object = action_objects[2 * i + 1]
                if l_old_object is not None:
                    l_new_object = map_of_objects_old_to_new[l_old_object.name]
            self.l_markers[i].update_ref_frames(world_objects, l_new_object)

    def _update_markers(self):
        """Updates the markers after a change"""
        for i in range(len(self.r_markers)):
            self.r_markers[i].update_viz()
        for i in range(len(self.l_markers)):
            self.l_markers[i].update_viz()

    def set_total_n_markers(self, total_n_markers):
        self.total_n_markers = total_n_markers

    def reset_viz(self):
        """Removes all visualization from RViz"""
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
        self.marker_publisher.publish(m_array)
        self.r_markers = []
        self.l_markers = []
        self.r_links = dict()
        self.l_links = dict()

    def _update_links(self):
        """Updates the visualized links b/w arm steps"""
        for i in self.r_links.keys():
            self.r_links[i] = self._get_link(0, i)
        for i in self.l_links.keys():
            self.l_links[i] = self._get_link(1, i)

    def update_viz(self):
        """Update the visualization of the action"""
        self._update_links()
        m_array = MarkerArray()
        for i in self.r_links.keys():
            m_array.markers.append(self.r_links[i])
        for i in self.l_links.keys():
            m_array.markers.append(self.l_links[i])
        self.marker_publisher.publish(m_array)

    def initialize_viz(self, steps, action_objects, world_objects, map_of_objects_old_to_new):
        """Initialize visualization"""
        self.set_total_n_markers(len(steps))
        for i in range(len(steps)):
            step = steps[i]
            r_marker = ArmStepMarker(i + 1, 0, step,
                                     self.marker_click_cb, self)
            l_marker = ArmStepMarker(i + 1, 1, step,
                                     self.marker_click_cb, self)
            r_new_object = None
            l_new_object = None
            if map_of_objects_old_to_new is not None:
                r_old_object = action_objects[2 * i]
                if r_old_object is not None:
                    r_new_object = map_of_objects_old_to_new[r_old_object.name]
                l_old_object = action_objects[2 * i + 1]
                if l_old_object is not None:
                    l_new_object = map_of_objects_old_to_new[l_old_object.name]
            r_marker.update_ref_frames(world_objects, r_new_object)
            l_marker.update_ref_frames(world_objects, l_new_object)

            self.r_markers.append(r_marker)
            self.l_markers.append(l_marker)

            if (i > 0):
                self.r_links[i] = self._get_link(0, i)
                self.l_links[i] = self._get_link(1, i)

        self._update_markers()

    def marker_click_cb(self, uid, is_selected):
        """Callback for when one of the markers is clicked.
        Goes over all markers and un-selects them"""
        for i in range(len(self.r_markers)):
            if (self.r_markers[i].get_uid() == uid):
                self.r_markers[i].is_dimmed = False
                self.r_markers[i].is_control_visible = is_selected
                self.r_markers[i].update_viz()
            else:
                if (self.r_markers[i].is_control_visible):
                    self.r_markers[i].is_control_visible = False
                    self.r_markers[i].update_viz()
                # If a new marker is selected, dim all the others.
                if (is_selected == True):
                    self.r_markers[i].is_dimmed = True
                    self.r_markers[i].update_viz()
                # Otherwise, it's deselected: undim all the others if they were dimmed.
                elif (self.r_markers[i].is_dimmed):
                    self.r_markers[i].is_dimmed = False
                    self.r_markers[i].update_viz()

        for i in range(len(self.l_markers)):
            if (self.l_markers[i].get_uid() == uid):
                self.l_markers[i].is_dimmed = False
                self.l_markers[i].is_control_visible = is_selected
                self.l_markers[i].update_viz()
            else:
                if (self.l_markers[i].is_control_visible):
                    self.l_markers[i].is_control_visible = False
                    self.l_markers[i].update_viz()
                # If a new marker is selected, dim all the others.
                if (is_selected == True):
                    self.l_markers[i].is_dimmed = True
                    self.l_markers[i].update_viz()
                # Otherwise, it's deselected: undim all the others if they were dimmed.
                elif (self.l_markers[i].is_dimmed):
                    self.l_markers[i].is_dimmed = False
                    self.l_markers[i].update_viz()

        if is_selected:
            self.step_click_cb(uid)


    def select_step(self, step_id):
        """ Makes the interactive marker for the indicated arm
        step selected, by showing the 6D controls"""
        self.marker_click_cb(step_id, True)

    def deselect_step(self, step_id):
        """ Makes the interactive marker for the indicated arm
        step deselected, by removing the 6D controls"""
        self.marker_click_cb(step_id, False)