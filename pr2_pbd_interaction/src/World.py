'''Everything related to perception of the world'''
import roslib

roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import time
import threading
from numpy.linalg import norm
from numpy import array, dot, cross

# ROS libraries
#from actionlib_msgs.msg import
import rospy
import tf
from tf import TransformListener, TransformBroadcaster
from manipulation_msgs.msg import GraspableObjectList
#from object_manipulation_msgs.msg import GraspableObjectList
from object_manipulation_msgs.srv import FindClusterBoundingBox
from pr2_interactive_object_detection.msg import UserCommandAction
from pr2_interactive_object_detection.msg import UserCommandGoal
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, PoseStamped
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from actionlib_msgs.msg import GoalStatus
import actionlib
from math import pi, sin, cos
from ar_track_alvar.msg import AlvarMarkers

# Local stuff
from pr2_pbd_interaction.msg import Object, ArmState
from pr2_social_gaze.msg import GazeGoal
from Response import Response


class WorldObject:
    '''Class for representing objects'''

    default_color = ColorRGBA(0.2, 0.8, 0.0, 0.6)
    fake_color = ColorRGBA(0.4, 0.6, 0.4, 0.6)
    selected_colors = [ColorRGBA(0.6, 0.6, 0.0, 0.6), ColorRGBA(0.6, 0.6, 0.2, 0.6)]

    def __init__(self, pose, index, dimensions, is_recognized, is_fake=False):
        ''' Initialization of objects'''
        self.index = index
        self.assigned_name = None
        self.is_recognized = is_recognized
        self.object = Object(Object.TABLE_TOP, self.get_name(),
                             pose, dimensions)
        self.menu_handler = MenuHandler()
        self.int_marker = None
        self.is_removed = False
        self.menu_handler.insert('Remove from scene', callback=self.remove)
        self.is_marker = False
        self.is_fake = is_fake

    def remove(self, dummy):
        '''Function for removing object from the world'''
        rospy.loginfo('Will remove object' + self.get_name())
        self.is_removed = True

    def assign_name(self, name):
        '''Function for assigning a different name'''
        self.assigned_name = name
        self.object.name = name

    def get_name(self):
        '''Function to get the object name'''
        if (self.assigned_name == None):
            if (self.is_recognized):
                return 'object' + str(self.index)
            else:
                return 'thing' + str(self.index)
        else:
            return self.assigned_name

    def decrease_index(self):
        '''Function to decrese object index'''
        self.index -= 1


class World:
    '''Object recognition and localization related stuff'''

    tf_listener = None
    objects = []
    world = None

    def __init__(self):
        if World.tf_listener == None:
            World.tf_listener = TransformListener()
        self._lock = threading.Lock()
        self.surface = None
        self._tf_broadcaster = TransformBroadcaster()
        self._im_server = InteractiveMarkerServer('world_objects')
        bb_service_name = 'find_cluster_bounding_box'
        rospy.wait_for_service(bb_service_name)
        self._bb_service = rospy.ServiceProxy(bb_service_name,
                                              FindClusterBoundingBox)
        rospy.Subscriber('interactive_object_recognition_result',
                         GraspableObjectList, self.receieve_object_info)
        self._object_action_client = actionlib.SimpleActionClient(
            'object_detection_user_command', UserCommandAction)
        self._object_action_client.wait_for_server()
        rospy.loginfo('Interactive object detection action ' +
                      'server has responded.')
        self.clear_all_objects()
        # The following is to get the table information
        rospy.Subscriber('tabletop_segmentation_markers',
                         Marker, self.receieve_table_marker)
        self.relative_frame_threshold = 0.4
        rospy.Subscriber("ar_pose_marker",
                         AlvarMarkers, self.receive_ar_markers)
        self.is_looking_for_markers = False
        self.marker_dims = Vector3(0.04, 0.04, 0.01)
        World.world = self

    @staticmethod
    def get_world():
        if World.world is None:
            World.world = World()
        return World.world

    def _reset_objects(self):
        '''Function that removes all objects'''
        self._lock.acquire()
        for i in range(len(World.objects)):
            self._im_server.erase(World.objects[i].int_marker.name)
            self._im_server.applyChanges()
        if self.surface != None:
            self._remove_surface()
        #self._im_server.clear()
        self._im_server.applyChanges()
        World.objects = []
        self._lock.release()

    def receieve_table_marker(self, marker):
        '''Callback function for markers to determine table'''
        if (marker.type == Marker.LINE_STRIP):
            if (len(marker.points) == 6):
                rospy.loginfo('Received a TABLE marker.')
                xmin = marker.points[0].x
                ymin = marker.points[0].y
                xmax = marker.points[2].x
                ymax = marker.points[2].y
                depth = xmax - xmin
                width = ymax - ymin

                pose = Pose(marker.pose.position, marker.pose.orientation)
                pose.position.x = pose.position.x + xmin + depth / 2
                pose.position.y = pose.position.y + ymin + width / 2
                dimensions = Vector3(depth, width, 0.01)
                self.surface = World._get_surface_marker(pose, dimensions)
                self._im_server.insert(self.surface,
                                       self.marker_feedback_cb)
                self._im_server.applyChanges()

    def receive_ar_markers(self, data):
        '''Callback function to receive marker info'''
        self._lock.acquire()
        if self.is_looking_for_markers:
            if len(data.markers) > 0:
                rospy.loginfo('Received non-empty marker list.')
                for i in range(len(data.markers)):
                    marker = data.markers[i]
                    self._add_new_object(marker.pose.pose, self.marker_dims, False, id=marker.id)
        self._lock.release()

    def receieve_object_info(self, object_list):
        '''Callback function to receive object info'''
        self._lock.acquire()
        rospy.loginfo('Received recognized object list.')
        if (len(object_list.graspable_objects) > 0):
            for i in range(len(object_list.graspable_objects)):
                models = object_list.graspable_objects[i].potential_models
                if (len(models) > 0):
                    object_pose = None
                    best_confidence = 0.0
                    for j in range(len(models)):
                        if (best_confidence < models[j].confidence):
                            object_pose = models[j].pose.pose
                            best_confidence = models[j].confidence
                    if (object_pose != None):
                        rospy.logwarn('Adding the recognized object ' +
                                      'with most confident model.')
                        self._add_new_object(object_pose,
                                             Vector3(0.2, 0.2, 0.2), True,
                                             object_list.meshes[i])
                else:
                    rospy.logwarn('... this is not a recognition result, ' +
                                  'it is probably just segmentation.')
                    cluster = object_list.graspable_objects[i].cluster
                    bbox = self._bb_service(cluster)
                    cluster_pose = bbox.pose.pose
                    if (cluster_pose != None):
                        rospy.loginfo('Adding unrecognized object with pose:' +
                                      World.pose_to_string(cluster_pose) + '\n' +
                                      'In ref frame' + str(bbox.pose.header.frame_id))
                        self._add_new_object(cluster_pose, bbox.box_dims,
                                             False)
        else:
            rospy.logwarn('... but the list was empty.')
        self._lock.release()

    @staticmethod
    def get_pose_from_transform(transform):
        '''Returns pose for transformation matrix'''
        pos = transform[:3, 3].copy()
        rot = tf.transformations.quaternion_from_matrix(transform)
        return Pose(Point(pos[0], pos[1], pos[2]),
                    Quaternion(rot[0], rot[1], rot[2], rot[3]))

    @staticmethod
    def get_matrix_from_pose(pose):
        '''Returns the transformation matrix for given pose'''
        rotation = [pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w]
        transformation = tf.transformations.quaternion_matrix(rotation)
        position = [pose.position.x, pose.position.y, pose.position.z]
        transformation[:3, 3] = position
        return transformation

    @staticmethod
    def get_absolute_pose(arm_state):
        '''Returns absolute pose of an end effector state'''
        if (arm_state.refFrame == ArmState.OBJECT):
            arm_state_copy = ArmState(arm_state.refFrame,
                                      Pose(arm_state.ee_pose.position,
                                           arm_state.ee_pose.orientation),
                                      arm_state.joint_pose[:],
                                      arm_state.refFrameObject)
            World.convert_ref_frame(arm_state_copy, ArmState.ROBOT_BASE)
            return arm_state_copy.ee_pose
        else:
            return arm_state.ee_pose

    @staticmethod
    def get_most_similar_obj(ref_object, ref_frame_list, threshold=0.075):
        '''Finds the most similar object in the world'''
        best_dist = 10000
        chosen_obj_index = -1
        for i in range(len(ref_frame_list)):
            dist = World.object_dissimilarity(ref_frame_list[i], ref_object)
            if (dist < best_dist):
                best_dist = dist
                chosen_obj_index = i
        if chosen_obj_index == -1:
            rospy.logwarn('Did not find a similar object..')
            return None
        else:
            print 'Object dissimilarity is --- ', best_dist
            if best_dist > threshold:
                rospy.logwarn('Found some objects, but not similar enough.')
                return None
            else:
                rospy.loginfo('Most similar to object '
                              + ref_frame_list[chosen_obj_index].name)
                return ref_frame_list[chosen_obj_index]

    @staticmethod
    def get_map_of_most_similar_obj(object_list, ref_frame_list, threshold=0.075):
        if None in object_list:
            object_list.remove(None)
        if len(object_list) == 0 or len(ref_frame_list) == 0:
            return None
        markers_dict = {}
        marker_names = []
        for obj in object_list:
            object_name = obj.name
            if object_name.startswith("marker"):
                marker_names.append(object_name)
                found_match = False
                for ref_frame in ref_frame_list:
                    if ref_frame.name == object_name:
                        markers_dict[object_name] = ref_frame
                        found_match = True
                        break
                if not found_match:
                    return None
        ref_frame_list = [x for x in ref_frame_list if x.name not in marker_names]
        object_list = [x for x in object_list if x.name not in marker_names]
        if len(object_list) == 0 or len(ref_frame_list) == 0:
            return markers_dict if len(markers_dict) > 0 else None
        orderings = []
        World.permute(object_list, orderings)
        costs = []
        assignments = []
        for ordering in orderings:
            cur_cost = 0
            cur_ref_frame_list = ref_frame_list[:]
            cur_assignment = []
            for object in ordering:
                most_similar_object = World.get_most_similar_obj(object, cur_ref_frame_list, threshold)
                if most_similar_object is None:
                    return None
                cur_ref_frame_list.remove(most_similar_object)
                cur_assignment.append(most_similar_object)
                cur_cost += World.object_dissimilarity(object, most_similar_object)
            costs.append(cur_cost)
            assignments.append(cur_assignment)
        min_cost, min_idx = min((val, idx) for (idx, val) in enumerate(costs))
        names = [x.name for x in orderings[min_idx]]
        object_dict = dict(zip(names, assignments[min_idx]))
        object_dict.update(markers_dict)
        return object_dict

    @staticmethod
    def permute(a, results):
        if len(a) == 1:
            results.insert(len(results), a)

        else:
            for i in range(0, len(a)):
                element = a[i]
                a_copy = [a[j] for j in range(0, len(a)) if j != i]
                subresults = []
                World.permute(a_copy, subresults)
                for subresult in subresults:
                    result = [element] + subresult
                    results.insert(len(results), result)

    @staticmethod
    def _get_mesh_marker(marker, mesh):
        '''Function that generated a marker from a mesh'''
        marker.type = Marker.TRIANGLE_LIST
        index = 0
        marker.scale = Vector3(1.0, 1.0, 1.0)
        while (index + 2 < len(mesh.triangles)):
            if ((mesh.triangles[index] < len(mesh.vertices))
                and (mesh.triangles[index + 1] < len(mesh.vertices))
                and (mesh.triangles[index + 2] < len(mesh.vertices))):
                marker.points.append(mesh.vertices[mesh.triangles[index]])
                marker.points.append(mesh.vertices[mesh.triangles[index + 1]])
                marker.points.append(mesh.vertices[mesh.triangles[index + 2]])
                index += 3
            else:
                rospy.logerr('Mesh contains invalid triangle!')
                break
        return marker

    def _add_new_object(self, pose, dimensions, is_recognized, mesh=None, id=None):
        '''Function to add new objects'''
        dist_threshold = 0.02
        to_remove = None
        if (is_recognized):
            # Temporary HACK for testing.
            # Will remove all recognition completely if this works.
            return False
            # Check if there is already an object
            for i in range(len(World.objects)):
                distance = World.pose_distance(World.objects[i].object.pose,
                                               pose)
                if (distance < dist_threshold):
                    if (World.objects[i].is_recognized):
                        rospy.loginfo('Previously recognized object at ' +
                                      'the same location, will not add this object.')
                        return False
                    else:
                        rospy.loginfo('Previously unrecognized object at ' +
                                      'the same location, will replace it with the ' +
                                      'recognized object.')
                        to_remove = i
                        break

            if (to_remove != None):
                self._remove_object(to_remove)

            n_objects = len(World.objects)
            new_object = WorldObject(pose, n_objects,
                                     dimensions, is_recognized)
            if id is not None:
                new_object.assign_name("marker" + str(id))
                new_object.is_marker = True
            World.objects.append(new_object)
            int_marker = self._get_object_marker(len(World.objects) - 1, mesh)
            World.objects[-1].int_marker = int_marker
            self._im_server.insert(int_marker, self.marker_feedback_cb)
            self._im_server.applyChanges()
            World.objects[-1].menu_handler.apply(self._im_server,
                                                 int_marker.name)
            self._im_server.applyChanges()
            return True
        else:
            for i in range(len(World.objects)):
                if (World.pose_distance(World.objects[i].object.pose, pose)
                        < dist_threshold):
                    rospy.loginfo('Previously detected object at the same' +
                                  'location, will not add this object.')
                    return False
                if id is not None and World.objects[i].get_name() == "marker" + str(id):
                    rospy.loginfo('Previously added marker with the same id, will not add this object.')
                    return False
            n_objects = len(World.objects)
            new_object = WorldObject(pose, n_objects,
                                     dimensions, is_recognized)
            if id is not None:
                new_object.assign_name("marker" + str(id))
                new_object.is_marker = True
            World.objects.append(new_object)
            int_marker = self._get_object_marker(len(World.objects) - 1)
            World.objects[-1].int_marker = int_marker
            self._im_server.insert(int_marker, self.marker_feedback_cb)
            self._im_server.applyChanges()
            World.objects[-1].menu_handler.apply(self._im_server,
                                                 int_marker.name)
            self._im_server.applyChanges()
            return True

    def _add_new_marker(self, id, pose):
        '''Function to add new markers'''
        #dist_threshold = 0.01
        #to_remove = None
        #for i in range(len(World.markers)):
        #    if (World.pose_distance(World.markers[i].pose, pose)
        #            < dist_threshold):
        #        rospy.loginfo('Previously detected marker at the same' +
        #                      'location, will not add this marker.')
        #        return False
        #n_markers = len(World.markers)
        #World.markers.append(WorldMarker(id, pose))
        #int_marker = self._get_object_marker(len(World.objects) - 1)
        #World.markers[-1].int_marker = int_marker
        #self._im_server.insert(int_marker, self.marker_feedback_cb)
        #self._im_server.applyChanges()
        #World.markers[-1].menu_handler.apply(self._im_server,
        #                                   int_marker.name)
        #self._im_server.applyChanges()
        return True

    def _remove_object(self, to_remove):
        '''Function to remove object by index'''
        obj = World.objects.pop(to_remove)
        rospy.loginfo('Removing object ' + obj.int_marker.name)
        self._im_server.erase(obj.int_marker.name)
        self._im_server.applyChanges()

    #        if (obj.is_recognized):
    #            for i in range(len(World.objects)):
    #                if ((World.objects[i].is_recognized)
    #                    and World.objects[i].index>obj.index):
    #                    World.objects[i].decrease_index()
    #            self.n_recognized -= 1
    #        else:
    #            for i in range(len(World.objects)):
    #                if ((not World.objects[i].is_recognized) and
    #                    World.objects[i].index>obj.index):
    #                    World.objects[i].decrease_index()
    #            self.n_unrecognized -= 1

    def _remove_surface(self):
        '''Function to request removing surface'''
        rospy.loginfo('Removing surface')
        self._im_server.erase('surface')
        self._im_server.applyChanges()
        self.surface = None


    def _get_object_reachability_marker(self, world_object):
        radius = self.relative_frame_threshold
        pointsList = []
        nx = 8
        ny = 8
        pointsList.append(Point(0, 0, radius))
        pointsList.append(Point(0, 0, -radius))
        for x in range(nx):
            theta = 2 * pi * (x * 1.0 / nx)
            for y in range(1, ny):
                phi = pi * (y * 1.0 / ny)
                destx = radius * cos(theta) * sin(phi)
                desty = radius * sin(theta) * sin(phi)
                destz = radius * cos(phi)
                pointsList.append(Point(destx, desty, destz))
        id = abs(hash(world_object.get_name() + "_reachability")) % (10 ** 8)
        marker = Marker(type=Marker.SPHERE_LIST, id=id,
                        lifetime=rospy.Duration(nsecs=10 ** 8),
                        scale=Vector3(0.01, 0.01, 0.01),
                        points=set(pointsList),
                        header=Header(frame_id='base_link'),
                        color=ColorRGBA(1, 1, 1, 0.5),
                        pose=world_object.object.pose)
        return marker

    def _get_object_marker(self, index, mesh=None):
        '''Generate a marker for world objects'''
        int_marker = InteractiveMarker()
        int_marker.name = World.objects[index].get_name()
        int_marker.header.frame_id = 'base_link'
        int_marker.pose = World.objects[index].object.pose
        int_marker.scale = 1

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True

        color = WorldObject.default_color if not World.objects[index].is_fake else WorldObject.fake_color
        object_marker = Marker(type=Marker.CUBE, id=index,
                               lifetime=rospy.Duration(2),
                               scale=World.objects[index].object.dimensions,
                               header=Header(frame_id='base_link'),
                               color=color,
                               pose=World.objects[index].object.pose)
        if (mesh != None):
            object_marker = World._get_mesh_marker(object_marker, mesh)
        button_control.markers.append(object_marker)

        text_pos = Point()
        text_pos.x = World.objects[index].object.pose.position.x
        text_pos.y = World.objects[index].object.pose.position.y
        text_pos.z = (World.objects[index].object.pose.position.z +
                      World.objects[index].object.dimensions.z / 2 + 0.06)
        button_control.markers.append(Marker(type=Marker.TEXT_VIEW_FACING,
                                             id=index, scale=Vector3(0, 0, 0.03),
                                             text=int_marker.name, color=ColorRGBA(0.0, 0.0, 0.0, 0.5),
                                             header=Header(frame_id='base_link'),
                                             pose=Pose(text_pos, Quaternion(0, 0, 0, 1))))
        int_marker.controls.append(button_control)
        return int_marker

    @staticmethod
    def _get_surface_marker(pose, dimensions):
        ''' Function that generates a surface marker'''
        int_marker = InteractiveMarker()
        int_marker.name = 'surface'
        int_marker.header.frame_id = 'base_link'
        int_marker.pose = pose
        int_marker.scale = 1
        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        object_marker = Marker(type=Marker.CUBE, id=2000,
                               lifetime=rospy.Duration(2),
                               scale=dimensions,
                               header=Header(frame_id='base_link'),
                               color=ColorRGBA(0.8, 0.0, 0.4, 0.4),
                               pose=pose)
        button_control.markers.append(object_marker)
        text_pos = Point()
        position = pose.position
        dimensions = dimensions
        text_pos.x = position.x + dimensions.x / 2 - 0.06
        text_pos.y = position.y - dimensions.y / 2 + 0.06
        text_pos.z = position.z + dimensions.z / 2 + 0.06
        text_marker = Marker(type=Marker.TEXT_VIEW_FACING, id=2001,
                             scale=Vector3(0, 0, 0.03), text=int_marker.name,
                             color=ColorRGBA(0.0, 0.0, 0.0, 0.5),
                             header=Header(frame_id='base_link'),
                             pose=Pose(text_pos, Quaternion(0, 0, 0, 1)))
        button_control.markers.append(text_marker)
        int_marker.controls.append(button_control)
        return int_marker

    def get_frame_list(self):
        '''Function that returns the list of ref. frames'''
        self._lock.acquire()
        objects = []
        for i in range(len(World.objects)):
            objects.append(World.objects[i].object)
        self._lock.release()
        return objects

    @staticmethod
    def has_objects():
        '''Function that checks if there are any objects'''
        return len(World.objects) > 0

    @staticmethod
    def has_marker_objects():
        '''Function that checks if there are any markers'''
        for o in World.objects:
            if o.is_marker:
                return True
        return False

    @staticmethod
    def has_non_marker_objects():
        '''Function that checks if there are any objects'''
        for o in World.objects:
            if not o.is_marker:
                return True
        return False

    @staticmethod
    def object_dissimilarity(obj1, obj2):
        '''Distance between two objects'''
        dims1 = obj1.dimensions
        dims2 = obj2.dimensions
        return norm(array([dims1.x, dims1.y, dims1.z]) -
                    array([dims2.x, dims2.y, dims2.z]))

    @staticmethod
    def get_ref_from_name(ref_name):
        '''Ref. frame type from ref. frame name'''
        if ref_name == 'base_link':
            return ArmState.ROBOT_BASE
        else:
            return ArmState.OBJECT

    @staticmethod
    def convert_ref_frame(arm_frame, ref_frame, ref_frame_obj=Object()):
        '''Transforms an arm frame to a new ref. frame'''
        if ref_frame == ArmState.ROBOT_BASE:
            if (arm_frame.refFrame == ArmState.ROBOT_BASE):
                rospy.logwarn('No reference frame transformations ' +
                              'needed (both absolute).')
            elif (arm_frame.refFrame == ArmState.OBJECT):
                abs_ee_pose = World.transform(arm_frame.ee_pose,
                                              arm_frame.refFrameObject.name, 'base_link')
                arm_frame.ee_pose = abs_ee_pose
                arm_frame.refFrame = ArmState.ROBOT_BASE
                arm_frame.refFrameObject = Object()
            else:
                rospy.logerr('Unhandled reference frame conversion:' +
                             str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        elif ref_frame == ArmState.OBJECT:
            if (arm_frame.refFrame == ArmState.ROBOT_BASE):
                rel_ee_pose = World.transform(arm_frame.ee_pose,
                                              'base_link', ref_frame_obj.name)
                arm_frame.ee_pose = rel_ee_pose
                arm_frame.refFrame = ArmState.OBJECT
                arm_frame.refFrameObject = ref_frame_obj
            elif (arm_frame.refFrame == ArmState.OBJECT):
                if (arm_frame.refFrameObject.name == ref_frame_obj.name):
                    rospy.logwarn('No reference frame transformations ' +
                                  'needed (same object).')
                else:
                    rel_ee_pose = World.transform(arm_frame.ee_pose,
                                                  arm_frame.refFrameObject.name, ref_frame_obj.name)
                    arm_frame.ee_pose = rel_ee_pose
                    arm_frame.refFrame = ArmState.OBJECT
                    arm_frame.refFrameObject = ref_frame_obj
            else:
                rospy.logerr('Unhandled reference frame conversion:' +
                             str(arm_frame.refFrame) + ' to ' + str(ref_frame))
        return arm_frame

    @staticmethod
    def has_object(object_name):
        '''Checks if the world contains the object'''
        for obj in World.objects:
            if obj.object.name == object_name:
                return True
        return False

    @staticmethod
    def is_frame_valid(object_name):
        '''Checks if the frame is valid for transforms'''
        return (object_name == 'base_link') or World.has_object(object_name)

    @staticmethod
    def transform(pose, from_frame, to_frame):
        ''' Function to transform a pose between two ref. frames
        if there is a TF exception or object does not exist it
        will return the pose back without any transforms'''
        if World.is_frame_valid(from_frame) and World.is_frame_valid(to_frame):
            pose_stamped = PoseStamped()
            try:
                common_time = World.tf_listener.getLatestCommonTime(from_frame,
                                                                    to_frame)
                pose_stamped.header.stamp = common_time
                pose_stamped.header.frame_id = from_frame
                pose_stamped.pose = pose
                rel_ee_pose = World.tf_listener.transformPose(to_frame,
                                                              pose_stamped)
                return rel_ee_pose.pose
            except tf.Exception:
                rospy.logerr('TF exception during transform.')
                return pose
            except rospy.ServiceException:
                rospy.logerr('Exception during transform.')
                return pose
        else:
            # rospy.logwarn('One of the frame objects might not exist: ' +
            #               from_frame + ' or ' + to_frame)
            return pose

    @staticmethod
    def pose_to_string(pose):
        '''For printing a pose to stdout'''
        return ('Position: ' + str(pose.position.x) + ", " +
                str(pose.position.y) + ', ' + str(pose.position.z) + '\n' +
                'Orientation: ' + str(pose.orientation.x) + ", " +
                str(pose.orientation.y) + ', ' + str(pose.orientation.z) +
                ', ' + str(pose.orientation.w) + '\n')

    def _publish_tf_pose(self, pose, name, parent):
        ''' Publishes a TF for object pose'''
        if (pose != None):
            pos = (pose.position.x, pose.position.y, pose.position.z)
            rot = (pose.orientation.x, pose.orientation.y,
                   pose.orientation.z, pose.orientation.w)
            self._tf_broadcaster.sendTransform(pos, rot,
                                               rospy.Time.now(), name, parent)

    def update_object_pose(self):
        ''' Function to externally update an object pose'''
        rospy.loginfo('Looking for objects now.')
        goal = UserCommandGoal(UserCommandGoal.RESET, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
                       self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo(Response.gaze_client.get_state())
        rospy.loginfo('Object recognition has been reset.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())
        self._reset_objects()

        if (self._object_action_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logerr('Could not reset recognition.')
            return False

        self.is_looking_for_markers = True
        rospy.loginfo('Looking for markers...')
        # Do segmentation
        goal = UserCommandGoal(UserCommandGoal.SEGMENT, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
                       self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Table segmentation is complete.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())

        if self._object_action_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr('Could not segment.')
            self.is_looking_for_markers = False
            if World.has_marker_objects():
                rospy.loginfo("But found some markers, will return that")
                return True
            return False

        # Do recognition
        goal = UserCommandGoal(UserCommandGoal.RECOGNIZE, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
                       self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Objects on the table have been recognized.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())

        # Record the result
        if (self._object_action_client.get_state() == GoalStatus.SUCCEEDED):
            wait_time = 0
            total_wait_time = 5
            while not World.has_non_marker_objects() and wait_time < total_wait_time:
                time.sleep(0.1)
                wait_time += 0.1

            self.is_looking_for_markers = False
            if (not World.has_objects()):
                rospy.logerr('Timeout waiting for a recognition result.')
                return False
            else:
                rospy.loginfo('Got the object list.')
                return True
        else:
            rospy.logerr('Could not recognize.')
            self.is_looking_for_markers = False
            if World.has_marker_objects():
                rospy.loginfo("But found some markers, will return that")
                return True
            return False

    def clear_all_objects(self):
        '''Removes all objects from the world'''
        goal = UserCommandGoal(UserCommandGoal.RESET, False)
        self._object_action_client.send_goal(goal)
        while (self._object_action_client.get_state() == GoalStatus.ACTIVE or
                       self._object_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.1)
        rospy.loginfo('Object recognition has been reset.')
        rospy.loginfo('STATUS: ' +
                      self._object_action_client.get_goal_status_text())
        if (self._object_action_client.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo('Successfully reset object localization pipeline.')
            self._reset_objects()
        self._remove_surface()

    def process_nearest_objects(self, nearest_object_pair):
        for i in range(len(World.objects)):
            old_color = World.objects[i].int_marker.controls[0].markers[0].color
            new_color = WorldObject.default_color if not World.objects[i].is_fake else WorldObject.fake_color
            for arm_index in [0, 1]:
                if World.objects[i].object == nearest_object_pair[arm_index]:
                    new_color = WorldObject.selected_colors[arm_index]
            if new_color != old_color:
                World.objects[i].int_marker.controls[0].markers[0].color = new_color
                self._im_server.insert(World.objects[i].int_marker, self.marker_feedback_cb)
                self._im_server.applyChanges()
                rospy.loginfo('Changed color of an object')


    def get_nearest_object(self, arm_pose):
        '''Gives a pointed to the nearest object'''
        distances = []
        for i in range(len(World.objects)):
            dist = World.pose_distance(World.objects[i].object.pose,
                                       arm_pose)
            distances.append(dist)
        dist_threshold = self.relative_frame_threshold
        if (len(distances) > 0):
            if (min(distances) < dist_threshold):
                chosen = distances.index(min(distances))
                return World.objects[chosen].object
            else:
                return None
        else:
            return None

    def add_fake_objects(self, objects):
        self._lock.acquire()
        for obj in objects:
            n_objects = len(World.objects)
            new_object = WorldObject(obj.pose, n_objects,
                                     obj.dimensions, False, is_fake=True)
            new_object.assign_name(obj.name)
            World.objects.append(new_object)
            int_marker = self._get_object_marker(len(World.objects) - 1)
            World.objects[-1].int_marker = int_marker
            self._im_server.insert(int_marker, self.marker_feedback_cb)
            self._im_server.applyChanges()
            World.objects[-1].menu_handler.apply(self._im_server,
                                                 int_marker.name)
            self._im_server.applyChanges()
        self._lock.release()

    def remove_fake_objects(self):
        self._lock.acquire()
        to_remove = []
        for i in range(len(World.objects)):
            if World.objects[i].is_fake:
                self._im_server.erase(World.objects[i].int_marker.name)
                self._im_server.applyChanges()
                to_remove.append(i)
        to_remove.reverse()
        for i in range(len(to_remove)):
            World.objects.pop(i)
        self._lock.release()

    @staticmethod
    def pose_distance(pose1, pose2, is_on_table=False):
        '''Distance between two world poses'''
        if pose1 == [] or pose2 == []:
            return 0.0
        else:
            if (is_on_table):
                arr1 = array([pose1.position.x, pose1.position.y])
                arr2 = array([pose2.position.x, pose2.position.y])
            else:
                arr1 = array([pose1.position.x,
                              pose1.position.y, pose1.position.z])
                arr2 = array([pose2.position.x,
                              pose2.position.y, pose2.position.z])
            dist = norm(arr1 - arr2)
            if dist < 0.0001:
                dist = 0
            return dist

    def marker_feedback_cb(self, feedback):
        '''Callback for when feedback from a marker is received'''
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo('Clicked on object ' + str(feedback.marker_name))
            rospy.loginfo('Number of objects ' + str(len(World.objects)))
        else:
            rospy.loginfo('Unknown event' + str(feedback.event_type))

    def update(self):
        '''Update function called in a loop'''
        # Visualize the detected object
        is_world_changed = False
        self._lock.acquire()
        if (World.has_objects()):
            to_remove = None
            for i in range(len(World.objects)):
                self._publish_tf_pose(World.objects[i].object.pose,
                                      World.objects[i].get_name(), 'base_link')
                if (World.objects[i].is_removed):
                    to_remove = i
            if to_remove != None:
                self._remove_object(to_remove)
                is_world_changed = True

        self._lock.release()
        return is_world_changed
