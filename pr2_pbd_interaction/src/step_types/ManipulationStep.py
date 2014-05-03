#!/usr/bin/env python
import threading
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import time
from visualization_msgs.msg import MarkerArray
from ArmStepMarkerSequence import ArmStepMarkerSequence
from Exceptions import ArmObstructedError
from Robot import Robot
from World import World
from condition_types.SpecificObjectCondition import SpecificObjectCondition
import rospy
from pr2_pbd_interaction.msg import ActionStep, ExecutionStatus
from step_types.Step import Step


class ManipulationStep(Step):
    """ Sequence of ArmSteps.
    """


    def __init__(self, step_click_cb):
        self.arm_steps = []
        self.lock = threading.Lock()
        self.conditions = [SpecificObjectCondition()]
        self.step_click_cb = step_click_cb

        if Step.marker_publisher is None:
            Step.marker_publisher = rospy.Publisher(
                'visualization_marker_array', MarkerArray)

        if Step.interactive_marker_server is None:
            im_server = InteractiveMarkerServer('programmed_actions')
            Step.interactive_marker_server = im_server

        self.marker_sequence = ArmStepMarkerSequence(Step.interactive_marker_server, Step.marker_publisher,
                                                     step_click_cb)

    def execute(self):
        for condition in self.conditions:
            try:
                condition.check()
            except:
                rospy.logerr("Condition failed when executing manipulation step.")
                raise
        self.update_objects()
        for step in self.arm_steps:
            # send a request to Robot to move the arms to their respective targets
            robot = Robot.get_robot()
            if (step.type == ActionStep.ARM_TARGET):
                rospy.loginfo('Will perform arm target action step.')

                if (not robot.move_to_joints(step.armTarget.rArm,
                                             step.armTarget.lArm)):
                    robot.status = ExecutionStatus.OBSTRUCTED
                    raise ArmObstructedError()
            # If arm trajectory action
            elif (step.type == ActionStep.ARM_TRAJECTORY):
                rospy.loginfo('Will perform arm trajectory action step.')
                # First move to the start frame
                if (not robot.move_to_joints(step.armTrajectory.r_arm[0],
                                             step.armTrajectory.l_arm[0])):
                    robot.status = ExecutionStatus.OBSTRUCTED
                    raise ArmObstructedError()

                #  Then execute the trajectory
                Robot.arms[0].exectute_joint_traj(step.armTrajectory.r_arm,
                                                  step.armTrajectory.timing)
                Robot.arms[1].exectute_joint_traj(step.armTrajectory.l_arm,
                                                  step.armTrajectory.timing)

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

    def add_arm_step(self, arm_step):
        self.lock.acquire()
        self.arm_steps.append(arm_step)
        # TODO: check: if the arm state is absolute, the object should be None
        self.conditions[0].add_object(arm_step.armTarget.rArm.refFrameObject)
        self.conditions[0].add_object(arm_step.armTarget.lArm.refFrameObject)
        self.marker_sequence.set_total_n_markers(len(self.arm_steps))
        last_step = self.arm_steps[len(self.arm_steps) - 1]
        world_objects = World.get_world().get_frame_list()
        self.marker_sequence.add_arm_step(arm_step, last_step, world_objects)
        self.lock.release()

    def n_frames(self):
        """Returns the number of arm steps in the manipulation step"""
        return len(self.arm_steps)

    def update_objects(self):
        """Updates the object list for all arm steps"""
        self.lock.acquire()
        action_objects = self.conditions[0].get_objects()
        unique_action_objects = self.conditions[0].get_unique_objects()
        world_objects = World.get_world().get_frame_list()
        map_of_objects_old_to_new = World.get_map_of_most_similar_obj(unique_action_objects, world_objects)
        self.marker_sequence.update_objects(action_objects, world_objects, map_of_objects_old_to_new)
        self.lock.release()


    def initialize_viz(self):
        self.lock.acquire()
        action_objects = self.conditions[0].get_objects()
        unique_action_objects = self.conditions[0].get_unique_objects()
        world_objects = World.get_world().get_frame_list()
        map_of_objects_old_to_new = World.get_map_of_most_similar_obj(unique_action_objects, world_objects)
        self.marker_sequence.initialize_viz(self.arm_steps, action_objects, world_objects, map_of_objects_old_to_new)
        self.lock.release()

    def update_viz(self):
        self.lock.acquire()
        self.marker_sequence.update_viz()
        self.lock.release()

    def reset_viz(self):
        self.lock.acquire()
        self.marker_sequence.reset_viz()
        self.lock.release()


    def select_step(self, step_id):
        """ Makes the interactive marker for the indicated arm
        step selected, by showing the 6D controls"""
        self.marker_sequence.select_step(step_id)

    def deselect_step(self, step_id):
        """ Makes the interactive marker for the indicated arm
        step deselected, by removing the 6D controls"""
        self.marker_sequence.deselect_step(step_id)
