#!/usr/bin/env python
import threading
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import MarkerArray
from ArmStepMarkerSequence import ArmStepMarkerSequence
from Exceptions import ConditionError, UnreachablePoseError, StoppedByUserError
from Robot import Robot
from World import World
from condition_types.GripperCondition import GripperCondition
from condition_types.SpecificObjectCondition import SpecificObjectCondition
import rospy
from pr2_pbd_interaction.msg import ArmState, ExecutionStatus, ArmMode
from step_types.Step import Step


class ManipulationStep(Step):
    """ Sequence of ArmSteps.
    """

    def __init__(self, *args, **kwargs):
        Step.__init__(self, *args, **kwargs)
        self.arm_steps = []
        self.lock = threading.Lock()
        self.conditions = [SpecificObjectCondition()]
        self.step_click_cb = args[0]
        if Step.marker_publisher is None:
            Step.marker_publisher = rospy.Publisher(
                'visualization_marker_array', MarkerArray)
        if Step.interactive_marker_server is None:
            im_server = InteractiveMarkerServer('programmed_actions')
            Step.interactive_marker_server = im_server
        self.marker_sequence = ArmStepMarkerSequence(Step.interactive_marker_server, Step.marker_publisher,
                                                     self.step_click_cb)
        self.initial_condition = GripperCondition()

    def execute(self):
        robot = Robot.get_robot()
        for condition in self.conditions:
            if not condition.check():
                rospy.logwarn("Condition failed when executing manipulation step.")
                if self.strategy == Step.STRATEGY_FAILFAST:
                    robot.status = ExecutionStatus.CONDITION_FAILED
                    raise ConditionError()
        self.update_objects()
        if not robot.solve_ik_for_manipulation_step(self):
            rospy.logwarn('Problems in finding IK solutions...')
            robot.status = ExecutionStatus.NO_IK
            rospy.logerr("Execution of a manipulation step failed, unreachable poses.")
            raise UnreachablePoseError()
        else:
            Robot.set_arm_mode(0, ArmMode.HOLD)
            Robot.set_arm_mode(1, ArmMode.HOLD)
            for (i, step) in enumerate(self.arm_steps):
                try:
                    if robot.status == ExecutionStatus.EXECUTING:
                        step.execute()
                    if robot.preempt:
                        robot.preempt = False
                        robot.status = ExecutionStatus.PREEMPTED
                        rospy.logerr('Execution of manipulation step failed, execution preempted by user.')
                        raise StoppedByUserError()
                    rospy.loginfo('Step ' + str(i) + ' of manipulation step is complete.')
                except:
                    rospy.logerr("Execution of a manipulation step failed")
                    raise

        Robot.arms[0].reset_movement_history()
        Robot.arms[1].reset_movement_history()

    def add_arm_step(self, arm_step):
        self.lock.acquire()
        r_object = None
        l_object = None
        self.arm_steps.append(arm_step.copy())
        if arm_step.armTarget.rArm.refFrame == ArmState.OBJECT:
            r_object = arm_step.armTarget.rArm.refFrameObject
        if arm_step.armTarget.lArm.refFrame == ArmState.OBJECT:
            l_object = arm_step.armTarget.lArm.refFrameObject
        self.conditions[0].add_object(r_object)
        self.conditions[0].add_object(l_object)
        cur_step = self.arm_steps[len(self.arm_steps) - 1]
        world_objects = World.get_world().get_frame_list()
        self.marker_sequence.add_arm_step(cur_step, world_objects)
        self.marker_sequence.set_total_n_markers(len(self.arm_steps))
        if len(self.arm_steps) > 1:
            prev_step = self.arm_steps[len(self.arm_steps) - 2]
            cur_step.set_gripper_condition(self.initial_condition if prev_step is None else prev_step.postCond)
            cur_step.postCond = GripperCondition(Robot.get_robot().get_gripper_position(0),
                                                 Robot.get_robot().get_gripper_position(1))
        self.lock.release()

    def delete_last_step_and_update_viz(self):
        self.lock.acquire()
        self.marker_sequence.delete_step(len(self.arm_steps) - 1)
        self.arm_steps.pop()
        # Deleting two last objects that correspond to rArm and lArm for last step.
        self.conditions[0].delete_last_object()
        self.conditions[0].delete_last_object()
        self.lock.release()


    def get_step(self, index):
        '''Returns a step of the manipulation'''
        self.lock.acquire()
        requested_step = self.arm_steps[index]
        self.lock.release()
        return requested_step

    def get_last_step(self):
        """ Returns the last step if there exists one and none otherwise.
        """
        if len(self.arm_steps) == 0:
            return None
        return self.arm_steps[len(self.arm_steps)-1]

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

    def reset_targets(self, arm_index):
        """Resets requests after reaching a previous target"""
        self.lock.acquire()
        self.marker_sequence.reset_targets(arm_index)
        self.lock.release()

    def delete_requested_steps(self):
        """Delete steps that were requested from interactive
        marker menus"""
        self.lock.acquire()
        to_delete = self.marker_sequence.delete_requested_steps()
        if to_delete is not None:
            self.delete_arm_step(to_delete)
        self.lock.release()

    def delete_arm_step(self, to_delete):
        self.arm_steps.pop(to_delete)

    def change_requested_steps(self, r_arm, l_arm):
        """Change an arm step to the current end effector
        pose if requested through the interactive marker menu"""
        self.lock.acquire()
        self.marker_sequence.change_requested_steps(r_arm, l_arm)
        self.lock.release()

    def get_requested_targets(self, arm_index):
        """Get arm steps that might have been requested from
        the interactive marker menus"""
        self.lock.acquire()
        pose = self.marker_sequence.get_requested_targets(arm_index)
        self.lock.release()
        return pose