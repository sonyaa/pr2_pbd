#!/usr/bin/env python
import functools
import threading
import yaml
import time
from ArmStepMarkerSequence import ArmStepMarkerSequence
from Exceptions import ConditionError, UnreachablePoseError, StoppedByUserError
from World import World
from condition_types.IKCondition import IKCondition
from condition_types.SpecificObjectCondition import SpecificObjectCondition
import rospy
from pr2_pbd_interaction.msg import ArmState, ExecutionStatus, ArmMode, Strategy, StepExecutionStatus
from step_types.Step import Step


def manipulation_step_constructor(loader, node):
    from Robot import Robot

    fields = loader.construct_mapping(node, deep=True)
    m_step = ManipulationStep()
    m_step.is_while = fields['is_while']
    m_step.ignore_conditions = fields['ignore_conditions']
    m_step.conditions = fields['conditions']
    m_step.arm_steps = fields['arm_steps']
    m_step.objects = fields.get('objects', [])
    # if the robot hasn't been initialized yet, that means we're on client side, so we don't need anything
    # except arm steps and basic step members
    if len(Robot.arms) == 2:  # if the robot is initialized, construct ArmStepMarkerSequence
        world_objects = World.get_world().get_frame_list()
        m_step.marker_sequence = ArmStepMarkerSequence.construct_from_arm_steps(m_step.interactive_marker_server,
                                                                                m_step.marker_publisher,
                                                                                m_step.step_click_cb, m_step.arm_steps,
                                                                                m_step.reference_change_cb)
    return m_step


yaml.add_constructor(u'!ManipulationStep', manipulation_step_constructor)


class ManipulationStep(Step):
    """ Sequence of ArmSteps.
    """

    def __init__(self, *args, **kwargs):
        from Session import Session
        from Robot import Robot

        Step.__init__(self, *args, **kwargs)
        self.arm_steps = []
        if len(Robot.arms) < 2:
            # if the robot hasn't been initialized yet, that means we're on client side, so we don't need anything
            # except arm steps and basic step members
            return
        self.head_position = Robot.get_head_position()
        self.lock = threading.Lock()
        self.conditions = self.conditions.extend([SpecificObjectCondition(), IKCondition()])
        self.condition_order = xrange(len(self.conditions))
        self.step_click_cb = Session.get_session().selected_arm_step_cb
        self.marker_sequence = ArmStepMarkerSequence(Step.interactive_marker_server, Step.marker_publisher,
                                                     self.step_click_cb, self.reference_change_cb)
        self.objects = []

    def execute(self, action_data):
        from Robot import Robot

        robot = Robot.get_robot()
        # If self.is_while, execute everything in a loop until a condition fails. Else execute everything once.
        while True:
            if not self.ignore_conditions:
                for condition in [self.conditions[i] for i in self.condition_order]:
                    if isinstance(condition, IKCondition):
                        condition.set_step(self.copy())
                    if not condition.check():
                        rospy.logwarn("Condition failed when executing manipulation step.")
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
                rospy.loginfo('Ignoring conditions for manipulation step')
            self.update_objects()
            self.initialize_viz()
            step_to_execute = self.copy()
            if not robot.solve_ik_for_manipulation_step(step_to_execute):
                # Shouldn't get here, this was supposed to be checked by IKCondition.
                rospy.logwarn('Problems in finding IK solutions...')
                robot.status = ExecutionStatus.NO_IK
                rospy.logerr("Execution of a manipulation step failed, unreachable poses.")
                self.execution_status = StepExecutionStatus.FAILED
                return
            else:
                Robot.set_arm_mode(0, ArmMode.HOLD)
                Robot.set_arm_mode(1, ArmMode.HOLD)
                for (i, step) in enumerate(step_to_execute.arm_steps):
                    if robot.preempt:
                        # robot.preempt = False
                        robot.status = ExecutionStatus.PREEMPTED
                        rospy.logerr('Execution of manipulation step failed, execution preempted by user.')
                        raise StoppedByUserError()
                    try:
                        step.execute()
                        rospy.loginfo('Step ' + str(i) + ' of manipulation step is complete.')
                    except:
                        rospy.logerr("Execution of a manipulation step failed")
                        self.execution_status = StepExecutionStatus.FAILED
                        return

            Robot.arms[0].reset_movement_history()
            Robot.arms[1].reset_movement_history()

            self.execution_status = StepExecutionStatus.SUCCEEDED

            if not self.is_while:
                return
            # If the manipulation step needs objects and we're in a while loop, look for objects again.
            else:
                need_objects = False
                for condition in self.conditions:
                    if isinstance(condition, SpecificObjectCondition):
                        if not condition.is_empty():
                            need_objects = True
                if need_objects:
                    robot.move_head_to_point(self.head_position)
                    world = World.get_world()
                    if not world.update_object_pose():
                        rospy.logwarn("Object detection failed.")
                        return
                    # Wait for all objects to be detected.
                    time.sleep(1)

    def add_arm_step(self, arm_step):
        self.lock.acquire()
        r_object = None
        l_object = None
        self.arm_steps.append(arm_step.copy())
        if arm_step.armTarget.rArm.refFrame == ArmState.OBJECT:
            r_object = arm_step.armTarget.rArm.refFrameObject
        self.objects.append(r_object)
        if arm_step.armTarget.lArm.refFrame == ArmState.OBJECT:
            l_object = arm_step.armTarget.lArm.refFrameObject
        self.objects.append(l_object)
        for condition in self.conditions:
            if isinstance(condition, SpecificObjectCondition):
                condition.add_object(r_object)
                condition.add_object(l_object)
                break
        cur_step = self.arm_steps[len(self.arm_steps) - 1]
        world_objects = World.get_world().get_frame_list()
        self.marker_sequence.add_arm_step(cur_step, world_objects)
        self.marker_sequence.set_total_n_markers(len(self.arm_steps))
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
        return self.arm_steps[len(self.arm_steps) - 1]

    def n_steps(self):
        """Returns the number of arm steps in the manipulation step"""
        return len(self.arm_steps)

    def update_objects(self):
        """Updates the object list for all arm steps"""
        self.lock.acquire()
        has_real_objects = True
        world_objects = World.get_world().get_frame_list()
        rospy.loginfo([x.name for x in world_objects])
        action_objects = None
        map_of_objects_old_to_new = None
        for condition in self.conditions:
            if isinstance(condition, SpecificObjectCondition):
                action_objects = condition.get_objects()
                unique_action_objects = condition.get_unique_objects()
                map_of_objects_old_to_new = World.get_map_of_most_similar_obj(unique_action_objects, world_objects,
                                                                              threshold=self.conditions[
                                                                                  0].similarity_threshold)
                if map_of_objects_old_to_new is None and len(unique_action_objects) > 0:
                    world_objects = self.get_unique_objects()
                    rospy.loginfo('fake objects')
                    rospy.loginfo([x.name for x in world_objects])
                    map_of_objects_old_to_new = World.get_map_of_most_similar_obj(unique_action_objects, world_objects)
                    has_real_objects = False
                break
        self.marker_sequence.update_objects(action_objects, world_objects, map_of_objects_old_to_new, has_real_objects)
        self.lock.release()

    def initialize_viz(self):
        self.lock.acquire()
        has_real_objects = True
        world_objects = World.get_world().get_frame_list()
        if len(world_objects) == 0 and len(self.get_unique_objects()) > 0:
            World.get_world().add_fake_objects(self.get_unique_objects())
            world_objects = World.get_world().get_frame_list()
            has_real_objects = False
        action_objects = None
        map_of_objects_old_to_new = None
        for condition in self.conditions:
            if isinstance(condition, SpecificObjectCondition):
                action_objects = condition.get_objects()
                unique_action_objects = condition.get_unique_objects()
                map_of_objects_old_to_new = World.get_map_of_most_similar_obj(unique_action_objects, world_objects,
                                                                              threshold=condition.similarity_threshold)
                break
        self.marker_sequence.initialize_viz(self.arm_steps, action_objects, world_objects,
                                            map_of_objects_old_to_new, has_real_objects)
        self.lock.release()

    def get_unique_objects(self):
        objects = []
        object_names = []
        for obj in self.objects:
            if obj is not None and obj.name not in object_names:
                object_names.append(obj.name)
                objects.append(obj)
        return objects

    def update_viz(self):
        self.lock.acquire()
        self.marker_sequence.update_viz()
        self.lock.release()

    def reset_viz(self):
        self.lock.acquire()
        World.get_world().remove_fake_objects()
        #World.get_world().clear_all_objects()
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
            rospy.loginfo('Deleting arm step ' + str(to_delete))
            self.arm_steps.pop(to_delete)
            # Deleting two objects that correspond to rArm and lArm for specified step.
            for condition in self.conditions:
                if isinstance(condition, SpecificObjectCondition):
                    if len(condition.objects) > 2 * to_delete:
                        condition.objects.pop(2 * to_delete + 1)
                        condition.objects.pop(2 * to_delete)
                    else:
                        rospy.logwarn('Condition for manipulation step has invalid number of objects.')
                    break
        self.lock.release()

    def delete_arm_step(self, step_id):
        """ Delete specified step and update visualization.
        """
        self.lock.acquire()
        self.marker_sequence.delete_step(step_id)
        self.arm_steps.pop(step_id)
        rospy.loginfo('Deleting arm step ' + str(step_id))
        # Deleting two objects that correspond to rArm and lArm for specified step.
        for condition in self.conditions:
            if isinstance(condition, SpecificObjectCondition):
                if len(condition.objects) > 2 * step_id:
                    condition.objects.pop(2 * step_id + 1)
                    condition.objects.pop(2 * step_id)
                else:
                    rospy.logwarn('Condition for manipulation step has invalid number of objects.')
                break
        self.lock.release()

    def delete_last_step_and_update_viz(self):
        self.delete_arm_step(len(self.arm_steps) - 1)

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

    def set_ignore_arm_step_conditions(self, index, ignore_conditions):
        if len(self.arm_steps) > 0 and index < len(self.arm_steps):
            self.arm_steps[index].ignore_conditions = ignore_conditions

    def reference_change_cb(self, uid, new_ref, new_ref_obj):
        if new_ref == ArmState.OBJECT:
            self.objects[uid] = new_ref_obj
        else:
            self.objects[uid] = None
        for condition in self.conditions:
            if isinstance(condition, SpecificObjectCondition):
                if new_ref == ArmState.OBJECT:
                    condition.objects[uid] = new_ref_obj
                else:
                    condition.objects[uid] = None
                rospy.loginfo("Changed reference object in SpecificObjectCondition")
                break

    def copy(self):
        copy = ManipulationStep()
        copy.conditions = self.conditions
        copy.is_while = self.is_while
        copy.ignore_conditions = self.ignore_conditions
        copy.objects = self.objects
        for step in self.arm_steps:
            copy.arm_steps.append(step.copy())
        return copy


def manipulation_step_representer(dumper, data):
    return dumper.represent_mapping(u'!ManipulationStep', {'is_while': data.is_while,
                                                           'ignore_conditions': data.ignore_conditions,
                                                           'conditions': data.conditions,
                                                           'arm_steps': data.arm_steps,
                                                           'objects': data.objects})

yaml.add_representer(ManipulationStep, manipulation_step_representer)