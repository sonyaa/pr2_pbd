#!/usr/bin/env python
import threading
from Exceptions import ConditionError, StoppedByUserError
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from World import World
from condition_types.SpecificObjectCondition import SpecificObjectCondition
from pr2_pbd_interaction.msg import ExecutionStatus
from pr2_pbd_interaction.msg._StepExecutionStatus import StepExecutionStatus
from pr2_pbd_interaction.msg._Strategy import Strategy
from step_types.ManipulationStep import ManipulationStep
from step_types.Step import Step

#functional code
from functools import partial

import os.path
from os import listdir
from os.path import isfile, join
import rospy
import yaml


class ActionData:
    def __init__(self):
        self.go_back = False


class Action(Step):
    """ Class for referencing a previously saved action by its name or id.
    """
    #TODO: the directory stuff should probably be moved to another place - Session?
    ACTION_DIRECTORY = "/home/sonyaa/pbd_actions/"
    FILE_EXTENSION = ".yaml"

    def __init__(self, *args, **kwargs):  #(self, id=None, name=None):
        Step.__init__(self, *args, **kwargs)
        self.name = kwargs.get('name')
        self.id = kwargs.get('id')
        self.steps = []
        self.selected_step_id = -1
        self.lock = threading.Lock()
        #TODO: only lock locks when you really need to
        self.action_data = ActionData()

    def execute(self, action_data=None):
        from Robot import Robot

        robot = Robot.get_robot()
        # If self.is_while, execute everything in a loop until a condition fails. Else execute everything once.
        while True:
            if not self.ignore_conditions:
                for condition in [self.conditions[i] for i in self.condition_order]:
                    if not condition.check():
                        rospy.logwarn("Condition failed when executing action.")
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
                            if action_data is not None:
                                action_data.go_back = True
                            else:
                                rospy.logwarn("Cannot go to previous step, no action data provided.")
                            return
                        else:
                            rospy.logwarn("Unknown strategy " + str(self.strategy))
            else:
                rospy.loginfo('Ignoring conditions for action step')
            cur_status = None
            i = 0
            while i < len(self.steps):
                step = self.steps[i]
                step.set_previous_step_status(cur_status)
                if robot.preempt:
                    # robot.preempt = False
                    robot.status = ExecutionStatus.PREEMPTED
                    rospy.logerr('Execution of action step failed, execution preempted by user.')
                    raise StoppedByUserError()
                try:
                    if robot.status == ExecutionStatus.EXECUTING:
                        step.execute(self.action_data)
                        cur_status = step.get_execution_status()
                        rospy.loginfo('Step ' + str(i) + ' of action step is complete. Status is ' + str(cur_status))
                        if self.action_data.go_back:
                            i -= 1
                            rospy.loginfo('Going back to previous step: step number ' + str(i))
                        else:
                            i += 1

                except Exception as e:
                    rospy.logerr("Execution of an action failed: " + str(e))
                    self.execution_status = StepExecutionStatus.FAILED
                    return

            self.execution_status = StepExecutionStatus.SUCCEEDED

            if not self.is_while:
                return

    def get_lock(self):
        try:
            return self.lock
        except AttributeError:
            self.lock = threading.Lock()
            return self.lock

    def add_step(self, step):
        # self.get_lock().acquire()
        self.steps.append(step)
        self.select_step(len(self.steps) - 1)
        # self.get_lock().release()

    def delete_last_step(self):
        self.delete_step(len(self.steps) - 1)

    def delete_step(self, index):
        self.get_lock().acquire()
        if len(self.steps) > 0 and index < len(self.steps):
            self.reset_viz()
            self.selected_step_id = -1
            self.steps.pop(index)
        self.get_lock().release()

    def set_loop_step(self, index, is_loop):
        self.get_lock().acquire()
        if len(self.steps) > 0 and index < len(self.steps):
            self.steps[index].is_while = is_loop
        self.get_lock().release()

    def set_ignore_conditions(self, index, ignore_conditions):
        self.get_lock().acquire()
        if len(self.steps) > 0 and index < len(self.steps):
            self.steps[index].ignore_conditions = ignore_conditions
        self.get_lock().release()

    def set_object_similarity_threshold(self, index, threshold):
        self.get_lock().acquire()
        if len(self.steps) > 0 and index < len(self.steps):
            done = False
            for condition in self.steps[index].conditions:
                if isinstance(condition, SpecificObjectCondition):
                    condition.set_similarity_threshold(threshold)
                    done = True
            if not done:
                rospy.logwarn(
                    "Couldn't set object similarity threshold because the step has no SpecificObjectCondition.")
        self.get_lock().release()

    def set_step_condition_order(self, index, cond_order):
        self.get_lock().acquire()
        if len(self.steps) > 0 and index < len(self.steps):
            self.steps[index].set_condition_order(cond_order)
        self.get_lock().release()

    def select_step(self, step_id):
        # self.get_lock().acquire()
        rospy.loginfo(self.selected_step_id)
        rospy.loginfo(step_id)
        if self.selected_step_id != step_id:
            self.reset_viz()
            self.selected_step_id = step_id
            self.initialize_viz()
        else:
            self.update_viz()
            # self.get_lock().release()

    def get_selected_step_id(self):
        return self.selected_step_id

    def get_selected_step(self):
        # self.get_lock().acquire()
        step = None
        if len(self.steps) > 0 and self.selected_step_id >= 0:
            step = self.steps[self.selected_step_id]
        # self.get_lock().release()
        return step

    def select_arm_step(self, step_id):
        """ Makes the interactive marker for the indicated arm
        step selected, by showing the 6D controls.
        Only works if the current step is manipulation."""
        current_step = self.get_selected_step()
        self.get_lock().acquire()
        if current_step is not None and isinstance(current_step, ManipulationStep):
            current_step.select_step(step_id)
        self.get_lock().release()

    def deselect_arm_step(self, step_id):
        """ Makes the interactive marker for the indicated arm
        step deselected, by removing the 6D controls.
        Only works if the current step is manipulation."""
        current_step = self.get_selected_step()
        self.get_lock().acquire()
        if current_step is not None and isinstance(current_step, ManipulationStep):
            current_step.deselect_step(step_id)
        self.get_lock().release()

    def get_last_step(self):
        self.get_lock().acquire()
        step = None
        if len(self.steps) > 0:
            step = self.steps[len(self.steps) - 1]
        self.get_lock().release()
        return step

    def initialize_viz(self):
        self.reset_viz()
        self.get_lock().acquire()
        if len(self.steps) > 0 and self.selected_step_id >= 0:
            selected_step = self.steps[self.selected_step_id]
            if not isinstance(selected_step, Action):
                selected_step.initialize_viz()
            if not isinstance(selected_step, ManipulationStep):
                World.get_world().clear_all_objects()
        self.get_lock().release()

    def update_viz(self):
        self.get_lock().acquire()
        if len(self.steps) > 0 and self.selected_step_id >= 0:
            if not isinstance(self.steps[self.selected_step_id], Action):
                self.steps[self.selected_step_id].update_viz()
        self.get_lock().release()

    def clear(self):
        self.reset_viz()
        self.lock.acquire()
        del self.steps[:]
        self.selected_step_id = -1
        self.lock.release()

    def reset_viz(self):
        # self.get_lock().acquire()
        if len(self.steps) > 0 and self.selected_step_id >= 0:
            self.steps[self.selected_step_id].reset_viz()
        self.interactive_marker_server.clear()
        self.interactive_marker_server.applyChanges()
        # self.get_lock().release()

    def update_objects(self):
        cur_step = self.get_selected_step()
        if isinstance(cur_step, ManipulationStep):
            cur_step.update_objects()

    def n_steps(self):
        return len(self.steps)

    def get_name(self):
        if self.name is not None:
            return self.name
        else:
            if self.id is None:
                self.id = 0
                while os.path.isfile(self.get_file(self.id)):
                    self.id += 1
            return 'Action ' + str(self.id)

    @staticmethod
    def get_file(action):
        return Action.ACTION_DIRECTORY + str(action) + Action.FILE_EXTENSION

    @staticmethod
    def get_saved_actions():
        actions = map(Action.load,
                      filter(lambda f: f.endswith(Action.FILE_EXTENSION),
                             filter(isfile,
                                    map(partial(join, Action.ACTION_DIRECTORY),
                                        listdir(Action.ACTION_DIRECTORY)))))
        actions.sort(key=lambda action: action.id)
        return actions

    @staticmethod
    def load(act_f_id):
        file_path = ""
        if type(act_f_id) is int:
            file_path = Action.get_file(act_f_id)
        else:
            file_path = act_f_id
        act_file = open(file_path, 'r')
        act = Action.from_string(act_file)
        act_file.close()
        return act

    @staticmethod
    def from_string(str):
        return yaml.load(str)

    def save(self):
        '''saves action to file'''
        if self.id is None:
            self.id = 0
            while os.path.isfile(self.get_file(self.id)):
                self.id += 1
        act_file = open(self.get_file(self.id), 'w')
        act_file.write(self.to_string())
        act_file.close()

    def to_string(self):
        '''gets the yaml representing this action'''
        return yaml.dump(self)

    def copy(self):
        return Action.from_string(self.to_string())


def action_step_constructor(loader, node):
    fields = loader.construct_mapping(node, deep=True)
    step = Action(fields)
    step.is_while = fields['is_while']
    step.ignore_conditions = fields['ignore_conditions']
    step.conditions = fields['conditions']
    step.condition_order = fields['condition_order']
    step.steps = fields['steps']
    step.selected_step_id = fields['selected_step_id']
    step.name = fields.get('name')
    step.id = fields.get('id')
    return step


yaml.add_constructor(u'!ActionStep', action_step_constructor)


def action_step_representer(dumper, data):
    return dumper.represent_mapping(u'!ActionStep', {'is_while': data.is_while,
                                                     'ignore_conditions': data.ignore_conditions,
                                                     'conditions': data.conditions,
                                                     'condition_order': data.condition_order,
                                                     'steps': data.steps,
                                                     'selected_step_id': data.selected_step_id,
                                                     'name': data.name,
                                                     'id': data.id})


yaml.add_representer(Action, action_step_representer)