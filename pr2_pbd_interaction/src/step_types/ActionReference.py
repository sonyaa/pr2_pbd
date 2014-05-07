#!/usr/bin/env python
from Exceptions import ConditionError
from Robot import Robot
from pr2_pbd_interaction.msg import ExecutionStatus
from step_types.Step import Step

#functional code
from functools import partial

import os.path
from os import listdir
from os.path import isfile, join
import rospy
import yaml


class ActionReference(Step):
    """ Class for referencing a previously saved action by its name or id.
    """
    ACTION_DIRECTORY = "~/pbd_actions/"
    FILE_EXTENSION = ".yaml"

    def __init__(self, *args, **kwargs): #(self, id=None, name=None):
        Step.__init__(self, *args, **kwargs)
        self.name = kwargs.get('name')
        self.id = kwargs.get('id')
        self.steps = []
        self.selected_step_id = None

    def execute(self):
        robot = Robot.get_robot()
        for condition in self.conditions:
            if not condition.check():
                rospy.logwarn("Condition failed when executing action.")
                if self.strategy == Step.STRATEGY_FAILFAST:
                    robot.status = ExecutionStatus.CONDITION_FAILED
                    raise ConditionError()
        for step in self.steps:
            try:
                step.execute()
            except:
                rospy.logerr("Execution of an action failed")
                raise

    def add_step(self, step):
        self.steps.append(step)
        self.select_step(len(self.steps)-1)

    def delete_last_step(self):
        if len(self.steps) > 0:
            if self.selected_step_id == len(self.steps)-1:
                self.reset_viz()
            self.steps.pop()

    def select_step(self, step_id):
        if self.selected_step_id != step_id:
            self.reset_viz()
            self.selected_step_id = step_id
            self.initialize_viz()
        else:
            self.update_viz()

    def get_selected_step(self):
        return self.selected_step_id

    def get_last_step(self):
        if len(self.steps) == 0:
            return None
        return self.steps[len(self.steps)-1]

    def initialize_viz(self):
        if self.selected_step_id is not None:
            self.steps[self.selected_step_id].initialize_viz()

    def update_viz(self):
        if self.selected_step_id is not None:
            self.steps[self.selected_step_id].update_viz()

    def reset_viz(self):
        if self.selected_step_id is not None:
            self.steps[self.selected_step_id].reset_viz()

    @staticmethod
    def get_saved_actions():
        return map(ActionReference.load,
                   filter(lambda f: f.endswith(ActionReference.FILE_EXTENSION),
                          filter(isfile,
                                 map(partial(join, ActionReference.ACTION_DIRECTORY),
                                     listdir(ActionReference.ACTION_DIRECTORY)))))

    @staticmethod
    def get_file(action):
        return ActionReference.ACTION_DIRECTORY + str(action) + ActionReference.FILE_EXTENSION

    @staticmethod
    def get_saved_actions():
        return map(ActionReference.load,
                   filter(lambda f: f.endswith(ActionReference.FILE_EXTENSION),
                          filter(isfile,
                                 map(partial(join, ActionReference.ACTION_DIRECTORY),
                                     listdir(ActionReference.ACTION_DIRECTORY)))))

    @staticmethod
    def load(act_f_id):
        file_path = ""
        if type(act_f_id) is int:
            file_path = ActionReference.get_file(act_f_id)
        else:
            file_path = act_f_id
        act_file = open(file_path, 'r')
        act = ActionReference.from_string(act_file)
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
        return ActionReference.from_string(self.to_string())