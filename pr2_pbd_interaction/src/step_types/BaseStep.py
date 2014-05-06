#!/usr/bin/env python
from Exceptions import BaseObstructedError
from Robot import Robot
from step_types.Step import Step


class BaseStep(Step):
    ''' Step that moves the base.
    '''

    def __init__(self, *args, **kwargs): #(self, end_pose):
        Step.__init__(self, *args, **kwargs)
        self.end_pose = args[0]

    def execute(self):
        # send a request to Robot to move to end_pose
        if not Robot.get_robot().move_base(self.end_pose):
           raise BaseObstructedError()