#!/usr/bin/env python
from step_types import Step


class IfStep(Step):
    ''' Step that contains a condition.
    '''

    def __init__(self, if_condition, if_step, else_step):
        self.if_condition = if_condition
        self.if_step = if_step
        self.else_step = else_step

    def execute(self):
        if self.if_condition.check():
            self.if_step.execute()
        else:
            self.else_step.execute()