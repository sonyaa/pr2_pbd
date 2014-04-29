#!/usr/bin/env python
from step_types import Step


class WhileStep(Step):
    ''' Step that contains a loop.
    '''
    def __init__(self, condition, step):
        self.condition = condition
        self.step = step

    def execute(self):
        while self.condition.check():
            self.step.execute()