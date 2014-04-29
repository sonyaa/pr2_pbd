#!/usr/bin/env python
from step_types import Step


class StepSequence(Step):
    ''' Sequence of steps.
    '''

    def __init__(self, steps):
        self.steps = steps

    def execute(self):
        for step in self.steps:
            step.execute()