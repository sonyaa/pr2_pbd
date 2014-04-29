#!/usr/bin/env python
from step_types import Step


class ArmStep(Step):
    ''' Step that moves the arms.
    '''

    def __init__(self, r_armTarget, l_armTarget):
        self.r_armTarget = r_armTarget
        self.l_armTarget = l_armTarget

    def execute(self):
        # send a request to Robot to move the arms to their respective targets
        pass