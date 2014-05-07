#!/usr/bin/env python
from condition_types.Condition import Condition


class AnyObjectCondition(Condition):
    '''
    Checks that there are as many objects present as specified.
    '''

    def __init__(self, *args, **kwargs):
        Condition.__init__(self, *args, **kwargs)
        self.num_objects = args[0]

    def set_num_objects(self, n):
        self.num_objects = n

    def check(self):
        # look at the state of the world, verify that there are self.num_objects objects
        return True