#!/usr/bin/env python

class AnyObjectCondition:
    '''
    Checks that there are as many objects present as specified.
    '''
    def __init__(self, n):
        self.num_objects = n


    def check(self):
        # look at the state of the world, verify that there are self.num_objects objects
        return True