#!/usr/bin/env python

class Condition:
    '''
    Checks that the objects present are similar enough to the specified ones.
    '''
    def __init__(self, dims):
        self.object_dims = dims


    def check(self):
        # record object poses, verify that the objects are similar in dimensions to self.object_dims
        return True