#!/usr/bin/env python

class Condition:
    '''
    Checks that there are as many objects present as specified.
    '''
    def __init__(self, n):
        self.num_objects = n


    def check(self):
        # record object poses, verify that there are self.num_objects objects
        return True