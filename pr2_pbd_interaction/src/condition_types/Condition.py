#!/usr/bin/env python

class Condition:
    def __init__(self, *args, **kwargs):
        pass

    def check(self):
        """ Returns True if condition is satisfied, False otherwise.
        """
        return True

    def __repr__(self):
        return "%s" % (self.__class__.__name__)