#!/usr/bin/env python
from pr2_pbd_interaction.msg import Strategy


class Condition:
    def __init__(self, *args, **kwargs):
        self.available_strategies = [Strategy.FAIL_FAST]
        self.current_strategy_index = 0

    def check(self):
        """ Returns True if condition is satisfied, False otherwise.
        """
        return True

    def set_strategy_index(self, index):
        self.current_strategy_index = index

    def __repr__(self):
        return "%s" % (self.__class__.__name__)