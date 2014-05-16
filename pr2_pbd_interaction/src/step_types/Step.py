#!/usr/bin/env python
from pr2_pbd_interaction.msg import Strategy


class Step:
    """ General step of an action
    """

    marker_publisher = None
    interactive_marker_server = None

    def __init__(self, *args, **kwargs):
        self.strategy = Strategy.FAIL_FAST
        # If self.is_while, execute step in a loop until a condition fails. Else execute step once.
        self.is_while = False
        self.conditions = []

    def set_is_while(self, is_while):
        self.is_while = is_while

    def set_strategy(self, strategy):
        self.strategy = strategy

    def add_condition(self, condition):
        self.conditions.append(condition)

    def remove_condition(self, index):
        self.conditions.pop(index)

    def execute(self):
        pass

    def initialize_viz(self):
        pass

    def update_viz(self):
        pass

    def reset_viz(self):
        pass