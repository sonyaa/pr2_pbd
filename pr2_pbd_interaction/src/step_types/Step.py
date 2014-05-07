#!/usr/bin/env python


class Step:
    """ General step of an action
    """

    STRATEGY_FAILFAST = 0   # If any condition is not satisfied, fail immediately.
    STRATEGY_CONTINUE = 1   # If any condition is not satisfied, continue.

    marker_publisher = None
    interactive_marker_server = None

    def __init__(self, *args, **kwargs):
        self.strategy = Step.STRATEGY_FAILFAST
        self.conditions = []

    def set_strategy(self, strategy):
        self.strategy = strategy

    def add_condition(self, condition):
        self.conditions.append(condition)

    def execute(self):
        pass

    def initialize_viz(self):
        pass

    def update_viz(self):
        pass

    def reset_viz(self):
        pass