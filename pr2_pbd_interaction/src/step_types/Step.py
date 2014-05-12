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
    
    def __repr__(self):
        return "%s(strategy=%r, is_while=%r, conditions=%r)" % (
            self.__class__.__name__, self.strategy, self.is_while, self.conditions)