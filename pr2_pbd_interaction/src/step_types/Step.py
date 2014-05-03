#!/usr/bin/env python


class Step:
    """ General step of an action
    """

    marker_publisher = None
    interactive_marker_server = None

    def execute(self):
        pass

    def initialize_viz(self):
        pass

    def update_viz(self):
        pass

    def reset_viz(self):
        pass