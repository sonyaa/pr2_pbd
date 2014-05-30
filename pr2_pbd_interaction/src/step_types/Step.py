#!/usr/bin/env python
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rospy
from visualization_msgs.msg import MarkerArray
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
        self.ignore_conditions = False
        self.conditions = []
        if Step.interactive_marker_server is None:
            im_server = InteractiveMarkerServer('programmed_actions')
            Step.interactive_marker_server = im_server
        if Step.marker_publisher is None:
            Step.marker_publisher = rospy.Publisher(
                'visualization_marker_array', MarkerArray)

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
        self.reset_viz()

    def update_viz(self):
        pass

    def reset_viz(self):
        self.interactive_marker_server.clear()
        self.interactive_marker_server.applyChanges()